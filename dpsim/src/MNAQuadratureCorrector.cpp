// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <algorithm>
#include <chrono>
#include <cmath>
#include <type_traits>

#include <Eigen/SparseLU>

#include <dpsim-models/IdentifiedObject.h>
#include <dpsim-models/MathUtils.h>
#include <dpsim/MNAQuadratureCorrector.h>

using namespace DPsim;
using namespace CPS;

namespace {

Complex elementAt(const Matrix &vector, Int node) {
  return node < 0 ? Complex(0, 0)
                  : Math::complexFromVectorElement(vector, node);
}

} // namespace

template <typename VarType>
void MNAQuadratureCorrector<VarType>::initialize(
    const typename CPS::MNAQuadratureCorrectionInterface<VarType>::List &states,
    UInt mnaVectorSize, UInt numFreqs, Real omega, Real timeStep,
    CPS::Logger::Log log) {
  mSLog = log;
  mMnaVectorSize = mnaVectorSize;
  mNumFreqs = numFreqs;
  mOmega = omega;
  mTimeStep = timeStep;
  mEntries.clear();
  mComponentEntries.clear();

  for (auto state : states) {
    if (!state)
      continue;

    auto idObj = std::dynamic_pointer_cast<IdentifiedObject>(state);
    const String name = idObj ? **idObj->mName : "<unnamed>";
    const UInt count = state->mnaCorrectionStateCount();

    mComponentEntries.emplace_back();

    for (UInt index = 0; index < count; ++index) {
      Entry entry;
      entry.state = state;
      entry.index = index;
      entry.name = count > 1 ? name + "[" + std::to_string(index) + "]" : name;
      mComponentEntries.back().push_back(static_cast<UInt>(mEntries.size()));
      mEntries.push_back(entry);
    }
  }

  mInitialized = !mEntries.empty();

  SPDLOG_LOGGER_INFO(mSLog,
                     "Initialized MNA quadrature corrector with {:d} "
                     "correctable states.",
                     mEntries.size());
}

template <typename VarType>
UInt MNAQuadratureCorrector<VarType>::correct(
    const SparseMatrix &systemMatrix, Real time,
    const std::vector<CPS::MNASwitchInterface::SwitchedPole> &switchedPoles) {
  if (!mInitialized || !mEnabled || switchedPoles.empty())
    return 0;

  ++mCallCount;

  const auto wallStart = std::chrono::steady_clock::now();

  const UInt changed = correctQuadrature(systemMatrix, time, switchedPoles);
  mCorrectedStates += changed;

  const std::chrono::duration<Real> elapsed =
      std::chrono::steady_clock::now() - wallStart;
  mCumulativeTime += elapsed.count();

  return changed;
}

template <typename VarType>
UInt MNAQuadratureCorrector<VarType>::correctQuadrature(
    const SparseMatrix &systemMatrix, Real time,
    const std::vector<CPS::MNASwitchInterface::SwitchedPole> &switchedPoles) {
  if constexpr (!std::is_same<VarType, Complex>::value) {
    return 0;
  } else {
    if (mNumFreqs != 1) {
      SPDLOG_LOGGER_WARN(
          mSLog,
          "Quadrature correction skipped at t={:.9f}s: derived for a single "
          "envelope frequency, but {:d} are in use.",
          time, mNumFreqs);
      return 0;
    }

    for (auto &entry : mEntries)
      entry.state->mnaCorrectionNodes(entry.index, entry.node0, entry.node1);

    // The step matrix with steady-state admittances for the storage elements
    // is the post-event network at the shift frequency.
    SparseMatrixRow steady = systemMatrix;
    auto stamp = [&](Int row, Int col, Complex value) {
      if (row >= 0 && col >= 0)
        Math::addToMatrixElement(steady, row, col, value);
    };

    for (const auto &members : mComponentEntries)
      for (auto rowIdx : members)
        for (auto colIdx : members) {
          const Entry &row = mEntries[rowIdx];
          const Entry &col = mEntries[colIdx];
          const Complex delta =
              row.state->mnaCorrectionSteadyStateAdmittance(row.index,
                                                            col.index, mOmega) -
              row.state->mnaCorrectionCouplingAdmittance(row.index, col.index);

          if (delta == Complex(0.0, 0.0))
            continue;

          // The state's current leaves terminal 1 and enters terminal 0.
          stamp(row.node1, col.node1, delta);
          stamp(row.node0, col.node0, delta);
          stamp(row.node1, col.node0, -delta);
          stamp(row.node0, col.node1, -delta);
        }

    for (const auto &component : mEventStamps)
      component->mnaApplyEventStamp(steady, mOmega);

    SparseMatrix steadyColumns = steady;
    Eigen::SparseLU<SparseMatrix> lu;
    lu.analyzePattern(steadyColumns);
    lu.factorize(steadyColumns);
    ++mQuadratureSolves;

    if (lu.info() != Eigen::Success) {
      SPDLOG_LOGGER_WARN(mSLog,
                         "Quadrature correction skipped at t={:.9f}s: the "
                         "steady-state post-event matrix is singular.",
                         time);
      return 0;
    }

    // One column per pole: the interrupted current for an opening, the current
    // the shorted voltage drives for a closing.
    const auto poleCount = static_cast<Eigen::Index>(switchedPoles.size());
    Matrix rhs = Matrix::Zero(mMnaVectorSize, poleCount);
    Real largestPoleCurrent = 0.0;
    Bool anyClosed = false;

    // The corrected states belong to the step the poles switched in, so the
    // quadrature is taken against that step's carrier, also for an opening:
    // the exact current zero lies up to a step earlier and would shift the
    // in-phase part of the states.
    const Complex carrier = std::polar(1.0, mOmega * (time - mTimeStep));

    for (std::size_t p = 0; p < switchedPoles.size(); ++p) {
      const auto &pole = switchedPoles[p];
      const Complex injection =
          pole.closed
              ? -(pole.closedConductance - pole.openConductance) * pole.voltage
              : pole.current;
      const Real magnitude = std::abs(injection);
      if (magnitude == 0.0)
        continue;

      Matrix column = Matrix::Zero(mMnaVectorSize, 1);
      if (pole.node1 >= 0)
        Math::addToVectorElement(column, pole.node1, injection);
      if (pole.node0 >= 0)
        Math::addToVectorElement(column, pole.node0, -injection);
      rhs.col(static_cast<Eigen::Index>(p)) = column;

      if (pole.closed)
        anyClosed = true;

      largestPoleCurrent = std::max(largestPoleCurrent, magnitude);

      SPDLOG_LOGGER_INFO(mSLog,
                         "Quadrature correction at t={:.9f}s: pole {:s}, "
                         "current |{:.6e}|.",
                         time, pole.closed ? "closed" : "opened", magnitude);
    }

    if (largestPoleCurrent == 0.0)
      return 0;

    const Matrix change = lu.solve(rhs);

    UInt changed = 0;

    for (const auto &members : mComponentEntries)
      for (auto rowIdx : members) {
        const Entry &row = mEntries[rowIdx];
        const auto kind = row.state->mnaCorrectionStateKind();
        const Bool isCapacitor =
            kind == CPS::MNAQuadratureCorrectionInterface<
                        VarType>::StateKind::CapacitorVoltage;

        // Inductors take the opening poles, capacitors the closing ones
        if (!row.state->mnaCorrectionAllowed() || (isCapacitor && !anyClosed) ||
            (!isCapacitor && kind != CPS::MNAQuadratureCorrectionInterface<
                                         VarType>::StateKind::InductorCurrent))
          continue;

        Complex quadrature(0.0, 0.0);

        for (std::size_t p = 0; p < switchedPoles.size(); ++p) {
          if (isCapacitor != switchedPoles[p].closed)
            continue;

          const Matrix column = change.col(static_cast<Eigen::Index>(p));
          Complex stateChange(0.0, 0.0);
          if (isCapacitor) {
            stateChange =
                elementAt(column, row.node1) - elementAt(column, row.node0);
          } else {
            for (auto colIdx : members) {
              const Entry &col = mEntries[colIdx];
              stateChange +=
                  row.state->mnaCorrectionSteadyStateAdmittance(
                      row.index, col.index, mOmega) *
                  (elementAt(column, col.node1) - elementAt(column, col.node0));
            }
          }

          // Only the quadrature part is imposed, the transient brings the rest
          quadrature += Complex(0.0, (stateChange * carrier).imag()) *
                        std::conj(carrier);
        }

        if (quadrature == Complex(0.0, 0.0))
          continue;

        const Complex before = row.state->mnaCorrectionState(row.index);
        row.state->mnaCorrectionSetState(row.index, before + quadrature);
        row.state->mnaCorrectionRebuildRightVector();
        ++changed;

        if (std::abs(quadrature) >= 1e-3 * largestPoleCurrent)
          SPDLOG_LOGGER_INFO(mSLog,
                             "Quadrature correction at t={:.9f}s: '{:s}' "
                             "changed by {:.3e} of the pole current, from "
                             "|{:.6e}| to |{:.6e}|.",
                             time, row.name,
                             std::abs(quadrature) / largestPoleCurrent,
                             std::abs(before), std::abs(before + quadrature));
      }

    return changed;
  }
}

template class DPsim::MNAQuadratureCorrector<Real>;
template class DPsim::MNAQuadratureCorrector<Complex>;
