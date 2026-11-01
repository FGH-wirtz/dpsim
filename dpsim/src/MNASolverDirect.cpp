/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include <type_traits>

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim/MNASolverDirect.h>
#include <dpsim/SequentialScheduler.h>

using namespace DPsim;
using namespace CPS;

namespace DPsim {

template <typename VarType>
MnaSolverDirect<VarType>::MnaSolverDirect(String name, CPS::Domain domain,
                                          CPS::Logger::Level logLevel)
    : MnaSolver<VarType>(name, domain, logLevel) {
#ifdef WITH_KLU
  mImplementationInUse = DirectLinearSolverImpl::KLU;
#elif defined(WITH_SPARSE)
  mImplementationInUse = DirectLinearSolverImpl::SparseLU;
#else
  mImplementationInUse = DirectLinearSolverImpl::DenseLU;
#endif
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixEmpty(std::size_t index) {
  mSwitchedMatrices[std::bitset<SWITCH_NUM>(index)][0].setZero();
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixEmpty(std::size_t swIdx,
                                                   Int freqIdx) {
  mSwitchedMatrices[std::bitset<SWITCH_NUM>(swIdx)][freqIdx].setZero();
}

template <typename VarType>
void MnaSolverDirect<VarType>::switchedMatrixStamp(
    std::size_t index, std::vector<std::shared_ptr<CPS::MNAInterface>> &comp) {
  auto bit = std::bitset<SWITCH_NUM>(index);
  auto &sys = mSwitchedMatrices[bit][0];
  for (auto component : comp) {
    component->mnaApplySystemMatrixStamp(sys);
  }
  for (UInt i = 0; i < mSwitches.size(); ++i)
    mSwitches[i]->mnaApplySwitchSystemMatrixStamp(bit[i], sys, 0);

  // Compute LU-factorization for system matrix
  mDirectLinearSolvers[bit][0]->preprocessing(sys,
                                              mListVariableSystemMatrixEntries);
  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolvers[bit][0]->factorize(sys);
  mCompensationResponse.clear();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mFactorizeTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::stampVariableSystemMatrix() {

  this->mDirectLinearSolverVariableSystemMatrix =
      createDirectSolverImplementation(mSLog);
  // TODO: a direct linear solver configuration is only applied if system matrix recomputation is used
  this->mDirectLinearSolverVariableSystemMatrix->setConfiguration(
      mConfigurationInUse);

  SPDLOG_LOGGER_INFO(mSLog,
                     "Number of variable Elements: {}"
                     "\nNumber of MNA components: {}",
                     mVariableComps.size(), mMNAComponents.size());

  // Build base matrix with only static elements
  mBaseSystemMatrix.setZero();
  for (auto statElem : mMNAComponents)
    statElem->mnaApplySystemMatrixStamp(mBaseSystemMatrix);
  SPDLOG_LOGGER_INFO(mSLog, "Base matrix with only static elements: {}",
                     Logger::matrixToString(mBaseSystemMatrix));
  mSLog->flush();

  // Continue from base matrix
  mVariableSystemMatrix = mBaseSystemMatrix;

  // Now stamp initial state of variable elements and switches into matrix
  SPDLOG_LOGGER_INFO(mSLog, "Stamping variable elements");
  for (auto varElem : mMNAIntfVariableComps)
    varElem->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}",
                     Logger::matrixToString(mVariableSystemMatrix));
  /* TODO: find replacement for flush() */
  mSLog->flush();

  // Calculate factorization of current matrix
  mDirectLinearSolverVariableSystemMatrix->preprocessing(
      mVariableSystemMatrix, mListVariableSystemMatrixEntries);

  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->factorize(mVariableSystemMatrix);
  mCompensationResponse.clear();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mFactorizeTimes.push_back(diff.count());
}

template <typename VarType>
void MnaSolverDirect<VarType>::solveWithSystemMatrixRecomputation(
    Real time, Int timeStepCount) {
  if (mQuadratureCorrector && !mIsInInitialization)
    mQuadratureCorrector->storeStep(**mLeftSideVector, time - mTimeStep);

  // Reset source vector
  mRightSideVector.setZero();

  // Add together the right side vector (computed by the components'
  // pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVector += *stamp;

  if (!mIsInInitialization)
    applyStateHandover();

  // Get switch and variable comp status and update system matrix and lu factorization accordingly
  mVariableComponentChanged = hasVariableComponentChanged();
  if (mVariableComponentChanged)
    recomputeSystemMatrix(time);

  // Calculate new solution vector
  auto start = std::chrono::steady_clock::now();
  **mLeftSideVector =
      mDirectLinearSolverVariableSystemMatrix->solve(mRightSideVector);
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mSolveTimes.push_back(diff.count());

  // Asked on every solve, since asking clears the switches' records. The
  // correction needs the post-event matrix, hence mVariableComponentChanged.
  const Bool correctionNeeded = anySwitchNeedsCorrection();

  if (!mIsInInitialization && mVariableComponentChanged && correctionNeeded)
    applyQuadratureCorrection(*mDirectLinearSolverVariableSystemMatrix,
                              mVariableSystemMatrix, time);

  applyCompensation(*mDirectLinearSolverVariableSystemMatrix, time);

  // For the next prepareStep(), the first point a rule change still reaches
  if (!mIsInInitialization && mVariableComponentChanged)
    mVariableComponentChangedLastStep = true;

  // TODO split into separate task? (dependent on x, updating all v attributes)
  for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
    mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

  // Components' states will be updated by the post-step tasks
}

template <typename VarType>
UInt MnaSolverDirect<VarType>::setIntegrationRule(
    CPS::MNAInterface::IntegrationRule rule) {
  UInt declined = 0;
  for (auto comp : mMNAComponents)
    if (!comp->mnaSetIntegrationRule(rule, mTimeStep))
      ++declined;
  for (auto comp : mMNAIntfVariableComps)
    if (!comp->mnaSetIntegrationRule(rule, mTimeStep))
      ++declined;

  mBaseSystemMatrix.setZero();
  for (auto statElem : mMNAComponents)
    statElem->mnaApplySystemMatrixStamp(mBaseSystemMatrix);

  mVariableSystemMatrix = mBaseSystemMatrix;
  for (auto varElem : mMNAIntfVariableComps)
    varElem->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  mDirectLinearSolverVariableSystemMatrix->factorize(mVariableSystemMatrix);
  mCompensationResponse.clear();
  return declined;
}

template <typename VarType> void MnaSolverDirect<VarType>::prepareStep() {
  if (!mDampedStepEnabled || !mSystemMatrixRecomputationEnabled)
    return;

  if (mDampedStepActive) {
    setIntegrationRule(CPS::MNAInterface::IntegrationRule::Trapezoidal);
    mDampedStepActive = false;
  } else if (mVariableComponentChangedLastStep) {
    UInt declined =
        setIntegrationRule(CPS::MNAInterface::IntegrationRule::BackwardEuler);
    mDampedStepActive = true;
    SPDLOG_LOGGER_INFO(mSLog,
                       "Damped step: backward Euler for one step, {} "
                       "components could not follow and stay trapezoidal.",
                       declined);
  }

  mVariableComponentChangedLastStep = false;
}

template <typename VarType>
void MnaSolverDirect<VarType>::recomputeSystemMatrix(Real time) {
  // Start from base matrix
  mVariableSystemMatrix = mBaseSystemMatrix;

  // Now stamp variable elements and switches into matrix
  for (auto comp : mMNAIntfVariableComps)
    comp->mnaApplySystemMatrixStamp(mVariableSystemMatrix);

  // Refactorization of matrix assuming that structure remained
  // constant by omitting analyzePattern
  auto start = std::chrono::steady_clock::now();
  mDirectLinearSolverVariableSystemMatrix->partialRefactorize(
      mVariableSystemMatrix, mListVariableSystemMatrixEntries);
  mCompensationResponse.clear();
  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<Real> diff = end - start;
  mRecomputationTimes.push_back(diff.count());
  ++mNumRecomputations;
}

template <typename VarType>
void MnaSolverDirect<VarType>::extractStateSpace(Real time) {
  if (!mStateSpaceExtractor)
    return;

  if (mSystemMatrixRecomputationEnabled) {
    mStateSpaceExtractor->extract(*mDirectLinearSolverVariableSystemMatrix,
                                  mVariableComponentChanged,
                                  mVariableComponentChanged, time);
  } else {
    mStateSpaceExtractor->extract(
        *mDirectLinearSolvers[mCurrentSwitchStatus][0], false,
        mSystemMatrixChanged, time);
  }
}

template <> void MnaSolverDirect<Real>::createEmptySystemMatrix() {
  if (mSwitches.size() > SWITCH_NUM)
    throw SystemError("Too many Switches.");

  if (mSystemMatrixRecomputationEnabled) {
    mBaseSystemMatrix =
        SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
    mVariableSystemMatrix =
        SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices);
  } else {
    for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
      auto bit = std::bitset<SWITCH_NUM>(i);
      mSwitchedMatrices[bit].push_back(
          SparseMatrix(mNumMatrixNodeIndices, mNumMatrixNodeIndices));
      mDirectLinearSolvers[bit].push_back(
          createDirectSolverImplementation(mSLog));
    }
  }
}

template <> void MnaSolverDirect<Complex>::createEmptySystemMatrix() {
  if (mSwitches.size() > SWITCH_NUM)
    throw SystemError("Too many Switches.");

  if (mFrequencyParallel) {
    for (UInt i = 0; i < std::pow(2, mSwitches.size()); ++i) {
      for (Int freq = 0; freq < mSystem.mFrequencies.size(); ++freq) {
        auto bit = std::bitset<SWITCH_NUM>(i);
        mSwitchedMatrices[bit].push_back(SparseMatrix(
            2 * (mNumMatrixNodeIndices), 2 * (mNumMatrixNodeIndices)));
        mDirectLinearSolvers[bit].push_back(
            createDirectSolverImplementation(mSLog));
      }
    }
  } else if (mSystemMatrixRecomputationEnabled) {
    mBaseSystemMatrix =
        SparseMatrix(2 * (mNumMatrixNodeIndices), 2 * (mNumMatrixNodeIndices));
    mVariableSystemMatrix =
        SparseMatrix(2 * (mNumMatrixNodeIndices), 2 * (mNumMatrixNodeIndices));
  } else {
    for (std::size_t i = 0; i < (1ULL << mSwitches.size()); i++) {
      auto bit = std::bitset<SWITCH_NUM>(i);
      mSwitchedMatrices[bit].push_back(SparseMatrix(
          2 * (mNumTotalMatrixNodeIndices), 2 * (mNumTotalMatrixNodeIndices)));
      mDirectLinearSolvers[bit].push_back(
          createDirectSolverImplementation(mSLog));
    }
  }
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTask() {
  return std::make_shared<MnaSolverDirect<VarType>::SolveTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createSolveTaskRecomp() {
  return std::make_shared<MnaSolverDirect<VarType>::SolveTaskRecomp>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task>
MnaSolverDirect<VarType>::createSolveTaskHarm(UInt freqIdx) {
  return std::make_shared<MnaSolverDirect<VarType>::SolveTaskHarm>(*this,
                                                                   freqIdx);
}

template <typename VarType>
std::shared_ptr<CPS::Task>
MnaSolverDirect<VarType>::createStateSpaceExtractionTask() {
  return std::make_shared<
      typename MnaSolverDirect<VarType>::StateSpaceExtractionTask>(*this);
}

template <typename VarType>
std::shared_ptr<CPS::Task> MnaSolverDirect<VarType>::createLogTask() {
  return std::make_shared<MnaSolverDirect<VarType>::LogTask>(*this);
}

template <typename VarType>
Bool MnaSolverDirect<VarType>::anySwitchNeedsCorrection() {
  Bool needed = false;
  mSwitchedPoles.clear();

  for (auto sw : mSwitches) {
    const Bool pending = sw->takeSwitchingEvent();
    const auto switched = sw->takeSwitchedPoles();

    if (pending && sw->requestsQuadratureCorrection()) {
      needed = true;
      mSwitchedPoles.insert(mSwitchedPoles.end(), switched.begin(),
                            switched.end());
    }
  }

  return needed;
}

template <typename VarType>
Bool MnaSolverDirect<VarType>::applyQuadratureCorrection(
    DirectLinearSolver &linearSolver, const SparseMatrix &systemMatrix,
    Real time) {
  if (!mQuadratureCorrector)
    return false;

  const Bool interpolated =
      mQuadratureCorrector->rewindToZero(time, mSwitchedPoles);
  const UInt corrected =
      mQuadratureCorrector->correct(systemMatrix, time, mSwitchedPoles);
  if (corrected == 0 && !interpolated)
    return false;

  assembleRightSide();

  // One step of the post-event network from the zero, then back to the step
  // the poles opened on
  if (interpolated) {
    mQuadratureCorrector->returnFromZero(linearSolver.solve(mRightSideVector));
    assembleRightSide();
  }

  **mLeftSideVector = linearSolver.solve(mRightSideVector);
  return true;
}

template <typename VarType>
void MnaSolverDirect<VarType>::assembleRightSide() {
  // composites sum the source vectors of their sub-components: re-stamp all
  for (auto comp : mMNAComponents)
    if (comp->getRightVector()->get().size() != 0)
      comp->mnaApplyRightSideVectorStamp(**comp->getRightVector());
  for (auto comp : mMNAIntfVariableComps)
    if (comp->getRightVector()->get().size() != 0)
      comp->mnaApplyRightSideVectorStamp(**comp->getRightVector());
  for (auto comp : mMNAIntfSwitches)
    if (comp->getRightVector()->get().size() != 0)
      comp->mnaApplyRightSideVectorStamp(**comp->getRightVector());

  mRightSideVector.setZero();
  for (auto stamp : mRightVectorStamps)
    mRightSideVector += *stamp;
}

template <typename VarType>
Bool MnaSolverDirect<VarType>::applyStateHandover() {
  std::vector<CPS::MNASwitchInterface::StateHandover> handover;
  for (auto sw : mSwitches) {
    auto taken = sw->takeStateHandover();
    handover.insert(handover.end(), taken.begin(), taken.end());
  }
  if (handover.empty())
    return false;

  if constexpr (!std::is_same<VarType, Complex>::value) {
    throw std::runtime_error(
        "MnaSolverDirect: a state handover needs the DP domain.");
  } else {
    // storage elements by name, sub-components included; duplicate names are
    // ignored
    std::unordered_map<String, CPS::MNAInterface::Ptr> states;
    std::function<void(const CPS::MNAInterface::Ptr &)> collect =
        [&](const CPS::MNAInterface::Ptr &comp) {
          auto state = std::dynamic_pointer_cast<
              CPS::MNAQuadratureCorrectionInterface<VarType>>(comp);
          auto object = std::dynamic_pointer_cast<CPS::IdentifiedObject>(comp);
          if (state && object) {
            auto inserted = states.emplace(object->name(), comp);
            if (!inserted.second)
              inserted.first->second = nullptr;
          }
          if (auto composite =
                  std::dynamic_pointer_cast<CPS::CompositePowerComp<VarType>>(
                      comp))
            for (auto sub : composite->mnaSubComponents())
              collect(sub);
        };
    for (auto comp : mMNAComponents)
      collect(comp);
    for (auto comp : mMNAIntfVariableComps)
      collect(comp);

    for (auto &h : handover) {
      const auto &e = h.element;
      auto it = states.find(e.name);
      if (it == states.end() || !it->second)
        throw std::runtime_error("MnaSolverDirect: no unique storage element " +
                                 e.name + " for a state handover.");
      auto state = std::dynamic_pointer_cast<
          CPS::MNAQuadratureCorrectionInterface<VarType>>(it->second);
      auto comp = std::dynamic_pointer_cast<CPS::SimPowerComp<VarType>>(
          it->second);
      if (!comp || comp->terminalNumber() != 2 ||
          comp->node(0)->name() != e.node0 ||
          comp->node(1)->name() != e.node1 ||
          std::abs(state->mnaCorrectionElementValue(0) - e.value) >
              1e-9 * std::abs(e.value))
        throw std::runtime_error("MnaSolverDirect: storage element " + e.name +
                                 " differs from the one handed over.");
      state->mnaCorrectionAddState(0, h.voltage, h.current);
      state->mnaCorrectionRebuildRightVector();
    }
  }
  assembleRightSide();
  return true;
}

template <typename VarType>
void MnaSolverDirect<VarType>::applyCompensation(
    DirectLinearSolver &linearSolver, Real time) {
  if (!mCompensatedCollected) {
    auto collect = [&](const CPS::MNAInterface::List &list) {
      for (auto comp : list)
        if (auto c = std::dynamic_pointer_cast<CPS::MNACompensatedInterface>(comp))
          if (c->mnaIsCompensated() &&
              std::find(mCompensated.begin(), mCompensated.end(), c) ==
                  mCompensated.end())
            mCompensated.push_back(c);
    };
    collect(mMNAComponents);
    collect(mMNAIntfVariableComps);
    collect(mMNAIntfSwitches);
    mCompensatedCollected = true;
  }
  if (mCompensated.empty())
    return;

  if constexpr (!std::is_same<VarType, Complex>::value) {
    throw std::runtime_error(
        "MnaSolverDirect: compensated components need the DP domain.");
  } else {
    // ports of all components, with their zero-sequence network and index
    struct Port {
      std::size_t comp;
      std::pair<Int, Int> nodes;
      CPS::MNAZeroSequenceNetwork::Ptr zero;
      UInt zeroIndex;
    };
    std::vector<Port> ports;
    std::vector<CPS::MNAZeroSequenceNetwork::Ptr> networks;
    std::vector<Matrix::Index> firstPort;
    for (std::size_t c = 0; c < mCompensated.size(); ++c) {
      firstPort.push_back(static_cast<Matrix::Index>(ports.size()));
      const auto terminals = mCompensated[c]->mnaCompensationTerminals();
      const auto zero = mCompensated[c]->mnaZeroSequenceNetwork();
      const auto indices = mCompensated[c]->mnaZeroSequencePorts();
      if (!zero || indices.size() != terminals.size())
        throw std::runtime_error("MnaSolverDirect: compensated ports need a "
                                 "zero-sequence network.");
      for (std::size_t j = 0; j < terminals.size(); ++j)
        ports.push_back({c, terminals[j], zero, indices[j]});
      if (std::find(networks.begin(), networks.end(), zero) == networks.end())
        networks.push_back(zero);
    }
    const Matrix::Index k = static_cast<Matrix::Index>(ports.size());

    Matrix &x = **mLeftSideVector;
    const Matrix::Index n = x.rows();
    // v1 - v0 (re, im) per port from MNA vectors
    auto voltages = [&](const Matrix &v) {
      Matrix out = Matrix::Zero(2 * k, v.cols());
      for (Matrix::Index j = 0; j < k; ++j)
        for (Matrix::Index c = 0; c < v.cols(); ++c) {
          const Matrix col = v.col(c);
          Complex u(0., 0.);
          if (ports[j].nodes.second >= 0)
            u += Math::complexFromVectorElement(col, ports[j].nodes.second);
          if (ports[j].nodes.first >= 0)
            u -= Math::complexFromVectorElement(col, ports[j].nodes.first);
          out(2 * j, c) = u.real();
          out(2 * j + 1, c) = u.imag();
        }
      return out;
    };
    // network response to unit (re, im) injections, once per factorization
    auto it = mCompensationResponse.find(&linearSolver);
    if (it == mCompensationResponse.end()) {
      Matrix r(n, 2 * k);
      for (Matrix::Index j = 0; j < k; ++j)
        for (int part = 0; part < 2; ++part) {
          Matrix b = Matrix::Zero(n, 1);
          const Complex unit = part == 0 ? Complex(1., 0.) : Complex(0., 1.);
          if (ports[j].nodes.second >= 0)
            Math::addToVectorElement(b, ports[j].nodes.second, unit);
          if (ports[j].nodes.first >= 0)
            Math::addToVectorElement(b, ports[j].nodes.first, -unit);
          r.col(2 * j + part) = linearSolver.solve(b);
        }
      it = mCompensationResponse.emplace(&linearSolver, r).first;
    }
    const Matrix &r = it->second;
    const Matrix open = voltages(x);
    const Matrix response = voltages(r);

    // zero-sequence networks: free voltages and responses per port
    std::vector<Matrix> zeroFree, zeroResponse;
    for (auto &net : networks) {
      zeroFree.push_back(net->zeroFreeVoltage());
      zeroResponse.push_back(net->zeroVoltageResponse());
    }
    auto networkOf = [&](Matrix::Index j) {
      return static_cast<std::size_t>(
          std::find(networks.begin(), networks.end(), ports[j].zero) -
          networks.begin());
    };

    // unknowns (Re I_j, Im I_j) for all ports, then i0_j; the zero-sequence
    // input of a port is the envelope i0 e^(-j w t)
    const Real wt = this->mSystem.mSystemOmega * time;
    const Complex carrier = std::polar<Real>(1.0, wt);
    const Complex zeroEnvelope = std::polar<Real>(1.0, -wt);
    Matrix z;
    for (int pass = 0; pass < 4; ++pass) {
      Matrix m = Matrix::Zero(3 * k, 3 * k), rhs = Matrix::Zero(3 * k, 1);
      for (std::size_t c = 0; c < mCompensated.size(); ++c) {
        Matrix a, b, g;
        mCompensated[c]->mnaCompensationConditions(time, a, b, g);
        const Matrix::Index j0 = firstPort[c];
        for (Matrix::Index row = 0; row < a.rows(); ++row) {
          const Matrix::Index j = j0 + row / 3;
          const int p = static_cast<int>(row % 3);
          const Matrix::Index eq = 3 * j + p;
          const Complex rot = std::polar<Real>(1.0, -2. * PI * p / 3.) * carrier;
          // instantaneous voltage: Re{rot (open + R I)} + Re{carrier V0}
          for (Matrix::Index l = 0; l < k; ++l) {
            m(eq, 2 * l) += a(row, 0) * (rot.real() * response(2 * j, 2 * l) -
                                         rot.imag() * response(2 * j + 1, 2 * l));
            m(eq, 2 * l + 1) +=
                a(row, 0) * (rot.real() * response(2 * j, 2 * l + 1) -
                             rot.imag() * response(2 * j + 1, 2 * l + 1));
          }
          const std::size_t net = networkOf(j);
          const Matrix::Index zj = ports[j].zeroIndex;
          for (Matrix::Index l = 0; l < k; ++l) {
            if (networkOf(l) != net)
              continue;
            const Matrix::Index zl = ports[l].zeroIndex;
            const Matrix &g0 = zeroResponse[net];
            const Complex perAmp(
                g0(2 * zj, 2 * zl) * zeroEnvelope.real() +
                    g0(2 * zj, 2 * zl + 1) * zeroEnvelope.imag(),
                g0(2 * zj + 1, 2 * zl) * zeroEnvelope.real() +
                    g0(2 * zj + 1, 2 * zl + 1) * zeroEnvelope.imag());
            m(eq, 2 * k + l) += a(row, 0) * std::real(carrier * perAmp);
          }
          const Complex openV(open(2 * j, 0), open(2 * j + 1, 0));
          const Complex zeroV(zeroFree[net](2 * zj, 0),
                              zeroFree[net](2 * zj + 1, 0));
          rhs(eq, 0) = g(row, 0) - a(row, 0) * (std::real(rot * openV) +
                                                std::real(carrier * zeroV));
          // instantaneous current: Re{rot I} + i0
          m(eq, 2 * j) += b(row, 0) * rot.real();
          m(eq, 2 * j + 1) -= b(row, 0) * rot.imag();
          m(eq, 2 * k + j) += b(row, 0);
        }
      }
      z = m.partialPivLu().solve(rhs);

      // results; a component whose conditions changed asks for another pass
      Bool again = false;
      for (std::size_t c = 0; c < mCompensated.size(); ++c) {
        const Matrix::Index j0 = firstPort[c];
        const Matrix::Index kc =
            (c + 1 < mCompensated.size() ? firstPort[c + 1] : k) - j0;
        Matrix injection(2 * kc, 1), zero(kc, 1), phase(3 * kc, 1);
        for (Matrix::Index j = 0; j < kc; ++j) {
          const Complex current(z(2 * (j0 + j), 0), z(2 * (j0 + j) + 1, 0));
          injection(2 * j, 0) = current.real();
          injection(2 * j + 1, 0) = current.imag();
          zero(j, 0) = z(2 * k + j0 + j, 0);
          for (int p = 0; p < 3; ++p)
            phase(3 * j + p, 0) =
                std::real(std::polar<Real>(1.0, -2. * PI * p / 3.) * carrier *
                          current) +
                zero(j, 0);
        }
        again = mCompensated[c]->mnaCompensationResult(time, injection, zero,
                                                       phase) ||
                again;
      }
      if (!again)
        break;
      if (pass == 3)
        throw std::runtime_error("MnaSolverDirect: compensated poles keep "
                                 "changing within a step.");
    }

    // advance the zero-sequence networks, add the injections
    for (std::size_t net = 0; net < networks.size(); ++net) {
      Matrix input = Matrix::Zero(zeroResponse[net].cols(), 1);
      for (Matrix::Index j = 0; j < k; ++j)
        if (networkOf(j) == net) {
          const Matrix::Index zj = ports[j].zeroIndex;
          input(2 * zj, 0) += z(2 * k + j, 0) * zeroEnvelope.real();
          input(2 * zj + 1, 0) += z(2 * k + j, 0) * zeroEnvelope.imag();
        }
      networks[net]->zeroAdvance(input);
    }
    x += r * z.topRows(2 * k);
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::solve(Real time, Int timeStepCount) {
  const auto previousSwitchStatus = mCurrentSwitchStatus;
  if (mQuadratureCorrector && !mIsInInitialization)
    mQuadratureCorrector->storeStep(**mLeftSideVector, time - mTimeStep);
  // Reset source vector
  mRightSideVector.setZero();

  // Add together the right side vector (computed by the components' pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVector += *stamp;

  if (!mIsInInitialization) {
    applyStateHandover();
    MnaSolver<VarType>::updateSwitchStatus();
  }

  if (mSwitchedMatrices.size() > 0) {
    std::chrono::steady_clock::time_point start;
    if (Solver::mLogSolveTimes)
      start = std::chrono::steady_clock::now();

    **mLeftSideVector =
        mDirectLinearSolvers[mCurrentSwitchStatus][0]->solve(mRightSideVector);

    const Bool correctionNeeded = anySwitchNeedsCorrection();

    if (!mIsInInitialization && correctionNeeded)
      applyQuadratureCorrection(*mDirectLinearSolvers[mCurrentSwitchStatus][0],
                                mSwitchedMatrices[mCurrentSwitchStatus][0],
                                time);

    applyCompensation(*mDirectLinearSolvers[mCurrentSwitchStatus][0], time);

    if (Solver::mLogSolveTimes) {
      auto end = std::chrono::steady_clock::now();
      std::chrono::duration<Real> diff = end - start;
      mSolveTimes.push_back(diff.count());
    }
  }

  // CHECK: Is this really required? Or can operations actually become part of
  // correctorStep and mnaPostStep?
  for (auto syncGen : mSyncGen)
    syncGen->updateVoltage(**mLeftSideVector);

  // Reset number of iterations
  mIter = 0;

  // Additional solve steps for iterative models
  if (mSyncGen.size() > 0) {
    if (!mCompensated.empty())
      throw std::runtime_error("MnaSolverDirect: compensated components with "
                               "iterating synchronous generators.");
    UInt numCompsRequireIter;
    do {
      // count synchronous generators that require iteration
      numCompsRequireIter = 0;
      for (auto syncGen : mSyncGen)
        if (syncGen->requiresIteration())
          numCompsRequireIter++;

      // recompute solve step if at least one component demands iteration
      if (numCompsRequireIter > 0) {
        mIter++;

        // Reset source vector
        mRightSideVector.setZero();

        if (!mIsInInitialization)
          MnaSolver<VarType>::updateSwitchStatus();

        for (auto syncGen : mSyncGen)
          syncGen->correctorStep();

        // Add together the right side vector (computed by the components' pre-step tasks)
        for (auto stamp : mRightVectorStamps)
          mRightSideVector += *stamp;

        if (mSwitchedMatrices.size() > 0) {
          auto start = std::chrono::steady_clock::now();
          **mLeftSideVector =
              mDirectLinearSolvers[mCurrentSwitchStatus][0]->solve(
                  mRightSideVector);
          auto end = std::chrono::steady_clock::now();
          std::chrono::duration<Real> diff = end - start;
          mSolveTimes.push_back(diff.count());
        }

        // CHECK: Is this really required? Or can operations actually become part of
        // correctorStep and mnaPostStep?
        for (auto syncGen : mSyncGen)
          syncGen->updateVoltage(**mLeftSideVector);
      }
    } while (numCompsRequireIter > 0);
  }

  mSystemMatrixChanged = (mCurrentSwitchStatus != previousSwitchStatus);

  // TODO split into separate task? (dependent on x, updating all v attributes)
  for (UInt nodeIdx = 0; nodeIdx < mNumNetNodes; ++nodeIdx)
    mNodes[nodeIdx]->mnaUpdateVoltage(**mLeftSideVector);

  // Components' states will be updated by the post-step tasks
}

template <typename VarType>
void MnaSolverDirect<VarType>::solveWithHarmonics(Real time, Int timeStepCount,
                                                  Int freqIdx) {
  if (!mCompensated.empty())
    throw std::runtime_error(
        "MnaSolverDirect: compensated components with frequency parallel "
        "solves.");
  mRightSideVectorHarm[freqIdx].setZero();

  // Sum of right side vectors (computed by the components' pre-step tasks)
  for (auto stamp : mRightVectorStamps)
    mRightSideVectorHarm[freqIdx] += stamp->col(freqIdx);

  **mLeftSideVectorHarm[freqIdx] =
      mDirectLinearSolvers[mCurrentSwitchStatus][freqIdx]->solve(
          mRightSideVectorHarm[freqIdx]);
}

template <typename VarType> void MnaSolverDirect<VarType>::logSystemMatrices() {
  if (mFrequencyParallel) {
    for (UInt i = 0; i < mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)].size();
         ++i) {
      SPDLOG_LOGGER_INFO(mSLog, "System matrix for frequency: {:d} \n{:s}", i,
                         Logger::matrixToString(
                             mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][i]));
    }

    for (UInt i = 0; i < mRightSideVectorHarm.size(); ++i) {
      SPDLOG_LOGGER_INFO(mSLog, "Right side vector for frequency: {:d} \n{:s}",
                         i, Logger::matrixToString(mRightSideVectorHarm[i]));
    }

  } else if (mSystemMatrixRecomputationEnabled) {
    SPDLOG_LOGGER_INFO(mSLog, "Summarizing matrices: ");
    SPDLOG_LOGGER_INFO(mSLog, "Base matrix with only static elements: {}",
                       Logger::matrixToString(mBaseSystemMatrix));
    SPDLOG_LOGGER_INFO(mSLog, "Initial system matrix with variable elements {}",
                       Logger::matrixToString(mVariableSystemMatrix));
    SPDLOG_LOGGER_INFO(mSLog, "Right side vector: {}",
                       Logger::matrixToString(mRightSideVector));
  } else {
    if (mSwitches.size() < 1) {
      SPDLOG_LOGGER_INFO(mSLog, "System matrix: \n{}",
                         mSwitchedMatrices[std::bitset<SWITCH_NUM>(0)][0]);
    } else {
      SPDLOG_LOGGER_INFO(mSLog, "Initial switch status: {:s}",
                         mCurrentSwitchStatus.to_string());

      for (auto sys : mSwitchedMatrices) {
        SPDLOG_LOGGER_INFO(mSLog, "Switching System matrix {:s} \n{:s}",
                           sys.first.to_string(),
                           Logger::matrixToString(sys.second[0]));
      }
    }
    SPDLOG_LOGGER_INFO(mSLog, "Right side vector: \n{}", mRightSideVector);
  }
}

template <typename VarType> void MnaSolverDirect<VarType>::logLUTimes() {
  logFactorizationTime();
  logRecomputationTime();
  logSolveTime();
  logCorrectionStatistics();
}

template <typename VarType>
void MnaSolverDirect<VarType>::logCorrectionStatistics() {
  if (!mQuadratureCorrector)
    return;

  SPDLOG_LOGGER_INFO(
      mSLog,
      "Quadrature correction statistics: enabled {:d}, correctable states "
      "{:d}, calls {:d}, quadrature solves {:d}, cumulative time {:.12f}, "
      "quadrature corrections {:d}",
      static_cast<Int>(mQuadratureCorrector->isEnabled()),
      mQuadratureCorrector->getStateCount(),
      mQuadratureCorrector->getCallCount(),
      mQuadratureCorrector->getQuadratureSolveCount(),
      mQuadratureCorrector->getCumulativeTime(),
      mQuadratureCorrector->getCorrectedStateCount());
}

template <typename VarType> void MnaSolverDirect<VarType>::logSolveTime() {
  Real solveSum = 0.0;
  Real solveMax = 0.0;
  for (auto meas : mSolveTimes) {
    solveSum += meas;
    if (meas > solveMax)
      solveMax = meas;
  }
  SPDLOG_LOGGER_INFO(mSLog, "Cumulative solve times: {:.12f}", solveSum);
  SPDLOG_LOGGER_INFO(mSLog, "Average solve time: {:.12f}",
                     solveSum / static_cast<double>(mSolveTimes.size()));
  SPDLOG_LOGGER_INFO(mSLog, "Maximum solve time: {:.12f}", solveMax);
  SPDLOG_LOGGER_INFO(mSLog, "Number of solves: {:d}", mSolveTimes.size());
}

template <typename VarType>
void MnaSolverDirect<VarType>::logFactorizationTime() {
  for (auto meas : mFactorizeTimes) {
    SPDLOG_LOGGER_INFO(mSLog, "LU factorization time: {:.12f}", meas);
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::logRecomputationTime() {
  Real recompSum = 0.0;
  Real recompMax = 0.0;
  for (auto meas : mRecomputationTimes) {
    recompSum += meas;
    if (meas > recompMax)
      recompMax = meas;
  }
  // Sometimes, refactorization is not used
  if (mRecomputationTimes.size() != 0) {
    SPDLOG_LOGGER_INFO(mSLog, "Cumulative refactorization times: {:.12f}",
                       recompSum);
    SPDLOG_LOGGER_INFO(mSLog, "Average refactorization time: {:.12f}",
                       recompSum / ((double)mRecomputationTimes.size()));
    SPDLOG_LOGGER_INFO(mSLog, "Maximum refactorization time: {:.12f}",
                       recompMax);
    SPDLOG_LOGGER_INFO(mSLog, "Number of refactorizations: {:d}",
                       mRecomputationTimes.size());
  }
}

template <typename VarType>
std::shared_ptr<DirectLinearSolver>
MnaSolverDirect<VarType>::createDirectSolverImplementation(
    CPS::Logger::Log mSLog) {
  switch (this->mImplementationInUse) {
  case DirectLinearSolverImpl::DenseLU:
    return std::make_shared<DenseLUAdapter>(mSLog);
#ifdef WITH_SPARSE
  case DirectLinearSolverImpl::SparseLU:
    return std::make_shared<SparseLUAdapter>(mSLog);
#endif // WITH_SPARSE
#ifdef WITH_KLU
  case DirectLinearSolverImpl::KLU:
    return std::make_shared<KLUAdapter>(mSLog);
#endif
#ifdef WITH_CUDA
  case DirectLinearSolverImpl::CUDADense:
    return std::make_shared<GpuDenseAdapter>(mSLog);
#ifdef WITH_CUDA_SPARSE
  case DirectLinearSolverImpl::CUDASparse:
    return std::make_shared<GpuSparseAdapter>(mSLog);
#endif
#ifdef WITH_MAGMA
  case DirectLinearSolverImpl::CUDAMagma:
    return std::make_shared<GpuMagmaAdapter>(mSLog);
#endif
#endif
  default:
    throw CPS::SystemError("unsupported linear solver implementation.");
  }
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverImplementation(
    DirectLinearSolverImpl implementation) {
  this->mImplementationInUse = implementation;
}

template <typename VarType>
void MnaSolverDirect<VarType>::setDirectLinearSolverConfiguration(
    DirectLinearSolverConfiguration &configuration) {
  this->mConfigurationInUse = configuration;
}

} // namespace DPsim

template class DPsim::MnaSolverDirect<Real>;
template class DPsim::MnaSolverDirect<Complex>;
