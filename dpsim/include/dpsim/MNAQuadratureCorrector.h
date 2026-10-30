// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>
#include <string>
#include <vector>

#include <dpsim-models/Logger.h>
#include <dpsim-models/SimPowerComp.h>
#include <dpsim-models/Solver/MNAEventStampInterface.h>
#include <dpsim-models/Solver/MNAQuadratureCorrectionInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim/Definitions.h>

namespace DPsim {

/// \brief Removes the quadrature part a switching event leaves in the storage
/// states of an envelope simulation.
///
/// An envelope at the event instant splits into an in-phase part x and a
/// quadrature part y, and only x is physical. At a current zero x_sw = 0 but
/// y_sw = |I_sw|, which the inductor history terms keep driving after the
/// opening. One solve of the post-event network at the shift frequency gives
/// the change of every state,
///
///     Y50 dV = -u dG (v1 - v0),   X_k += j Im{dX_k c} conj(c),
///
/// with c the carrier of the step the poles switched in, the step the corrected
/// states belong to, for openings and closings. Inductors take the opening poles,
/// capacitors the closing ones. The optional zero-crossing interpolation
/// rewinds the states of an opening to the interpolated current zero t0,
/// corrects them there with the carrier of t0, steps the post-event network
/// from t0 and interpolates back to the step. See switches-and-loads.md for the
/// derivation.
template <typename VarType> class MNAQuadratureCorrector {
public:
  using Ptr = std::shared_ptr<MNAQuadratureCorrector<VarType>>;

  MNAQuadratureCorrector() = default;

  void
  initialize(const typename CPS::MNAQuadratureCorrectionInterface<VarType>::List
                 &states,
             UInt mnaVectorSize, UInt numFreqs, Real omega, Real timeStep,
             CPS::Logger::Log log);

  Bool isInitialized() const { return mInitialized; }

  /// Components without correctable states that the correction's network
  /// solve has to see, such as a machine that injects its current
  void setEventStamps(
      const typename CPS::MNAEventStampInterface<VarType>::List &stamps) {
    mEventStamps = stamps;
  }

  /// Off makes correct() a no-op, for a with-and-without comparison
  void setEnabled(Bool enabled) { mEnabled = enabled; }
  Bool isEnabled() const { return mEnabled; }

  /// On rewinds an opening to its interpolated current zero, see the class
  /// description. The solver stores the states before every step.
  void setZeroCrossingInterpolation(Bool enabled) { mInterpolation = enabled; }
  Bool isZeroCrossingInterpolation() const { return mInterpolation; }

  /// Keeps the storage states and node voltages of the last two steps, called
  /// before every step with the solution of the previous one.
  void storeStep(const Matrix &leftVector, Real time);

  /// Moves the storage states to the earliest current zero of the opening
  /// poles and replaces each pole's current by its value there. False if the
  /// event cannot be interpolated, then nothing is changed.
  Bool rewindToZero(Real time,
                    std::vector<CPS::MNASwitchInterface::SwitchedPole> &poles);

  /// After a step from the zero with the post-event network, whose solution is
  /// openStep, interpolates the storage states back to the step the poles
  /// opened on.
  void returnFromZero(const Matrix &openStep);

  UInt getStateCount() const { return static_cast<UInt>(mEntries.size()); }

  /// Corrects the quadrature part of the storage states for the poles that
  /// opened at a current zero or closed, and returns how many states were
  /// changed. A non-zero result means the caller must re-assemble the
  /// right-hand side and solve again. systemMatrix is the post-event matrix
  /// the step is solved with.
  UInt correct(
      const SparseMatrix &systemMatrix, Real time,
      const std::vector<CPS::MNASwitchInterface::SwitchedPole> &switchedPoles);

  /// #### Cost accounting
  /// Times correct() ran, which is once per reported switching event
  UInt getCallCount() const { return mCallCount; }
  /// Factorisations of the post-event network, one per event
  UInt getQuadratureSolveCount() const { return mQuadratureSolves; }
  /// States whose quadrature part the correction changed
  UInt getCorrectedStateCount() const { return mCorrectedStates; }
  /// Seconds spent inside correct()
  Real getCumulativeTime() const { return mCumulativeTime; }

private:
  struct Snapshot {
    Matrix leftVector;
    std::vector<CPS::MatrixVar<VarType>> voltages;
    std::vector<CPS::MatrixVar<VarType>> currents;
    Real time = 0.0;
    Bool valid = false;
  };

  struct Entry {
    typename CPS::MNAQuadratureCorrectionInterface<VarType>::Ptr state;
    UInt index = 0;
    CPS::String name;
    Int node0 = -1;
    Int node1 = -1;
  };

  /// One steady-state solve of the post-event network for the switched poles,
  /// whose quadrature part is added for an opening to every correctable
  /// inductor state, and for a closing to every capacitor
  UInt correctQuadrature(
      const SparseMatrix &systemMatrix, Real time,
      const std::vector<CPS::MNASwitchInterface::SwitchedPole> &switchedPoles);

  void takeSnapshot(Snapshot &snapshot, const Matrix &leftVector, Real time);
  void setStates(const Snapshot &from, const Snapshot &to, Real fraction);

  std::vector<Entry> mEntries;
  /// One per component, with its interface voltage and current, which hold
  /// the storage state and the history term
  std::vector<typename CPS::SimPowerComp<VarType>::Ptr> mComponents;
  Snapshot mPreviousStep;
  Snapshot mLastStep;
  Snapshot mAtZero;
  /// Carrier at the current zero of each switched pole, empty outside an
  /// interpolated event
  std::vector<Complex> mPoleCarriers;
  typename CPS::MNAEventStampInterface<VarType>::List mEventStamps;
  /// Entry indices grouped by component, because coupling only ever runs
  /// between states of one component
  std::vector<std::vector<UInt>> mComponentEntries;
  UInt mMnaVectorSize = 0;
  UInt mNumFreqs = 1;
  Real mOmega = 0.0;
  Real mTimeStep = 0.0;
  UInt mCallCount = 0;
  UInt mQuadratureSolves = 0;
  UInt mCorrectedStates = 0;
  Real mCumulativeTime = 0.0;
  Bool mEnabled = true;
  Bool mInterpolation = false;
  Bool mInitialized = false;
  CPS::Logger::Log mSLog;
};
} // namespace DPsim
