/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <dpsim-models/DP/DP_Ph1_Switch.h>

using namespace CPS;

DP::Ph1::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Complex>(uid, name, false, true, logLevel),
      Base::Ph1::Switch(mAttributes),
      mOpeningRequested(mAttributes->create<Bool>("opening_requested")),
      mPoleClosed(mAttributes->create<Bool>("pole_closed")),
      mInstantCurrent(mAttributes->create<Real>("i_instantaneous")),
      mZeroCrossingTime(mAttributes->create<Real>("zero_crossing_time")),
      mExponentialTransitionActive(
          mAttributes->create<Bool>("exponential_transition_active")),
      mExponentialTransitionClosing(
          mAttributes->create<Bool>("exponential_transition_closing")),
      mExponentialProgress(mAttributes->create<Real>("exponential_progress")),
      mExponentialTransitionStartTime(
          mAttributes->create<Real>("exponential_transition_start_time")),
      mExponentialTransitionEndTime(
          mAttributes->create<Real>("exponential_transition_end_time")),
      mEffectiveResistance(mAttributes->create<Real>("effective_resistance")),
      mCompensationActive(
          mAttributes->create<Bool>("compensation_active", false)),
      mCompensationCurrent(mAttributes->create<MatrixComp>(
          "compensation_current", MatrixComp::Zero(3, 1))),
      mTheveninIndex(mAttributes->create<Int>("thevenin_index", 0)),
      mCompHandedOver(
          mAttributes->create<Bool>("compensation_handed_over", false)) {
  setTerminalNumber(2);
  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  **mOpeningRequested = false;
  **mPoleClosed = false;

  **mInstantCurrent = 0.0;

  resetZeroCrossingTime();

  **mExponentialTransitionActive = false;
  **mExponentialTransitionClosing = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  **mEffectiveResistance = 0.0;
}

SimPowerComp<Complex>::Ptr DP::Ph1::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  copy->setSwitchingMode(mSwitchingMode);
  copy->setZeroCrossingTolerance(mZeroCrossingTolerance);
  copy->setExponentialSwitchingTime(mExponentialSwitchingTime);
  return copy;
}

void DP::Ph1::Switch::setSwitchingMode(SwitchingMode mode) {
  mSwitchingMode = mode;

  **mPoleClosed = **mIsClosed;
  **mOpeningRequested = false;
  resetZeroCrossingTime();

  mPreviousCurrentValid = false;
  mResetZeroCrossingHistory = false;

  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);
}

void DP::Ph1::Switch::setZeroCrossingTolerance(Real tolerance) {
  if (tolerance < 0.0) {
    throw std::invalid_argument(
        "DP::Ph1::Switch zero-crossing tolerance must be non-negative.");
  }

  mZeroCrossingTolerance = tolerance;
}

void DP::Ph1::Switch::setExponentialSwitchingTime(Real switchingTime) {
  if (switchingTime <= 0.0) {
    throw std::invalid_argument(
        "DP::Ph1::Switch exponential switching time must be positive.");
  }

  mExponentialSwitchingTime = switchingTime;
}

void DP::Ph1::Switch::setTheveninEquivalents(
    const TheveninEquivalent &positive, const TheveninEquivalent &zero,
    const std::vector<ObservedNode> &nodes) {
  const Matrix::Index rows = 2 + 2 * static_cast<Matrix::Index>(nodes.size());
  for (auto *eq : {&positive, &zero}) {
    if (eq->C.rows() != rows || eq->D.rows() != rows || eq->B.cols() != 2 ||
        eq->D.cols() != 2) {
      throw std::invalid_argument(
          "DP::Ph1::Switch Thevenin equivalent does not match the observed "
          "nodes.");
    }
  }
  mThevenin1 = {positive};
  mThevenin0 = {zero};
  mCompNodes = nodes;
  mCompNodeCorrections.clear();
  for (auto &node : nodes)
    mCompNodeCorrections.push_back(mAttributes->create<MatrixComp>(
        "compensation_v_" + node.name, MatrixComp::Zero(3, 1)));
}

Int DP::Ph1::Switch::addTheveninEquivalents(const TheveninEquivalent &positive,
                                            const TheveninEquivalent &zero) {
  if (mThevenin1.empty() || positive.A.rows() != mThevenin1[0].A.rows() ||
      zero.A.rows() != mThevenin0[0].A.rows() ||
      positive.C.rows() != mThevenin1[0].C.rows() ||
      zero.C.rows() != mThevenin0[0].C.rows()) {
    throw std::invalid_argument(
        "DP::Ph1::Switch further Thevenin equivalents must match the first "
        "set.");
  }
  if (positive.stateNames != mThevenin1[0].stateNames ||
      zero.stateNames != mThevenin0[0].stateNames) {
    throw std::invalid_argument(
        "DP::Ph1::Switch further Thevenin equivalents must have the states of "
        "the first set.");
  }
  mThevenin1.push_back(positive);
  mThevenin0.push_back(zero);
  return static_cast<Int>(mThevenin1.size()) - 1;
}

void DP::Ph1::Switch::openSwitch() {
  Base::Ph1::Switch::openSwitch();

  // An opening command during a closing ramp abandons that ramp. The pole is
  // still open at that point, so the switch simply stays open.
  if (**mExponentialTransitionActive && **mExponentialTransitionClosing) {
    resetExponentialTransition();
    **mPoleClosed = false;
    synchronizeEffectiveResistance(false);
    **mOpeningRequested = false;

    SPDLOG_LOGGER_INFO(mSLog,
                       "DP opening command received during an exponential "
                       "closing transition. Transition abandoned.");
    return;
  }

  if (mSwitchingMode == SwitchingMode::Ideal) {
    **mOpeningRequested = false;
    **mPoleClosed = false;
    resetZeroCrossingTime();
    resetExponentialTransition();
    synchronizeEffectiveResistance(false);
    return;
  }

  // the poles open at their current zeros, the network pole is open anyway
  if (mSwitchingMode == SwitchingMode::SpaceVector) {
    if (mCurrentZeroOpening) {
      **mOpeningRequested = true;
      mCompArmed = false;
    } else {
      mCompOpen.fill(true);
      **mOpeningRequested = false;
    }
    SPDLOG_LOGGER_INFO(mSLog, "DP space-vector opening command received.");
    return;
  }

  // The network pole stays closed, the compensation opens the poles
  if (mSwitchingMode == SwitchingMode::Compensated) {
    if (mThevenin1.empty()) {
      throw std::invalid_argument(
          "DP::Ph1::Switch compensated mode requires Thevenin equivalents.");
    }
    **mCompensationActive = true;
    **mOpeningRequested = true;
    mCompArmed = false;
    SPDLOG_LOGGER_INFO(mSLog, "DP compensated opening command received.");
    return;
  }

  if (!**mPoleClosed) {
    **mOpeningRequested = false;
    return;
  }

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    **mOpeningRequested = true;
    resetZeroCrossingTime();
    resetExponentialTransition();
    mResetZeroCrossingHistory = true;

    SPDLOG_LOGGER_INFO(mSLog,
                       "DP current-zero opening command received. "
                       "Waiting for the reconstructed physical current zero.");
    return;
  }

  validateExponentialResistanceParameters();
  **mOpeningRequested = true;
  resetZeroCrossingTime();
  resetExponentialTransition();
  **mExponentialTransitionActive = true;
  synchronizeEffectiveResistance(true);
  **mPoleClosed = true;

  SPDLOG_LOGGER_INFO(mSLog,
                     "DP exponential ZCS-emulation opening command received. "
                     "Switching duration={:.6e}s.",
                     mExponentialSwitchingTime);
}

void DP::Ph1::Switch::closeSwitch() {
  Base::Ph1::Switch::closeSwitch();

  **mOpeningRequested = false;
  resetZeroCrossingTime();
  mResetZeroCrossingHistory = false;

  if (mSwitchingMode == SwitchingMode::SpaceVector) {
    for (int p = 0; p < 3; ++p)
      mCompOpen[p] = !mPoleMask[p];
    SPDLOG_LOGGER_INFO(mSLog, "DP space-vector closing command: poles closed.");
    return;
  }

  // All poles close together; the compensation then holds zero voltage
  if (mSwitchingMode == SwitchingMode::Compensated) {
    mCompOpen.fill(false);
    **mCompHandedOver = false;
    setPoleClosed(true);
    SPDLOG_LOGGER_INFO(mSLog, "DP compensated closing command: poles closed.");
    return;
  }

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    setPoleClosed(true);
    resetExponentialTransition();

    SPDLOG_LOGGER_INFO(mSLog, "DP switch closing command: pole closed.");
    return;
  }

  if (mSwitchingMode != SwitchingMode::ExponentialZCSEmulation) {
    **mPoleClosed = true;
    resetExponentialTransition();
    synchronizeEffectiveResistance(true);

    SPDLOG_LOGGER_INFO(mSLog, "DP switch closing command: pole closed.");
    return;
  }

  if (**mPoleClosed && !**mExponentialTransitionActive) {
    resetExponentialTransition();
    synchronizeEffectiveResistance(true);
    return;
  }

  // Energising a branch is the mirror image of interrupting it: the same
  // resistance path is traversed from R_open towards R_closed. The DP envelope
  // cannot represent the voltage step across a capacitance any more than it can
  // represent an interrupted inductive current, so closing is ramped as well.
  validateExponentialResistanceParameters();
  resetExponentialTransition();
  **mExponentialTransitionActive = true;
  **mExponentialTransitionClosing = true;
  **mExponentialProgress = 1.0;
  synchronizeEffectiveResistance(false);
  **mPoleClosed = false;

  SPDLOG_LOGGER_INFO(mSLog,
                     "DP exponential ZCS-emulation closing command received. "
                     "Switching duration={:.6e}s.",
                     mExponentialSwitchingTime);
}

void DP::Ph1::Switch::initializeFromNodesAndTerminals(Real frequency) {

  Real impedance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;
  (**mIntfVoltage)(0, 0) = initialSingleVoltage(1) - initialSingleVoltage(0);
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / impedance;

  // Power-flow initialization defines the initial physical breaker state.
  **mPoleClosed = **mIsClosed;
  **mOpeningRequested = false;
  resetZeroCrossingTime();
  resetExponentialTransition();
  synchronizeEffectiveResistance(**mIsClosed);

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\nSwitching mode: {:s}"
                     "\nInitial breaker state: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString((**mIntfVoltage)(0, 0)),
                     Logger::phasorToString((**mIntfCurrent)(0, 0)),
                     Logger::phasorToString(initialSingleVoltage(0)),
                     Logger::phasorToString(initialSingleVoltage(1)),
                     mSwitchingMode == SwitchingMode::Ideal
                         ? "Ideal"
                     : mSwitchingMode == SwitchingMode::CurrentZero
                         ? "CurrentZero"
                     : mSwitchingMode == SwitchingMode::Compensated
                         ? "Compensated"
                         : "ExponentialZCSEmulation",
                     **mIsClosed ? "closed" : "open");
}

void DP::Ph1::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                        Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;
  **mRightVector = Matrix::Zero(0, 0);

  // omega is the DP reference-frame angular frequency supplied by the solver.
  mShiftOmega = omega;

  // Final safeguard in case setSwitchingMode() was called before setParameters().
  **mPoleClosed = **mIsClosed;
  synchronizeEffectiveResistance(**mIsClosed);
  resetExponentialTransition();

  mPoleClosedPrev = **mPoleClosed;
  mIsClosedPrev = **mIsClosed;

  **mInstantCurrent = reconstructInstantaneousCurrent(0.0);

  mPreviousInstantaneousCurrent = **mInstantCurrent;
  mEffectiveResistancePrev = **mEffectiveResistance;

  mPreviousCurrentTime = 0.0;
  mPreviousCurrentValid = true;
  mResetZeroCrossingHistory = false;

  // the network pole stays open, the poles live in the compensation
  if (mSwitchingMode == SwitchingMode::SpaceVector) {
    if (!mZeroNetwork) {
      throw std::invalid_argument("DP::Ph1::Switch space-vector mode requires "
                                  "a zero-sequence network.");
    }
    **mPoleClosed = false;
    mPoleClosedPrev = false;
    synchronizeEffectiveResistance(false);
    for (int p = 0; p < 3; ++p)
      mCompOpen[p] = !(**mIsClosed && mPoleMask[p]);
    mSvResultTime = -1.;
    mCompStarted = false;
    mCompInstCount = 0;
    mCompArmed = false;
    mSvCurrent = Complex(0., 0.);
    mSvZeroCurrent = 0.;
    **mCompensationActive = true;
    **mCompHandedOver = true;
  }
}

void DP::Ph1::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  MNAStampUtils::stampAdmittance(currentConductance(), systemMatrix,
                                 matrixNodeIndex(0), matrixNodeIndex(1),
                                 terminalNotGrounded(0), terminalNotGrounded(1),
                                 mSLog);
}

void DP::Ph1::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  // Conventional precomputed two-state path. The CurrentZero and exponential
  // modes bypass this by returning supportsPrecomputedSystemMatrices() == false.
  Complex conductance = (closed) ? Complex(1. / **mClosedResistance, 0)
                                 : Complex(1. / **mOpenResistance, 0);

  MNAStampUtils::stampAdmittance(conductance, systemMatrix, matrixNodeIndex(0),
                                 matrixNodeIndex(1), terminalNotGrounded(0),
                                 terminalNotGrounded(1), mSLog);
}

void DP::Ph1::Switch::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {}

Bool DP::Ph1::Switch::mnaIsClosed() {
  if (mSwitchingMode == SwitchingMode::Ideal)
    return **mIsClosed;

  return **mPoleClosed;
}

Bool DP::Ph1::Switch::supportsPrecomputedSystemMatrices() const {
  return mSwitchingMode == SwitchingMode::Ideal;
}

Bool DP::Ph1::Switch::requestsQuadratureCorrection() const {
  // Only the current-zero mode leaves a quadrature part behind. Ideal switching
  // is unchanged by design, and the exponential ramp already avoids the
  // discontinuity, so correcting at the end of its ramp would reintroduce one.
  return mSwitchingMode == SwitchingMode::CurrentZero;
}

Bool DP::Ph1::Switch::takeSwitchingEvent() {
  const Bool pending = mSwitchingEventPending;
  mSwitchingEventPending = false;
  return pending;
}

std::vector<MNASwitchInterface::SwitchedPole>
DP::Ph1::Switch::takeSwitchedPoles() {
  if (!mSwitchedPolePending)
    return {};
  mSwitchedPolePending = false;

  // An opening pole turns in the post-step after the current is updated, so
  // the interface current is still the one the closed pole carried. The
  // voltage is read from the nodes, which hold the previous step until the
  // solver updates them, because a closing pole turns before the step and the
  // post-step that would update the interface voltage may be scheduled out.
  SwitchedPole pole;
  pole.node0 = terminalNotGrounded(0) ? matrixNodeIndex(0) : -1;
  pole.node1 = terminalNotGrounded(1) ? matrixNodeIndex(1) : -1;
  pole.closedConductance = 1. / **mClosedResistance;
  pole.openConductance = 1. / **mOpenResistance;
  pole.closed = **mPoleClosed;
  pole.current = (**mIntfCurrent)(0, 0);
  pole.zeroTime = **mZeroCrossingTime;
  pole.voltage =
      mTerminals[1]->singleVoltage() - mTerminals[0]->singleVoltage();
  return {pole};
}

std::vector<MNASwitchInterface::StateHandover>
DP::Ph1::Switch::takeStateHandover() {
  std::vector<StateHandover> taken;
  taken.swap(mCompHandoverPending);
  return taken;
}

void DP::Ph1::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
  (**mIntfVoltage)(0, 0) = 0;
  if (terminalNotGrounded(1))
    (**mIntfVoltage)(0, 0) =
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  if (terminalNotGrounded(0))
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}

void DP::Ph1::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = currentConductance() * (**mIntfVoltage)(0, 0);
}

void DP::Ph1::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {

  prevStepDependencies.push_back(mPoleClosed);
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);

  modifiedAttributes.push_back(mOpeningRequested);
  modifiedAttributes.push_back(mPoleClosed);

  modifiedAttributes.push_back(mInstantCurrent);
  modifiedAttributes.push_back(mZeroCrossingTime);

  modifiedAttributes.push_back(mExponentialTransitionActive);
  modifiedAttributes.push_back(mExponentialTransitionClosing);
  modifiedAttributes.push_back(mExponentialProgress);
  modifiedAttributes.push_back(mExponentialTransitionStartTime);
  modifiedAttributes.push_back(mExponentialTransitionEndTime);
  modifiedAttributes.push_back(mEffectiveResistance);

  modifiedAttributes.push_back(mCompensationCurrent);
  modifiedAttributes.push_back(mCompHandedOver);
  for (auto &correction : mCompNodeCorrections)
    modifiedAttributes.push_back(correction);
}

void DP::Ph1::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                      Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);

  **mInstantCurrent = reconstructInstantaneousCurrent(time);

  if (mSwitchingMode == SwitchingMode::CurrentZero) {
    updateZeroCrossingState(time);
  } else if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    updateExponentialTransition(time);
  } else if (mSwitchingMode == SwitchingMode::SpaceVector) {
    (**mIntfCurrent)(0, 0) -= mSvCurrent;
    **mInstantCurrent = std::real(mSvCurrent *
                                  std::polar<Real>(1.0, mShiftOmega * time)) +
                        mSvZeroCurrent;
  } else if (mSwitchingMode == SwitchingMode::Compensated) {
    if (**mCompensationActive) {
      updateCompensation(time);
    } else {
      std::array<Real, 3> inst;
      for (int p = 0; p < 3; ++p)
        inst[p] = -std::real((**mIntfCurrent)(0, 0) *
                             std::polar<Real>(1.0, mShiftOmega * time -
                                                       2. * PI * p / 3.));
      updatePoleCurrents(inst);
    }
  }
}

Bool DP::Ph1::Switch::hasParameterChanged() {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    // Check if state of switch changed
    if (mIsClosedPrev != **mIsClosed) {
      mIsClosedPrev = **mIsClosed;
      return true; //recompute system matrix
    }

    return false; // do not recompute system matrix
  }

  if (mSwitchingMode == SwitchingMode::SpaceVector)
    return false;

  if (mSwitchingMode == SwitchingMode::ExponentialZCSEmulation) {
    if (**mEffectiveResistance != mEffectiveResistancePrev) {
      mEffectiveResistancePrev = **mEffectiveResistance;
      return true;
    }

    return false;
  }

  if (**mPoleClosed != mPoleClosedPrev) {
    mPoleClosedPrev = **mPoleClosed;
    return true;
  }

  return false;
}

void DP::Ph1::Switch::setPoleClosed(Bool closed) {
  // Recorded where the pole actually turns, so it is independent of which
  // solver path runs and of hasParameterChanged() having consumed the change.
  if (**mPoleClosed != closed) {
    mSwitchingEventPending = true;
    mSwitchedPolePending = true;
  }

  **mPoleClosed = closed;

  // Keep the resistance diagnostics synchronized with the discrete
  // CurrentZero pole state. Exponential mode manages R(t) independently.
  if (mSwitchingMode == SwitchingMode::CurrentZero)
    synchronizeEffectiveResistance(closed);
}

void DP::Ph1::Switch::resetZeroCrossingTime() { **mZeroCrossingTime = -1.0; }

void DP::Ph1::Switch::synchronizeEffectiveResistance(Bool closed) {
  **mEffectiveResistance = closed ? **mClosedResistance : **mOpenResistance;
}

void DP::Ph1::Switch::resetExponentialTransition() {
  **mExponentialTransitionActive = false;
  **mExponentialTransitionClosing = false;
  **mExponentialProgress = 0.0;
  **mExponentialTransitionStartTime = -1.0;
  **mExponentialTransitionEndTime = -1.0;
  mExponentialTransitionStarted = false;
}

void DP::Ph1::Switch::validateExponentialResistanceParameters() const {
  if (mExponentialSwitchingTime <= 0.0) {
    throw std::invalid_argument(
        "DP::Ph1::Switch exponential switching time must be positive.");
  }

  if (**mClosedResistance <= 0.0 || **mOpenResistance <= 0.0) {
    throw std::invalid_argument("DP::Ph1::Switch exponential mode requires "
                                "positive open and closed resistances.");
  }
}

Real DP::Ph1::Switch::exponentialResistance(Real alpha) const {
  const Real rClosed = **mClosedResistance;
  const Real rOpen = **mOpenResistance;
  const Real boundedAlpha = std::max(0.0, std::min(1.0, alpha));

  return std::exp(std::log(rClosed) +
                  boundedAlpha * (std::log(rOpen) - std::log(rClosed)));
}

void DP::Ph1::Switch::updateExponentialTransition(Real time) {
  if (!**mExponentialTransitionActive)
    return;

  if (!mExponentialTransitionStarted) {
    mExponentialTransitionStarted = true;
    **mExponentialTransitionStartTime = time;
    **mExponentialTransitionEndTime = time + mExponentialSwitchingTime;
  }

  const Real targetTime = time + mTimeStep;
  const Real elapsed = (targetTime - **mExponentialTransitionStartTime) /
                       mExponentialSwitchingTime;
  const Real boundedElapsed = std::max(0.0, std::min(1.0, elapsed));

  const Bool closing = **mExponentialTransitionClosing;

  // alpha is the position on the resistance path: 0 at R_closed, 1 at R_open.
  const Real alpha = closing ? 1.0 - boundedElapsed : boundedElapsed;

  **mExponentialProgress = alpha;
  **mEffectiveResistance = exponentialResistance(alpha);

  if (boundedElapsed >= 1.0) {
    synchronizeEffectiveResistance(closing);
    **mPoleClosed = closing;
    **mOpeningRequested = false;
    **mExponentialTransitionActive = false;
    **mExponentialTransitionClosing = false;

    SPDLOG_LOGGER_INFO(
        mSLog,
        "DP exponential ZCS-emulation {:s} completed: start={:.9f}s, "
        "end={:.9f}s, duration={:.6e}s.",
        closing ? "closing" : "opening", **mExponentialTransitionStartTime,
        **mExponentialTransitionEndTime, mExponentialSwitchingTime);
  }
}

Complex DP::Ph1::Switch::currentConductance() const {
  if (mSwitchingMode == SwitchingMode::Ideal) {
    return (**mIsClosed) ? Complex(1. / **mClosedResistance, 0)
                         : Complex(1. / **mOpenResistance, 0);
  }

  if (mSwitchingMode == SwitchingMode::SpaceVector)
    return Complex(1. / **mOpenResistance, 0);

  if (mSwitchingMode == SwitchingMode::CurrentZero ||
      mSwitchingMode == SwitchingMode::Compensated) {
    return (**mPoleClosed) ? Complex(1. / **mClosedResistance, 0)
                           : Complex(1. / **mOpenResistance, 0);
  }

  return Complex(1. / **mEffectiveResistance, 0);
}

Real DP::Ph1::Switch::reconstructInstantaneousCurrent(Real time) const {
  // Restore the reference-frequency carrier removed by the DP formulation.
  // DP::Ph1 envelopes keep the power-flow amplitude scaling, so no RMS-to-peak
  // conversion is applied here; the zero crossing instant does not depend on it.
  const Complex carrier = std::polar<Real>(1.0, mShiftOmega * time);

  return std::real((**mIntfCurrent)(0, 0) * carrier);
}

Bool DP::Ph1::Switch::currentCrossedZero(Real previousCurrent,
                                         Real current) const {
  if (std::abs(current) <= mZeroCrossingTolerance)
    return true;

  return (previousCurrent > mZeroCrossingTolerance &&
          current < -mZeroCrossingTolerance) ||
         (previousCurrent < -mZeroCrossingTolerance &&
          current > mZeroCrossingTolerance);
}

Real DP::Ph1::Switch::interpolateZeroCrossingTime(Real previousCurrent,
                                                  Real current,
                                                  Real previousTime,
                                                  Real currentTime) const {
  const Real denominator = std::abs(previousCurrent) + std::abs(current);

  if (denominator <= mZeroCrossingTolerance)
    return currentTime;

  const Real fraction = std::abs(previousCurrent) / denominator;

  return previousTime + fraction * (currentTime - previousTime);
}

void DP::Ph1::Switch::updateCompensation(Real time) {
  // phase envelopes of the closed-run current from terminal 1 to 0,
  // phase k shifted by -2 pi k / 3
  const Complex ib = -(**mIntfCurrent)(0, 0);
  Phases ic;
  for (int p = 0; p < 3; ++p)
    ic[p] = ib * std::polar<Real>(1.0, -2. * PI * p / 3.);
  const Int set = **mTheveninIndex;
  if (set < 0 || set >= static_cast<Int>(mThevenin1.size())) {
    throw std::out_of_range("DP::Ph1::Switch thevenin_index out of range.");
  }
  const TheveninEquivalent &th1 = mThevenin1[set];
  const TheveninEquivalent &th0 = mThevenin0[set];
  // all poles open through this step: hand over at its end
  const Bool handover = mCompHandover && **mPoleClosed && mCompOpen[0] &&
                        mCompOpen[1] && mCompOpen[2];
  const Bool first = !mCompStarted;
  if (first) {
    for (auto &x : mCompX1)
      x = Matrix::Zero(th1.A.rows(), 1);
    mCompX0 = Matrix::Zero(th0.A.rows(), 1);
    mCompStarted = true;
  }

  // outputs of the earlier states
  std::array<Matrix, 3> y1;
  for (int p = 0; p < 3; ++p)
    y1[p] = th1.C * mCompX1[p];
  const Matrix y0 = th0.C * mCompX0;

  // closed poles: v_p = y_p + G1 (D_p - m) + G0 m = 0 with m = mean(D);
  // open poles: no current
  const Matrix g1 = th1.D.topRows(2);
  const Matrix g0 = th0.D.topRows(2);
  Matrix m = Matrix::Zero(6, 6);
  Matrix r = Matrix::Zero(6, 1);
  for (int p = 0; p < 3; ++p) {
    if (mCompOpen[p]) {
      m.block(2 * p, 2 * p, 2, 2).setIdentity();
      r(2 * p, 0) = -ic[p].real();
      r(2 * p + 1, 0) = -ic[p].imag();
      continue;
    }
    for (int q = 0; q < 3; ++q)
      m.block(2 * p, 2 * q, 2, 2) =
          (g0 - g1) / 3. + (p == q ? g1 : Matrix::Zero(2, 2));
    r.middleRows(2 * p, 2) = -(y1[p].topRows(2) + y0.topRows(2));
  }
  const Matrix dn = m.partialPivLu().solve(r);
  Phases d;
  for (int p = 0; p < 3; ++p)
    d[p] = Complex(dn(2 * p, 0), dn(2 * p + 1, 0));

  // a pole opens at the zero of its smoothed current; the smoothing removes
  // the undamped step-to-step oscillation of the trapezoidal rule
  std::array<Real, 3> inst;
  for (int p = 0; p < 3; ++p)
    inst[p] = std::real((ic[p] + d[p]) *
                        std::polar<Real>(1.0, mShiftOmega * time));
  const Bool valid = mCompInstCount >= 3;
  const std::array<Real, 3> before = mCompSmooth1;
  const std::array<Real, 3> now = updatePoleCurrents(inst);
  if (valid && mCompArmed && **mOpeningRequested)
    for (int p = 0; p < 3; ++p) {
      if (mCompOpen[p])
        continue;
      if (before[p] * now[p] <= 0. &&
          std::abs(before[p]) + std::abs(now[p]) > 0.) {
        mCompOpen[p] = true;
        SPDLOG_LOGGER_INFO(mSLog, "DP compensated pole {} opened at t={:.9f}s.",
                           p, time);
      }
    }

  // sequence inputs, observed node corrections (phase-a frame), state update
  const Complex mean = (d[0] + d[1] + d[2]) / 3.;
  Matrix u0(2, 1);
  u0 << mean.real(), mean.imag();
  const Matrix v0 = y0 + th0.D * u0;
  std::array<Matrix, 3> v, xPrev, u;
  for (int p = 0; p < 3; ++p) {
    Matrix u1(2, 1);
    u1 << (d[p] - mean).real(), (d[p] - mean).imag();
    v[p] = y1[p] + th1.D * u1;
    xPrev[p] = mCompX1[p];
    u[p] = u1;
    mCompX1[p] = th1.A * mCompX1[p] + th1.B * u1;
  }
  if (handover)
    handOverToNetwork(th1, xPrev, u, time);
  const Complex a = std::polar<Real>(1.0, 2. * PI / 3.);
  for (std::size_t i = 0; i < mCompNodes.size(); ++i) {
    // sequence parts of the node correction, per phase envelopes
    auto at = [&](const Matrix &x) {
      return Complex(x(2 + 2 * i, 0), x(3 + 2 * i, 0));
    };
    const Complex c0 = mCompNodes[i].zeroSequence ? at(v0) : Complex(0, 0);
    const Complex c1 = (at(v[0]) + a * at(v[1]) + a * a * at(v[2])) / 3.;
    const Complex c2 = (at(v[0]) + a * a * at(v[1]) + a * at(v[2])) / 3. *
                       std::polar<Real>(1.0, -2. * mCompNodes[i].phaseShift);
    const Phases c = {c0 + c1 + c2, c0 + a * a * c1 + a * c2,
                      c0 + a * c1 + a * a * c2};
    for (int p = 0; p < 3; ++p)
      (**mCompNodeCorrections[i])(p, 0) =
          c[p] * std::polar<Real>(1.0, 2. * PI * p / 3.);
  }
  mCompX0 = th0.A * mCompX0 + th0.B * u0;

  for (int p = 0; p < 3; ++p)
    (**mCompensationCurrent)(p, 0) = d[p];
  mCompArmed = **mOpeningRequested;
}

std::vector<std::pair<Int, Int>> DP::Ph1::Switch::mnaCompensationTerminals() {
  return {{terminalNotGrounded(0) ? static_cast<Int>(matrixNodeIndex(0)) : -1,
           terminalNotGrounded(1) ? static_cast<Int>(matrixNodeIndex(1))
                                  : -1}};
}

MNAZeroSequenceNetwork::Ptr DP::Ph1::Switch::mnaZeroSequenceNetwork() {
  return mZeroNetwork;
}

void DP::Ph1::Switch::mnaCompensationConditions(Real time, Matrix &a,
                                                Matrix &b, Matrix &c) {
  a = Matrix::Zero(3, 1);
  b = Matrix::Zero(3, 1);
  c = Matrix::Zero(3, 1);
  for (int p = 0; p < 3; ++p) {
    if (mCompOpen[p]) {
      b(p, 0) = 1.;
    } else {
      a(p, 0) = 1.;
      b(p, 0) = **mClosedResistance;
    }
  }
}

Bool DP::Ph1::Switch::mnaCompensationResult(Real time,
                                            const Matrix &injection,
                                            const Matrix &zeroCurrent,
                                            const Matrix &phaseCurrent) {
  mSvCurrent = Complex(injection(0, 0), injection(1, 0));
  mSvZeroCurrent = zeroCurrent(0, 0);
  for (int p = 0; p < 3; ++p)
    (**mCompensationCurrent)(p, 0) =
        mSvCurrent * std::polar<Real>(1.0, -2. * PI * p / 3.) +
        mSvZeroCurrent * std::polar<Real>(1.0, -mShiftOmega * time);
  // the first solve of a step takes over the final currents of the previous
  // step into the history; every solve of the step tests against it
  if (time != mSvResultTime) {
    if (mSvResultTime >= 0.)
      updatePoleCurrents(mSvPendingCurrent);
    mSvResultTime = time;
    mSvArmedStep = mCompArmed;
    mCompArmed = **mOpeningRequested;
  }
  std::array<Real, 3> inst;
  for (int p = 0; p < 3; ++p)
    inst[p] = phaseCurrent(p, 0);
  mSvPendingCurrent = inst;

  // between opening and closing command, a pole opens at the zero of its
  // smoothed instantaneous current
  const Bool valid = mCompInstCount >= 3;
  const std::array<Real, 3> before = mCompSmooth1;
  std::array<Real, 3> now = inst;
  if (mCompInstCount >= 2)
    for (int p = 0; p < 3; ++p)
      now[p] = 0.25 * (3. * inst[p] + 2. * mCompInst1[p] - mCompInst2[p]);
  Bool opened = false;
  if (valid && mSvArmedStep && **mOpeningRequested)
    for (int p = 0; p < 3; ++p) {
      if (mCompOpen[p])
        continue;
      if (before[p] * now[p] <= 0. &&
          std::abs(before[p]) + std::abs(now[p]) > 0.) {
        mCompOpen[p] = true;
        opened = true;
        SPDLOG_LOGGER_INFO(mSLog, "DP space-vector pole {} opened at t={:.9f}s.",
                           p, time);
      }
    }
  // the pole interrupts in this step; the solver solves again
  return opened && mSameStepOpening;
}

DP::Ph1::ZeroSequenceEquivalent::ZeroSequenceEquivalent(
    String name, UInt ports, const Switch::TheveninEquivalent &equivalent,
    const std::vector<Switch::ObservedNode> &nodes)
    : IdentifiedObject(name, name),
      mTheveninIndex(mAttributes->create<Int>("thevenin_index", 0)),
      mPorts(ports), mSets{equivalent}, mNodes(nodes) {
  const Matrix::Index inputs = 2 * static_cast<Matrix::Index>(ports);
  const Matrix::Index rows =
      inputs + 2 * static_cast<Matrix::Index>(nodes.size());
  if (equivalent.C.rows() != rows || equivalent.D.rows() != rows ||
      equivalent.B.cols() != inputs || equivalent.D.cols() != inputs) {
    throw std::invalid_argument("DP::Ph1::ZeroSequenceEquivalent does not "
                                "match the ports and observed nodes.");
  }
  for (auto &node : nodes)
    mCorrections.push_back(mAttributes->create<MatrixComp>(
        "compensation_v_" + node.name, MatrixComp::Zero(3, 1)));
  mState = Matrix::Zero(equivalent.A.rows(), 1);
}

Int DP::Ph1::ZeroSequenceEquivalent::addEquivalent(
    const Switch::TheveninEquivalent &equivalent) {
  const auto &first = mSets[0];
  if (equivalent.A.rows() != first.A.rows() ||
      equivalent.C.rows() != first.C.rows() ||
      equivalent.D.cols() != first.D.cols() ||
      equivalent.stateNames != first.stateNames) {
    throw std::invalid_argument("DP::Ph1::ZeroSequenceEquivalent further sets "
                                "must match the first.");
  }
  mSets.push_back(equivalent);
  return static_cast<Int>(mSets.size()) - 1;
}

const DP::Ph1::Switch::TheveninEquivalent &
DP::Ph1::ZeroSequenceEquivalent::current() const {
  const Int set = **mTheveninIndex;
  if (set < 0 || set >= static_cast<Int>(mSets.size()))
    throw std::out_of_range(
        "DP::Ph1::ZeroSequenceEquivalent thevenin_index out of range.");
  return mSets[set];
}

Matrix DP::Ph1::ZeroSequenceEquivalent::zeroFreeVoltage() {
  // outputs are v0 - v1
  return -(current().C.topRows(2 * mPorts) * mState);
}

Matrix DP::Ph1::ZeroSequenceEquivalent::zeroVoltageResponse() {
  return -current().D.topRows(2 * mPorts);
}

void DP::Ph1::ZeroSequenceEquivalent::zeroAdvance(const Matrix &input) {
  const auto &th = current();
  const Matrix y = th.C * mState + th.D * input;
  const Matrix::Index offset = 2 * static_cast<Matrix::Index>(mPorts);
  for (std::size_t i = 0; i < mNodes.size(); ++i) {
    const Complex c0 =
        mNodes[i].zeroSequence
            ? Complex(y(offset + 2 * i, 0), y(offset + 2 * i + 1, 0))
            : Complex(0, 0);
    for (int p = 0; p < 3; ++p)
      (**mCorrections[i])(p, 0) = c0 * std::polar<Real>(1.0, 2. * PI * p / 3.);
  }
  mState = th.A * mState + th.B * input;
}

std::array<Real, 3>
DP::Ph1::Switch::updatePoleCurrents(const std::array<Real, 3> &inst) {
  // (3 x[k] + 2 x[k-1] - x[k-2]) / 4: exact for a linear course, zero for an
  // alternation (-1)^k
  std::array<Real, 3> smooth = inst;
  if (mCompInstCount >= 2)
    for (int p = 0; p < 3; ++p)
      smooth[p] = 0.25 * (3. * inst[p] + 2. * mCompInst1[p] - mCompInst2[p]);
  mCompInst2 = mCompInst1;
  mCompInst1 = inst;
  mCompSmooth1 = smooth;
  mCompInstCount = std::min(mCompInstCount + 1, 3);
  return smooth;
}

void DP::Ph1::Switch::handOverToNetwork(const TheveninEquivalent &th,
                                        const std::array<Matrix, 3> &xPrev,
                                        const std::array<Matrix, 3> &u,
                                        Real time) {
  if (th.elements.empty() || th.Ex.rows() != 4 * static_cast<Matrix::Index>(
                                                     th.elements.size())) {
    throw std::invalid_argument(
        "DP::Ph1::Switch network handover needs the storage elements of the "
        "Thevenin equivalent.");
  }

  // storage element states at the end of the step as space vector
  // P + conj(N) e^(-2 j omega t) in the phase-a frame, zero sequence apart
  const Complex a = std::polar<Real>(1.0, 2. * PI / 3.);
  const Complex carrier2 = std::polar<Real>(1.0, -2. * mShiftOmega * time);
  std::array<Matrix, 3> s;
  for (int p = 0; p < 3; ++p)
    s[p] = th.Ex * xPrev[p] + th.Eu * u[p];
  for (std::size_t e = 0; e < th.elements.size(); ++e) {
    const auto &element = th.elements[e];
    const Real shift = element.phaseShift;
    const Matrix::Index r = 4 * static_cast<Matrix::Index>(e);
    auto spaceVector = [&](Matrix::Index row) {
      Phases z;
      for (int p = 0; p < 3; ++p)
        z[p] = Complex(s[p](row, 0), s[p](row + 1, 0));
      const Complex pos = (z[0] + a * z[1] + a * a * z[2]) / 3.;
      const Complex neg = (z[0] + a * a * z[1] + a * z[2]) / 3. *
                          std::polar<Real>(1.0, -2. * shift);
      return pos + carrier2 * std::conj(neg);
    };
    mCompHandoverPending.push_back(
        {element, spaceVector(r), spaceVector(r + 2)});
  }

  // nothing physical stays in the per-phase equivalents
  for (auto &x : mCompX1)
    x.setZero();

  **mCompHandedOver = true;
  setPoleClosed(false);
  SPDLOG_LOGGER_INFO(mSLog, "DP compensated poles handed over to the network.");
}

void DP::Ph1::Switch::updateZeroCrossingState(Real time) {
  const Real current = **mInstantCurrent;

  // Keep a current history while no opening command is active.
  if (!**mOpeningRequested) {
    mPreviousInstantaneousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    return;
  }

  // First post-command sample: do not compare against the pre-command sample.
  if (mResetZeroCrossingHistory || !mPreviousCurrentValid) {
    if (**mPoleClosed && std::abs(current) <= mZeroCrossingTolerance) {
      setPoleClosed(false);
      **mZeroCrossingTime = time;

      SPDLOG_LOGGER_INFO(mSLog,
                         "DP pole opened at sampled physical current zero: "
                         "t={:.9f}s, i={:.6e}A",
                         time, current);
    }

    mPreviousInstantaneousCurrent = current;
    mPreviousCurrentTime = time;
    mPreviousCurrentValid = true;
    mResetZeroCrossingHistory = false;

    if (!**mPoleClosed)
      **mOpeningRequested = false;

    return;
  }

  if (**mPoleClosed &&
      currentCrossedZero(mPreviousInstantaneousCurrent, current)) {
    const Real zeroTime = interpolateZeroCrossingTime(
        mPreviousInstantaneousCurrent, current, mPreviousCurrentTime, time);

    setPoleClosed(false);
    **mZeroCrossingTime = zeroTime;

    SPDLOG_LOGGER_INFO(mSLog,
                       "DP physical current zero detected: "
                       "t_z={:.9f}s, i_prev={:.6e}A, i={:.6e}A. Pole opened.",
                       zeroTime, mPreviousInstantaneousCurrent, current);
  }

  mPreviousInstantaneousCurrent = current;
  mPreviousCurrentTime = time;
  mPreviousCurrentValid = true;

  if (!**mPoleClosed) {
    **mOpeningRequested = false;

    SPDLOG_LOGGER_INFO(mSLog, "DP current-zero interruption completed: "
                              "the breaker pole is open.");
  }
}
