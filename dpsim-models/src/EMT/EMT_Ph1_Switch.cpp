/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#include <cmath>

#include <dpsim-models/EMT/EMT_Ph1_Switch.h>

using namespace CPS;

EMT::Ph1::Switch::Switch(String uid, String name, Logger::Level logLevel)
    : MNASimPowerComp<Real>(uid, name, false, true, logLevel),
      Base::Ph1::Switch(mAttributes),
      mPoleClosed(mAttributes->create<Bool>("pole_closed")),
      mZeroCrossingTime(mAttributes->create<Real>("zero_crossing_time")) {
  setTerminalNumber(2);
  **mIntfVoltage = Matrix::Zero(1, 1);
  **mIntfCurrent = Matrix::Zero(1, 1);
  **mPoleClosed = false;
  **mZeroCrossingTime = -1.0;
}

SimPowerComp<Real>::Ptr EMT::Ph1::Switch::clone(String name) {
  auto copy = Switch::make(name, mLogLevel);
  copy->setParameters(**mOpenResistance, **mClosedResistance, **mIsClosed);
  copy->setSwitchingMode(mSwitchingMode);
  copy->setZeroCrossingTolerance(mZeroCrossingTolerance);
  return copy;
}

void EMT::Ph1::Switch::setSwitchingMode(SwitchingMode mode) {
  mSwitchingMode = mode;
  **mPoleClosed = **mIsClosed;
  **mZeroCrossingTime = -1.0;
  mOpeningRequested = false;
  mPreviousCurrentValid = false;
}

void EMT::Ph1::Switch::openSwitch() {
  Base::Ph1::Switch::openSwitch();

  if (mSwitchingMode == SwitchingMode::Ideal) {
    **mPoleClosed = false;
    return;
  }

  if (!**mPoleClosed)
    return;

  // The pole keeps conducting until the post-step sees the current cross zero
  mOpeningRequested = true;
  mPreviousCurrentValid = false;
  **mZeroCrossingTime = -1.0;
  SPDLOG_LOGGER_INFO(mSLog, "Current-zero opening command received.");
}

void EMT::Ph1::Switch::closeSwitch() {
  Base::Ph1::Switch::closeSwitch();
  mOpeningRequested = false;
  **mPoleClosed = true;
}

void EMT::Ph1::Switch::initializeFromNodesAndTerminals(Real frequency) {

  Real resistance = (**mIsClosed) ? **mClosedResistance : **mOpenResistance;

  (**mIntfVoltage)(0, 0) =
      RMS3PH_TO_PEAK1PH *
      (initialSingleVoltage(1) - initialSingleVoltage(0)).real();
  (**mIntfCurrent)(0, 0) = (**mIntfVoltage)(0, 0) / resistance;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nVoltage across: {:s}"
                     "\nCurrent: {:s}"
                     "\nTerminal 0 voltage: {:s}"
                     "\nTerminal 1 voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::matrixToString(**mIntfVoltage),
                     Logger::matrixToString(**mIntfCurrent),
                     Logger::phasorToString(RMS3PH_TO_PEAK1PH *
                                            initialSingleVoltage(0).real()),
                     Logger::phasorToString(RMS3PH_TO_PEAK1PH *
                                            initialSingleVoltage(1).real()));
}

void EMT::Ph1::Switch::mnaCompInitialize(Real omega, Real timeStep,
                                         Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  **mPoleClosed = **mIsClosed;
  mIsClosedPrev = **mIsClosed;
  mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
}

Bool EMT::Ph1::Switch::mnaIsClosed() {
  return mSwitchingMode == SwitchingMode::Ideal ? **mIsClosed : **mPoleClosed;
}

Bool EMT::Ph1::Switch::supportsPrecomputedSystemMatrices() const {
  return mSwitchingMode == SwitchingMode::Ideal;
}

void EMT::Ph1::Switch::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  Real conductance;

  conductance =
      mnaIsClosed() ? 1. / (**mClosedResistance) : 1. / (**mOpenResistance);

  MNAStampUtils::stampConductance(conductance, systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::Ph1::Switch::mnaCompApplySwitchSystemMatrixStamp(
    Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {
  Real conductance;

  conductance =
      (closed) ? 1. / (**mClosedResistance) : 1. / (**mOpenResistance);

  MNAStampUtils::stampConductance(conductance, systemMatrix, matrixNodeIndex(0),
                                  matrixNodeIndex(1), terminalNotGrounded(0),
                                  terminalNotGrounded(1), mSLog);
}

void EMT::Ph1::Switch::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {}

void EMT::Ph1::Switch::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
  modifiedAttributes.push_back(mPoleClosed);
}

void EMT::Ph1::Switch::mnaCompPostStep(Real time, Int timeStepCount,
                                       Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
  if (mSwitchingMode == SwitchingMode::CurrentZero)
    updateZeroCrossingState(time);
}

void EMT::Ph1::Switch::updateZeroCrossingState(Real time) {
  const Real current = (**mIntfCurrent)(0, 0);

  if (mOpeningRequested && **mPoleClosed) {
    const Bool crossed =
        std::abs(current) <= mZeroCrossingTolerance ||
        (mPreviousCurrentValid &&
         ((mPreviousCurrent > mZeroCrossingTolerance &&
           current < -mZeroCrossingTolerance) ||
          (mPreviousCurrent < -mZeroCrossingTolerance &&
           current > mZeroCrossingTolerance)));

    if (crossed) {
      const Real denominator = std::abs(mPreviousCurrent) + std::abs(current);
      **mZeroCrossingTime =
          (mPreviousCurrentValid && denominator > mZeroCrossingTolerance)
              ? mPreviousTime + std::abs(mPreviousCurrent) / denominator *
                                    (time - mPreviousTime)
              : time;
      **mPoleClosed = false;
      mOpeningRequested = false;
      SPDLOG_LOGGER_INFO(mSLog,
                         "Current zero detected: t_z={:.9f}s, i_prev={:.6e}A, "
                         "i={:.6e}A. Pole opened.",
                         **mZeroCrossingTime, mPreviousCurrent, current);
    }
  }

  mPreviousCurrent = current;
  mPreviousTime = time;
  mPreviousCurrentValid = true;
}

void EMT::Ph1::Switch::mnaCompUpdateVoltage(const Matrix &leftVector) {
  // Voltage across component is defined as V1 - V0
  **mIntfVoltage = Matrix::Zero(1, 1);
  if (terminalNotGrounded(1)) {
    (**mIntfVoltage)(0, 0) =
        Math::realFromVectorElement(leftVector, matrixNodeIndex(1, 0));
  }
  if (terminalNotGrounded(0)) {
    (**mIntfVoltage)(0, 0) =
        (**mIntfVoltage)(0, 0) -
        Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  }
}

void EMT::Ph1::Switch::mnaCompUpdateCurrent(const Matrix &leftVector) {
  (**mIntfCurrent)(0, 0) = mnaIsClosed()
                               ? (**mIntfVoltage)(0, 0) / (**mClosedResistance)
                               : (**mIntfVoltage)(0, 0) / (**mOpenResistance);
}

Bool EMT::Ph1::Switch::hasParameterChanged() {
  // Check if state of switch changed
  if (!(mIsClosedPrev == this->mnaIsClosed())) {
    mIsClosedPrev = this->mnaIsClosed();
    return 1; //recompute system matrix
  } else {
    return 0; // do not recompute system matrix
  }
};
