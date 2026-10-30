// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorTrStab.h>
using namespace CPS;

DP::Ph3::SynchronGeneratorTrStab::SynchronGeneratorTrStab(
    String uid, String name, Logger::Level logLevel)
    : Base::SynchronGenerator(mAttributes),
      CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mEp(mAttributes->create<Complex>("Ep")),
      mEp_abs(mAttributes->create<Real>("Ep_mag")),
      mDelta_p(mAttributes->create<Real>("delta_r")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(2);
  setTerminalNumber(1);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);
}

SimPowerComp<Complex>::Ptr
DP::Ph3::SynchronGeneratorTrStab::clone(String name) {
  auto copy = SynchronGeneratorTrStab::make(name, mLogLevel);
  copy->setStandardParametersPU(mNomPower, mNomVolt, mNomFreq, mXpd / mBase_Z,
                                **mInertia, **mRs, mKd);
  return copy;
}

void DP::Ph3::SynchronGeneratorTrStab::setStandardParametersSI(
    Real nomPower, Real nomVolt, Real nomFreq, Int polePairNumber, Real Rs,
    Real Lpd, Real inertiaJ, Real Kd) {
  setBaseParameters(nomPower, nomVolt, nomFreq);

  mParameterType = ParameterType::statorReferred;
  mStateType = StateType::statorReferred;

  // H = J * 0.5 * omegaNom^2 / polePairNumber
  **mInertia = calcHfromJ(inertiaJ, 2 * PI * nomFreq, polePairNumber);
  mXpd = mNomOmega * Lpd;
  mLpd = Lpd;
  mKd = Kd;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Parameters ---"
                     "\nimpedance: {:f}"
                     "\ninductance: {:f}",
                     mXpd, mLpd);
}

void DP::Ph3::SynchronGeneratorTrStab::setStandardParametersPU(
    Real nomPower, Real nomVolt, Real nomFreq, Real Xpd, Real inertia, Real Rs,
    Real D) {
  setBaseParameters(nomPower, nomVolt, nomFreq);

  mParameterType = ParameterType::statorReferred;
  mStateType = StateType::statorReferred;

  **mInertia = inertia;
  mXpd = Xpd * mBase_Z;
  mLpd = Xpd * mBase_L;
  **mRs = Rs;
  mKd = D * mNomPower / mNomOmega;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Parameters ---"
                     "\nimpedance: {:f}"
                     "\ninductance: {:f}",
                     mXpd, mLpd);
}

void DP::Ph3::SynchronGeneratorTrStab::setInitialValues(Complex elecPower,
                                                        Real mechPower) {
  mInitElecPower = elecPower;
  mInitMechPower = mechPower;
}

void DP::Ph3::SynchronGeneratorTrStab::createSubComponents() {
  if (mSubCompCreated)
    return;
  mSubCompCreated = true;

  mSubVoltageSource = DP::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);
  mSubVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
  mSubVoltageSource->setVirtualNodeAt(mVirtualNodes[1], 0);
  addMNASubComponent(mSubVoltageSource,
                     MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mSubInductor = DP::Ph3::Inductor::make(**mName + "_ind", mLogLevel);
  mSubInductor->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mLpd));
  mSubInductor->connect({mVirtualNodes[0], terminal(0)->node()});
  addMNASubComponent(mSubInductor, MNA_SUBCOMP_TASK_ORDER::TASK_AFTER_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
}

void DP::Ph3::SynchronGeneratorTrStab::initializeParentFromNodesAndTerminals(
    Real frequency) {
  **mOmMech = mNomOmega;

  mInitElecPower = (mInitElecPower == Complex(0, 0))
                       ? -terminal(0)->singlePower()
                       : mInitElecPower;
  mInitMechPower = (mInitElecPower == Complex(0, 0)) ? mInitElecPower.real()
                                                     : mInitMechPower;

  // Balanced initialization from the phase a envelope, as in EMT::Ph3, where
  // the three-phase power is 3/2 V I* in peak phasors
  const Complex voltage = initialSingleVoltage(0);
  const Complex current = std::conj(-2. / 3. * mInitElecPower / voltage);
  **mIntfVoltage = CPS::Math::singlePhaseVariableToThreePhase(voltage);
  **mIntfCurrent = CPS::Math::singlePhaseVariableToThreePhase(current);

  mImpedance = Complex(**mRs, mXpd);
  **mEp = voltage - mImpedance * current;
  **mEp_abs = Math::abs(**mEp);
  **mDelta_p = Math::phase(**mEp);

  **mElecActivePower = (3. / 2. * voltage * std::conj(-current)).real();
  **mMechPower = **mElecActivePower - mKd * (**mOmMech - mNomOmega);

  mVirtualNodes[0]->setInitialVoltage(**mEp);
  mSubVoltageSource->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(**mEp));

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialize according to powerflow ---"
                     "\nTerminal 0 voltage: {:e}<{:e}"
                     "\nVoltage behind reactance: {:e}<{:e}"
                     "\ninitial electrical power: {:e}+j{:e}"
                     "\nactive electrical power: {:e}"
                     "\nmechanical power: {:e}"
                     "\n--- End of powerflow initialization ---",
                     Math::abs(voltage), Math::phaseDeg(voltage),
                     Math::abs(**mEp), Math::phaseDeg(**mEp),
                     mInitElecPower.real(), mInitElecPower.imag(),
                     **mElecActivePower, **mMechPower);
}

void DP::Ph3::SynchronGeneratorTrStab::step(Real time) {
  // The interface values are those of the previous step
  const Complex carrier = std::polar(1.0, mSystemOmega * (time - mTimeStep));
  Real power = 0.0;
  for (UInt phase = 0; phase < 3; ++phase)
    power += ((**mIntfVoltage)(phase, 0) * carrier).real() *
             ((**mIntfCurrent)(phase, 0) * carrier).real();
  **mElecActivePower = -power;

  // As in EMT::Ph3::SynchronGeneratorTrStab, which the model is compared with
  mKd = 1 * mNomPower;

  Real dOmMech =
      mNomOmega / (2. * **mInertia * mNomPower) *
      (**mMechPower - **mElecActivePower - mKd * (**mOmMech - mNomOmega));
  if (mBehaviour == Behaviour::MNASimulation)
    **mOmMech = **mOmMech + mTimeStep * dOmMech;
  Real dDelta_p = **mOmMech - mNomOmega;
  if (mBehaviour == Behaviour::MNASimulation) {
    **mDelta_p = **mDelta_p + mTimeStep * dDelta_p;
    **mEp = Complex(**mEp_abs * cos(**mDelta_p), **mEp_abs * sin(**mDelta_p));
  }
}

void DP::Ph3::SynchronGeneratorTrStab::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  mTimeStep = timeStep;
  mSystemOmega = omega;
  mMnaTasks.push_back(std::make_shared<AddBStep>(*this));
}

void DP::Ph3::SynchronGeneratorTrStab::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mIntfVoltage);
}

void DP::Ph3::SynchronGeneratorTrStab::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
}

void DP::Ph3::SynchronGeneratorTrStab::mnaParentPreStep(Real time,
                                                        Int timeStepCount) {
  step(time);
  mSubVoltageSource->mVoltageRef->set(
      CPS::Math::singlePhaseVariableToThreePhase(**mEp));
}

void DP::Ph3::SynchronGeneratorTrStab::AddBStep::execute(Real time,
                                                         Int timeStepCount) {
  **mGenerator.mRightVector = **mGenerator.mSubInductor->mRightVector +
                              **mGenerator.mSubVoltageSource->mRightVector;
}

void DP::Ph3::SynchronGeneratorTrStab::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateVoltage(**leftVector);
  mnaCompUpdateCurrent(**leftVector);
}

void DP::Ph3::SynchronGeneratorTrStab::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  for (UInt phase = 0; phase < 3; ++phase)
    (**mIntfVoltage)(phase, 0) = Math::complexFromVectorElement(
        leftVector, matrixNodeIndex(0, phase));
}

void DP::Ph3::SynchronGeneratorTrStab::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  **mIntfCurrent = **mSubInductor->mIntfCurrent;
}
