// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#include <dpsim-models/DP/DP_Ph3_AvVoltageSourceInverterDQ.h>

using namespace CPS;

DP::Ph3::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ(
    String uid, String name, Logger::Level logLevel)
    : CompositePowerComp<Complex>(uid, name, true, true, logLevel),
      mOmegaN(mAttributes->create<Real>("Omega_nom")),
      mPref(mAttributes->create<Real>("P_ref")),
      mQref(mAttributes->create<Real>("Q_ref")),
      mVcd(mAttributes->create<Real>("Vc_d", 0)),
      mVcq(mAttributes->create<Real>("Vc_q", 0)),
      mIrcd(mAttributes->create<Real>("Irc_d", 0)),
      mIrcq(mAttributes->create<Real>("Irc_q", 0)),
      mVsrefDq(mAttributes->create<Matrix>("Vsref_dq", Matrix::Zero(2, 1))),
      mVsref(mAttributes->create<MatrixComp>("Vsref", MatrixComp::Zero(3, 1))),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")),
      mPowerctrlInputs(mAttributes->createDynamic<Matrix>("powerctrl_inputs")),
      mPowerctrlStates(mAttributes->createDynamic<Matrix>("powerctrl_states")),
      mPowerctrlOutputs(
          mAttributes->createDynamic<Matrix>("powerctrl_outputs")) {
  mPhaseType = PhaseType::ABC;
  setVirtualNodeNumber(3);
  setTerminalNumber(1);

  SPDLOG_LOGGER_INFO(mSLog, "Create {} {}", type(), name);
  **mIntfVoltage = MatrixComp::Zero(3, 1);
  **mIntfCurrent = MatrixComp::Zero(3, 1);

  mSubResistorF = DP::Ph3::Resistor::make(**mName + "_resF", mLogLevel);
  mSubResistorC = DP::Ph3::Resistor::make(**mName + "_resC", mLogLevel);
  mSubCapacitorF = DP::Ph3::Capacitor::make(**mName + "_capF", mLogLevel);
  mSubInductorF = DP::Ph3::Inductor::make(**mName + "_indF", mLogLevel);
  mSubCtrledVoltageSource =
      DP::Ph3::VoltageSource::make(**mName + "_src", mLogLevel);
  addMNASubComponent(mSubResistorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  addMNASubComponent(mSubResistorC, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, false);
  addMNASubComponent(mSubCapacitorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  addMNASubComponent(mSubInductorF, MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);
  // Pre-step of the controlled source is handled explicitly in mnaParentPreStep
  addMNASubComponent(mSubCtrledVoltageSource, MNA_SUBCOMP_TASK_ORDER::NO_TASK,
                     MNA_SUBCOMP_TASK_ORDER::TASK_BEFORE_PARENT, true);

  mPLL = Signal::PLL::make(**mName + "_PLL", mLogLevel);
  mPowerControllerVSI = Signal::PowerControllerVSI::make(
      **mName + "_PowerControllerVSI", mLogLevel);

  mPLL->mInputRef->setReference(mVcq);
  mPllOutput->setReference(mPLL->mOutputCurr);

  mPowerControllerVSI->mVc_d->setReference(mVcd);
  mPowerControllerVSI->mVc_q->setReference(mVcq);
  mPowerControllerVSI->mIrc_d->setReference(mIrcd);
  mPowerControllerVSI->mIrc_q->setReference(mIrcq);
  mPowerctrlInputs->setReference(mPowerControllerVSI->mInputCurr);
  mPowerctrlStates->setReference(mPowerControllerVSI->mStateCurr);
  mPowerctrlOutputs->setReference(mPowerControllerVSI->mOutputCurr);
}

void DP::Ph3::AvVoltageSourceInverterDQ::setParameters(Real sysOmega,
                                                       Real sysVoltNom,
                                                       Real Pref, Real Qref) {
  mParametersSet = true;
  mPowerControllerVSI->setParameters(Pref, Qref);
  **mOmegaN = sysOmega;
  mVnom = sysVoltNom;
  **mPref = Pref;
  **mQref = Qref;
}

void DP::Ph3::AvVoltageSourceInverterDQ::setControllerParameters(
    Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl, Real Ki_powerCtrl,
    Real Kp_currCtrl, Real Ki_currCtrl, Real Omega_cutoff) {
  mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
  mPLL->composeStateSpaceMatrices();
  mPowerControllerVSI->setControllerParameters(
      Kp_powerCtrl, Ki_powerCtrl, Kp_currCtrl, Ki_currCtrl, Omega_cutoff);
}

void DP::Ph3::AvVoltageSourceInverterDQ::setFilterParameters(Real Lf, Real Cf,
                                                             Real Rf,
                                                             Real Rc) {
  Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);
  mSubResistorC->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mRc));
  mSubResistorF->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mRf));
  mSubInductorF->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mLf));
  mSubCapacitorF->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(mCf));
}

void DP::Ph3::AvVoltageSourceInverterDQ::setInitialStateValues(
    Real pInit, Real qInit, Real phi_dInit, Real phi_qInit, Real gamma_dInit,
    Real gamma_qInit) {
  mPowerControllerVSI->setInitialStateValues(pInit, qInit, phi_dInit, phi_qInit,
                                             gamma_dInit, gamma_qInit);
}

Matrix DP::Ph3::AvVoltageSourceInverterDQ::parkTransformPowerInvariant(
    Real theta, const Matrix &fabc) {
  Matrix Tdq = Matrix::Zero(2, 3);
  Real k = sqrt(2. / 3.);
  Tdq << k * cos(theta), k * cos(theta - 2. * M_PI / 3.),
      k * cos(theta + 2. * M_PI / 3.), -k * sin(theta),
      -k * sin(theta - 2. * M_PI / 3.), -k * sin(theta + 2. * M_PI / 3.);
  return Tdq * fabc;
}

MatrixComp DP::Ph3::AvVoltageSourceInverterDQ::inverseParkToEnvelope(
    Real theta, Real time, const Matrix &fdq) {
  // f_k(t) = sqrt(2/3) Re{(f_d + j f_q) e^{j(theta - 2 pi k / 3)}}, so the
  // envelope at shift frequency w is that phasor turned back by w t
  const Complex dq = sqrt(2. / 3.) * Complex(fdq(0, 0), fdq(1, 0));
  MatrixComp envelope(3, 1);
  for (UInt phase = 0; phase < 3; ++phase)
    envelope(phase, 0) =
        dq * std::polar(1.0, theta - 2. * M_PI / 3. * phase -
                                 mSystemOmega * time);
  return envelope;
}

Matrix DP::Ph3::AvVoltageSourceInverterDQ::instantaneous(
    const MatrixComp &envelope, Real time) {
  const Complex carrier = std::polar(1.0, mSystemOmega * time);
  Matrix values(envelope.rows(), 1);
  for (Eigen::Index row = 0; row < envelope.rows(); ++row)
    values(row, 0) = (envelope(row, 0) * carrier).real();
  return values;
}

void DP::Ph3::AvVoltageSourceInverterDQ::initializeParentFromNodesAndTerminals(
    Real frequency) {
  mSystemOmega = 2. * PI * frequency;

  // Balanced initialization from the RMS line value of phase a, the node
  // convention of the DP::Ph3 and EMT::Ph3 components
  const Complex voltage = RMS3PH_TO_PEAK1PH * initialSingleVoltage(0);
  const Complex current =
      -std::conj(2. / 3. * Complex(**mPref, **mQref) / voltage);
  const Complex vcInit = voltage - current * Complex(mRc, 0);
  const Complex icfInit = vcInit * Complex(0., 2. * PI * frequency * mCf);
  const Complex vfInit =
      vcInit - (current - icfInit) * Complex(0., 2. * PI * frequency * mLf);
  const Complex vsInit = vfInit - (current - icfInit) * Complex(mRf, 0);
  mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vsInit);
  mVirtualNodes[1]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vfInit);
  mVirtualNodes[2]->setInitialVoltage(PEAK1PH_TO_RMS3PH * vcInit);

  **mIntfVoltage = CPS::Math::singlePhaseVariableToThreePhase(voltage);
  **mIntfCurrent = CPS::Math::singlePhaseVariableToThreePhase(current);

  mSubCtrledVoltageSource->setParameters(
      CPS::Math::singlePhaseVariableToThreePhase(vsInit));
  **mVsref = CPS::Math::singlePhaseVariableToThreePhase(vsInit);

  mSubCtrledVoltageSource->connect({SimNode::GND, mVirtualNodes[0]});
  mSubResistorF->connect({mVirtualNodes[0], mVirtualNodes[1]});
  mSubInductorF->connect({mVirtualNodes[1], mVirtualNodes[2]});
  mSubCapacitorF->connect({mVirtualNodes[2], SimNode::GND});
  mSubResistorC->connect({mVirtualNodes[2], mTerminals[0]->node()});

  for (auto subcomp : mSubComponents) {
    subcomp->initialize(mFrequencies);
    subcomp->initializeFromNodesAndTerminals(frequency);
  }

  // Controller inputs at t = 0, where the instantaneous values are the real
  // parts of the envelopes, as in EMT
  const Real theta = std::arg(voltage);
  const Matrix vcdq = parkTransformPowerInvariant(
      theta, CPS::Math::singlePhaseVariableToThreePhase(voltage).real());
  const Matrix ircdq = parkTransformPowerInvariant(
      theta, -CPS::Math::singlePhaseVariableToThreePhase(current).real());
  **mVcd = vcdq(0, 0);
  **mVcq = vcdq(1, 0);
  **mIrcd = ircdq(0, 0);
  **mIrcq = ircdq(1, 0);

  Matrix matrixStateInit = Matrix::Zero(2, 1);
  Matrix matrixOutputInit = Matrix::Zero(2, 1);
  matrixStateInit(0, 0) = theta;
  matrixOutputInit(0, 0) = theta;
  mPLL->setInitialValues(**mVcq, matrixStateInit, matrixOutputInit);
  mControlAngle = theta;
  mControlTime = 0;

  SPDLOG_LOGGER_INFO(mSLog,
                     "\n--- Initialization from powerflow ---"
                     "\nInterface voltage: {:s}"
                     "\nInterface current: {:s}"
                     "\nSource voltage: {:s}"
                     "\n--- Initialization from powerflow finished ---",
                     Logger::phasorToString(voltage),
                     Logger::phasorToString(current),
                     Logger::phasorToString(vsInit));
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaParentInitialize(
    Real omega, Real timeStep, Attribute<Matrix>::Ptr leftVector) {
  mTimeStep = timeStep;
  mSystemOmega = omega;

  // The current loop holds the filter current, so at a switching event the
  // converter is a current source and its filter state is not corrected
  mSubInductorF->mnaCorrectionSetCurrentControlled(mWithControl);

  mPowerControllerVSI->initializeStateSpaceModel(omega, timeStep, leftVector);
  mPLL->setSimulationParameters(timeStep);

  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));
}

void DP::Ph3::AvVoltageSourceInverterDQ::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddPreStepDependencies(prevStepDependencies,
                                     attributeDependencies, modifiedAttributes);
  mPowerControllerVSI->signalAddPreStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);
}

void DP::Ph3::AvVoltageSourceInverterDQ::controlPreStep(Real time,
                                                        Int timeStepCount) {
  mPLL->signalPreStep(time, timeStepCount);
  mPowerControllerVSI->signalPreStep(time, timeStepCount);
}

void DP::Ph3::AvVoltageSourceInverterDQ::addControlStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);
  mPowerControllerVSI->signalAddStepDependencies(
      prevStepDependencies, attributeDependencies, modifiedAttributes);
  attributeDependencies.push_back(mIntfCurrent);
  attributeDependencies.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mVsrefDq);
}

void DP::Ph3::AvVoltageSourceInverterDQ::controlStep(Real time,
                                                     Int timeStepCount) {
  // The values were solved for this step's instant
  Real theta = mPLL->mOutputPrev->get()(0, 0);
  const Matrix vcdq =
      parkTransformPowerInvariant(theta, instantaneous(**mIntfVoltage, time));
  const Matrix ircdq = parkTransformPowerInvariant(
      theta, -instantaneous(**mSubResistorC->mIntfCurrent, time));

  **mVcd = vcdq(0, 0);
  **mVcq = vcdq(1, 0);
  **mIrcd = ircdq(0, 0);
  **mIrcq = ircdq(1, 0);

  mPLL->signalStep(time, timeStepCount);
  mPowerControllerVSI->signalStep(time, timeStepCount);

  **mVsrefDq = mPowerControllerVSI->mOutputCurr->get();
  mControlAngle = mPLL->mOutputPrev->get()(0, 0);
  mControlTime = time;
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaParentAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  prevStepDependencies.push_back(mVsrefDq);
  prevStepDependencies.push_back(mIntfCurrent);
  prevStepDependencies.push_back(mIntfVoltage);
  attributeDependencies.push_back(mPowerControllerVSI->mOutputPrev);
  attributeDependencies.push_back(mPLL->mOutputPrev);
  modifiedAttributes.push_back(mRightVector);
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaParentPreStep(Real time,
                                                          Int timeStepCount) {
  // The dq output is held over the step, so the envelope uses the angle and
  // instant it was computed for. Until the first control step the initial
  // source voltage stays.
  if (mWithControl && timeStepCount > 0) {
    **mVsref =
        inverseParkToEnvelope(mControlAngle, mControlTime, **mVsrefDq);
    mSubCtrledVoltageSource->mVoltageRef->set(**mVsref);
  }

  std::dynamic_pointer_cast<MNAInterface>(mSubCtrledVoltageSource)
      ->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaParentAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(mIntfVoltage);
  modifiedAttributes.push_back(mIntfCurrent);
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaParentPostStep(
    Real time, Int timeStepCount, Attribute<Matrix>::Ptr &leftVector) {
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaCompUpdateCurrent(
    const Matrix &leftVector) {
  **mIntfCurrent = mSubResistorC->mIntfCurrent->get();
}

void DP::Ph3::AvVoltageSourceInverterDQ::mnaCompUpdateVoltage(
    const Matrix &leftVector) {
  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);
  for (UInt phase = 0; phase < 3; ++phase)
    (**mIntfVoltage)(phase, 0) = Math::complexFromVectorElement(
        leftVector, matrixNodeIndex(0, phase));
}
