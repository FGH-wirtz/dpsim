/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/EMT/EMT_Ph3_WECC_WT4B.h>

using namespace CPS;

namespace {
inline Real clampR(Real u, Real lo, Real hi) {
  return (u > hi) ? hi : ((u < lo) ? lo : u);
}
// Modelica DeadZone: zero inside [lo, hi], shifted outside
inline Real deadZone(Real u, Real lo, Real hi) {
  if (u > hi)
    return u - hi;
  if (u < lo)
    return u - lo;
  return 0.0;
}
} // namespace

EMT::Ph3::WECC_WT4B::WECC_WT4B(String uid, String name, Logger::Level logLevel,
                               Bool withTrafo)
    : MNASimPowerComp<Real>(uid, name, true, true, logLevel),
      mOmegaN(mAttributes->create<Real>("Omega_nom")),
      mPref(mAttributes->create<Real>("P_ref")),
      mQref(mAttributes->create<Real>("Q_ref")),
      mP(mAttributes->create<Real>("p", 0)),
      mQ(mAttributes->create<Real>("q", 0)),
      mUFilteredPu(mAttributes->create<Real>("UFilteredPu", 1)),
      mIdRefPu(mAttributes->create<Real>("idRefPu", 0)),
      mIqRefPu(mAttributes->create<Real>("iqRefPu", 0)),
      mVcd(mAttributes->create<Real>("Vc_d", 0)),
      mVcq(mAttributes->create<Real>("Vc_q", 0)),
      mIrcd(mAttributes->create<Real>("Irc_d", 0)),
      mIrcq(mAttributes->create<Real>("Irc_q", 0)),
      mFaultState(mAttributes->create<Bool>("FaultState", false)),
      mTripped(mAttributes->create<Bool>("tripped", false)),
      mUProtPu(mAttributes->create<Real>("UProtPu", 1.0)),
      mU2Pu(mAttributes->create<Complex>("U2Pu", Complex(0., 0.))),
      mI2Pu(mAttributes->create<Complex>("I2Pu", Complex(0., 0.))),
      mIsref(mAttributes->create<Matrix>("Isref", Matrix::Zero(3, 1))),
      mIs(mAttributes->createDynamic<Matrix>("Is")),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")) {
  mPhaseType = PhaseType::ABC;
  mWithConnectionTransformer = withTrafo;
  if (withTrafo) {
    setVirtualNodeNumber(1); // VN0 = LV injection node
    mConnectionTransformer = EMT::Ph3::Transformer::make(
        **mName + "_trans", **mName + "_trans", mLogLevel, false);
    mSubComponents.push_back(mConnectionTransformer);
  } else {
    setVirtualNodeNumber(0); // inject directly at terminal
  }
  setTerminalNumber(1);

  **mIntfVoltage = Matrix::Zero(3, 1);
  **mIntfCurrent = Matrix::Zero(3, 1);

  mSubCtrledCurrentSource =
      EMT::Ph3::ControlledCurrentSource::make(**mName + "_src", mLogLevel);
  mSubShuntResistor =
      EMT::Ph3::Resistor::make(**mName + "_resShunt", mLogLevel);
  mSubComponents.push_back(mSubCtrledCurrentSource);
  mSubComponents.push_back(mSubShuntResistor);

  mPLL = Signal::PLL_Enhanced::make(**mName + "_PLL", mLogLevel);

  mIs->setReference(mSubCtrledCurrentSource->mIntfCurrent);
  mPLL->mInputRef->setReference(mVcq);
  mPLL->mInputRefD->setReference(mVcd);
  mPllOutput->setReference(mPLL->mOutputCurr);

  mSLog->info("Create {} {}", type(), name);
}

void EMT::Ph3::WECC_WT4B::setParameters(Real sysOmega, Real sysVoltNom,
                                        Real Pref, Real Qref, Real Sn,
                                        Real cosPhi) {
  mParametersSet = true;
  **mOmegaN = sysOmega;
  mVnom = sysVoltNom;
  mVnomHV = sysVoltNom;
  **mPref = Pref;
  **mQref = Qref;
  Real Smin = sqrt(Pref * Pref + Qref * Qref);
  mSn = (Sn < Smin) ? Smin : Sn;
  mSNom = (mSn > 0) ? mSn : 1.0;

  mSLog->info("General Parameters: Vnom={} [V] Omega={} [1/s]", sysVoltNom,
              sysOmega);
  mSLog->info("Pref={} [W] Qref={} [VAr] SNom={} [VA]", Pref, Qref, mSNom);
}

void EMT::Ph3::WECC_WT4B::setControllerParameters(Real Kp_pll, Real Ki_pll,
                                                  Real Omega_cutoff) {
  mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
  mPLL->composeStateSpaceMatrices();
  mSLog->info("PLL: Kp={} Ki={} Omega={}", Kp_pll, Ki_pll, Omega_cutoff);
}

void EMT::Ph3::WECC_WT4B::setTransformerParameters(
    Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
    Real ratioPhase, Real resistance, Real inductance, Real omega) {
  Base::AvVoltageSourceInverterDQ::setTransformerParameters(
      nomVoltageEnd1, nomVoltageEnd2, ratedPower, ratioAbs, ratioPhase,
      resistance, inductance);
  if (mWithConnectionTransformer)
    mConnectionTransformer->setParameters(
        mTransformerNominalVoltagePrimary, mTransformerNominalVoltageSecondary,
        mTransformerRatedPower, mTransformerRatioAbs, mTransformerRatioPhase,
        CPS::Math::singlePhaseParameterToThreePhase(mTransformerResistance),
        CPS::Math::singlePhaseParameterToThreePhase(mTransformerInductance));
}

void EMT::Ph3::WECC_WT4B::setFilterParameters(Real Lf, Real Cf, Real Rf,
                                              Real Rc) {
  // direct current injection: no series filter
  Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);
  mCf = 0;
}

void EMT::Ph3::WECC_WT4B::initializeFromNodesAndTerminals(Real frequency) {
  if (mWithConnectionTransformer)
    mVnomHV = mVnom * mTransformerRatioAbs;

  // power-flow terminal (HV) phasors, LL-RMS -> phase peak (sqrt(2/3))
  MatrixComp Vhv = MatrixComp::Zero(3, 1);
  MatrixComp Ihv = MatrixComp::Zero(3, 1);
  Complex initVoltage = (std::abs(initialSingleVoltage(0)) > 0)
                            ? sqrt(2. / 3.) * initialSingleVoltage(0)
                            : sqrt(2. / 3.) * Complex(mVnomHV, 0);
  Vhv(0, 0) = initVoltage;
  Vhv(1, 0) = Vhv(0, 0) * SHIFT_TO_PHASE_B;
  Vhv(2, 0) = Vhv(0, 0) * SHIFT_TO_PHASE_C;
  Ihv(0, 0) = -std::conj((2. / 3.) * (Complex(**mPref, **mQref) / Vhv(0, 0)));
  Ihv(1, 0) = Ihv(0, 0) * SHIFT_TO_PHASE_B;
  Ihv(2, 0) = Ihv(0, 0) * SHIFT_TO_PHASE_C;

  // injection (LV) side quantities
  MatrixComp Vlv = MatrixComp::Zero(3, 1);
  MatrixComp Ilv = MatrixComp::Zero(3, 1);
  SimNode::Ptr injNode;
  if (mWithConnectionTransformer) {
    Vlv = (Vhv - Complex(mTransformerResistance,
                         mTransformerInductance * **mOmegaN) *
                     Ihv) /
          Complex(mTransformerRatioAbs, mTransformerRatioPhase);
    Ilv = Ihv * Complex(mTransformerRatioAbs, mTransformerRatioPhase);
    mVirtualNodes[0]->setInitialVoltage(PEAK1PH_TO_RMS3PH * Vlv);
    mConnectionTransformer->connect({mTerminals[0]->node(), mVirtualNodes[0]});
    injNode = mVirtualNodes[0];
  } else {
    Vlv = Vhv;
    Ilv = Ihv;
    injNode = mTerminals[0]->node();
  }

  **mIntfVoltage = Vhv.real();
  **mIntfCurrent = Ihv.real();

  mSubCtrledCurrentSource->connect({SimNode::GND, injNode});
  mSubShuntResistor->connect({injNode, SimNode::GND});
  // shunt on the node impedance base, independent of the voltage level
  Real zBase = (mVnomHV * mVnomHV) / mSNom;
  Real rShuntFactor = mShuntResistanceFactor;
  if (const char *rs = std::getenv("DPSIM_WT4B_RSHUNT"))
    try { rShuntFactor = std::stod(rs); } catch (...) {}
  Real rShunt = rShuntFactor * zBase;
  mSubShuntResistor->setParameters(
      CPS::Math::singlePhaseParameterToThreePhase(rShunt));

  Real theta0 = std::arg(injNode->initialSingleVoltage());
  mTheta0 = theta0;
  mThetaN = theta0;
  mThetaOff = 0;
  if (const char *tp = std::getenv("DPSIM_WT4B_TPLL"))
    try { mTPll = std::stod(tp); } catch (...) {}
  if (std::getenv("DPSIM_WT4B_FROZEN_FRAME"))
    mTPll = 0.0;
  Matrix vcdq0 = parkTransform(theta0, Vlv.real());
  Matrix idq0 = parkTransform(theta0, Ilv.real());
  // dq bases on the nominal voltage (Dynawo); Park dq is on the LL-RMS scale
  mUdqNom = mVnom;
  mIBaseDq = mSNom / mUdqNom;
  Real UPu0 = std::hypot(vcdq0(0, 0), vcdq0(1, 0)) / mUdqNom;
  if (UPu0 < 1e-6)
    UPu0 = 1.0;
  // generator convention; the terminal current is receptor-signed
  mId0Pu = -idq0(0, 0) / mIBaseDq;
  mIq0Pu = -idq0(1, 0) / mIBaseDq;

  **mVcd = vcdq0(0, 0);
  **mVcq = vcdq0(1, 0);
  **mIrcd = idq0(0, 0);
  **mIrcq = idq0(1, 0);
  **mIdRefPu = mId0Pu;
  **mIqRefPu = mIq0Pu;
  **mUFilteredPu = UPu0;
  // protection starts at the pre-fault voltage
  mVrt.initialize(UPu0);
  for (auto &h : mUHalf)
    h.initialize(UPu0 * UPu0);
  **mUProtPu = UPu0;
  mFrt.setParameters(mP_.DroopMode, mP_.Kqv, mP_.FrtRamp, mP_.UMinPu, mP_.UMaxPu);
  mFrt.initialize(UPu0);
  **mTripped = false;
  **mIsref = Ilv.real();

  mSubCtrledCurrentSource->setParameters(**mIsref);

  for (auto subcomp : mSubComponents) {
    subcomp->initialize(mFrequencies);
    subcomp->initializeFromNodesAndTerminals(frequency);
  }

  // seed control blocks at the operating point (no transient at t=0)
  mUFilter.setParameters(1.0, mP_.tRv, -9999, 9999);
  mUFilter.initialize(UPu0);
  mUFilterGC.setParameters(1.0, mP_.tFilterGC, -9999, 9999);
  mUFilterGC.initialize(UPu0);
  mPmeasFilter.setParameters(1.0, mP_.tP, -9999, 9999);
  // PeFilt tracks the RECEPTOR-signed Pinj (negative when generating)
  mPmeasFilter.initialize(
      (vcdq0(0, 0) * idq0(0, 0) + vcdq0(1, 0) * idq0(1, 0)) / mSNom);
  mIqFilter.setParameters(1.0, mP_.Tiq, -9999, 9999);
  mIqFilter.initialize(mIq0Pu);
  mPfaRef = std::atan2(**mQref, **mPref);
  mPSlew.setParameters(mP_.DPMin, mP_.DPMax, -9999, 9999);
  mPSlew.initialize(mId0Pu * UPu0); // Pord path carries P (=id*U), not id
  mPordFilter.setParameters(1.0, mP_.tPord, -9999, 9999);
  mPordFilter.initialize(mId0Pu * UPu0);
  // Q PI outputs a voltage command; t=0 output UPu0
  mQPid.setParameters(mP_.Kqp, (mP_.Kqi != 0) ? mP_.Kqp / mP_.Kqi : 1e6, 1.0,
                      mP_.Ni, false, 1.0);
  mQPid.initialize((mP_.Kqp != 0) ? UPu0 / mP_.Kqp : 0.0);
  // V PI outputs the reactive current order; t=0 output Iq0Pu
  mVPid.setParameters(mP_.Kvp, (mP_.Kvi != 0) ? mP_.Kvp / mP_.Kvi : 1e6, 1.0,
                      mP_.Ni, false, 1.0);
  mVPid.initialize((mP_.Kvp != 0) ? -mIq0Pu / mP_.Kvp : 0.0);
  mIdRegc.setParameters(1.0, mP_.tG, -9999, mP_.Rrpwr);
  mIdRegc.initialize(mId0Pu);
  mPRec.setParameters(mP_.TPRec);
  mPRec.initialize(mId0Pu);
  mIqRegc.setParameters(1.0, mP_.tG, mP_.IqrMinPu, mP_.IqrMaxPu);
  mIqRegc.initialize(mIq0Pu);

  // PLL init
  Matrix stateInit = Matrix::Zero(2, 1);
  Matrix outputInit = Matrix::Zero(2, 1);
  stateInit(0, 0) = theta0;
  outputInit(0, 0) = theta0;
  mPLL->setInitialValues(**mVcq, stateInit, outputInit);

  mSLog->info("\n--- WT4B init --- UdqNom={} IBaseDq={} Id0Pu={} Iq0Pu={}",
              mUdqNom, mIBaseDq, mId0Pu, mIq0Pu);
}

void EMT::Ph3::WECC_WT4B::mnaCompInitialize(Real omega, Real timeStep,
                                            Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;

  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaInitialize(omega, timeStep, leftVector);

  mPLL->setSimulationParameters(timeStep);

  mRightVectorStamps.push_back(
      &mSubCtrledCurrentSource->attributeTyped<Matrix>("right_vector")->get());
  if (mWithConnectionTransformer)
    mRightVectorStamps.push_back(
        &mConnectionTransformer->attributeTyped<Matrix>("right_vector")->get());

  mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));

  **mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void EMT::Ph3::WECC_WT4B::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
}

void EMT::Ph3::WECC_WT4B::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  rightVector.setZero();
  for (auto stamp : mRightVectorStamps)
    rightVector += *stamp;
}

void EMT::Ph3::WECC_WT4B::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies,
                                     modifiedAttributes);
}

void EMT::Ph3::WECC_WT4B::controlPreStep(Real time, Int timeStepCount) {
  mPLL->signalPreStep(time, timeStepCount);
}

void EMT::Ph3::WECC_WT4B::addControlStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);
  attributeDependencies.push_back(attribute("i_intf"));
  attributeDependencies.push_back(attribute("v_intf"));
  modifiedAttributes.push_back(attribute("Isref"));
}

Matrix EMT::Ph3::WECC_WT4B::parkTransform(Real theta, const Matrix &fabc) {
  return getParkTransformMatrix(theta) * fabc;
}

Matrix EMT::Ph3::WECC_WT4B::getParkTransformMatrix(Real theta) {
  Matrix Tdq = Matrix::Zero(2, 3);
  Real k = sqrt(2. / 3.);
  Tdq << k * cos(theta), k * cos(theta - (2. * M_PI) / 3.),
      k * cos(theta + (2. * M_PI) / 3.), k * -sin(theta),
      k * -sin(theta - (2. * M_PI) / 3.), k * -sin(theta + (2. * M_PI) / 3.);
  return Tdq;
}

Matrix EMT::Ph3::WECC_WT4B::inverseParkTransform(Real theta, const Matrix &fdq) {
  return getInverseParkTransformMatrix(theta) * fdq;
}

Matrix EMT::Ph3::WECC_WT4B::getInverseParkTransformMatrix(Real theta) {
  Matrix Tabc = Matrix::Zero(3, 2);
  Real k = sqrt(2. / 3.);
  Tabc << k * cos(theta), -k * sin(theta), k * cos(theta - 2. * M_PI / 3.),
      -k * sin(theta - 2. * M_PI / 3.), k * cos(theta + 2. * M_PI / 3.),
      -k * sin(theta + 2. * M_PI / 3.);
  return Tabc;
}

void EMT::Ph3::WECC_WT4B::controlStep(Real time, Int timeStepCount) {
  Real dt = mTimeStep;
  // dq frame: theta0 + omegaN*t plus the first-order lag offset
  Real theta = mTheta0 + **mOmegaN * time + mThetaOff;

  Matrix Vabc =
      mWithConnectionTransformer ? **mVirtualNodes[0]->mVoltage : **mIntfVoltage;
  if (mTPll > 0) {
    Matrix vdq0 = parkTransform(theta, Vabc);
    if (std::hypot(vdq0(0, 0), vdq0(1, 0)) > 1e-6 * mUdqNom) {
      mThetaOff += (dt / mTPll) * std::atan2(vdq0(1, 0), vdq0(0, 0));
      theta = mTheta0 + **mOmegaN * time + mThetaOff;
    }
  }
  Matrix vcdq = parkTransform(theta, Vabc);
  **mVcd = vcdq(0, 0);
  **mVcq = vcdq(1, 0);

  // injected current (previous command) for power feedback
  Real idPrevPu = **mIdRefPu;
  Real iqPrevPu = **mIqRefPu;
  // injected (receptor) current = -(generator command)
  **mIrcd = -idPrevPu * mIBaseDq;
  **mIrcq = -iqPrevPu * mIBaseDq;

  mPLL->signalStep(time, timeStepCount);

  Real Umag = std::hypot(**mVcd, **mVcq);
  Real UPu = Umag / mUdqNom;
  Real Pinj = (**mVcd * **mIrcd + **mVcq * **mIrcq) / mSNom;
  Real Qinj = (**mVcq * **mIrcd - **mVcd * **mIrcq) / mSNom;
  **mP = Pinj * mSNom;
  **mQ = Qinj * mSNom;

  Real UFilt = mUFilter.step(UPu, dt, false);
  **mUFilteredPu = UFilt;
  Real Uden = std::max(UFilt, 0.01);
  // FRT detection on the tRv-filtered voltage (REEC Vt_filt); the filter
  // also prevents FRT chatter through the one-step V-I loop
  bool frt = mFrt.step(UFilt, dt);
  // envelope in dq units: the park transform at the nominal rotation w t
  Matrix venv = parkTransform(**mOmegaN * time, Vabc);
  Complex i2 = mNegSeq.step(Complex(venv(0, 0), venv(1, 0)), **mOmegaN, time,
                            dt, mUdqNom, frt || mP_.DroopMode == 2,
                            mP_.FrtRamp);
  **mFaultState = frt;

  // ---- active power channel ----
  // no drive train in WT4B: generator speed 1 pu
  Real omegaGen = 1.0;
  Real PrefPu = (**mPref / mSNom) * (mP_.PFlag ? omegaGen : 1.0);
  Real Pslew = mPSlew.step(PrefPu, dt, false);
  Real Pord = mPordFilter.step(Pslew, dt, frt); // freeze ramp during FRT
  Pord = clampR(Pord, mP_.PMinPu, mP_.PMaxPu);
  Real idCmd = Pord / Uden;

  // ---- reactive control (PfFlag / QFlag / VFlag) ----
  bool freezePI = frt;
  Real PeFilt = mPmeasFilter.step(Pinj, dt, false); // tP-filtered P (pu)
  // QrefSig is generation-signed, PeFilt receptor-signed (hence the minus)
  Real QrefSig =
      mP_.PfFlag ? -PeFilt * std::tan(mPfaRef) : (**mQref / mSNom);
  // fixed voltage reference; also feeds the droop deadband
  Real VRefConst = (mP_.VRef0Pu < 0.5) ? 1.0 : mP_.VRef0Pu;

  // outer Q PI: output is a voltage command in [VMinPu, VMaxPu]
  Real QrefLim = clampR(QrefSig, mP_.QMinPu, mP_.QMaxPu);
  // Qinj is receptor-signed, so the generated Q tracks +QrefLim
  Real Vqcmd = mQPid.step(QrefLim, -Qinj, dt, freezePI, mP_.VMinPu, mP_.VMaxPu);

  // inner V PI: reference is the Q-PI voltage (VFlag) or VRefConst
  Real vInnerRef = clampR(mP_.VFlag ? Vqcmd : VRefConst, mP_.VMinPu, mP_.VMaxPu);
  Real iqVloop =
      -mVPid.step(vInnerRef, UFilt, dt, freezePI, -mP_.IMaxPu, mP_.IMaxPu);

  // direct constant-Q branch, unclamped reference; iq < 0 injects Q
  Real iqDirect = mIqFilter.step(-QrefSig / Uden, dt, freezePI);

  // QFlag selects the voltage-regulator branch vs the direct constant-Q branch
  Real iqMain = mP_.QFlag ? iqVloop : iqDirect;

  // supplemental reactive current from the voltage deadband droop
  Real db = deadZone(VRefConst - UFilt, mP_.Dbd1, mP_.Dbd2);
  // q-axis sign: iq > 0 absorbs, iq < 0 injects; REEC droop and limits negated
  Real dIqSupport =
      (mP_.DroopMode == 0) ? mP_.Kqv * db : mFrt.supportCurrent(UFilt);
  Real iqInj = clampR(-dIqSupport, -mP_.Iqh1Pu, -mP_.Iql1Pu);
  Real iqCmd = iqMain + iqInj;

  // ---- current limit (circle, IMaxPu) ----
  Real Imax = mP_.IMaxPu;
  if (mP_.PPriority && !frt) {
    idCmd = clampR(idCmd, -Imax, Imax);
    Real iqLim = sqrt(std::max(0.0, Imax * Imax - idCmd * idCmd));
    iqCmd = clampR(iqCmd, -iqLim, iqLim);
  } else {
    iqCmd = clampR(iqCmd, -Imax, Imax);
    Real idLim = sqrt(std::max(0.0, Imax * Imax - iqCmd * iqCmd));
    idCmd = clampR(idCmd, -idLim, idLim);
  }

  // ---- REGC_A conditioning ----
  idCmd = mPRec.step(frt, idCmd, dt);
  Real idRamp = mIdRegc.step(idCmd, dt, false); // Rrpwr-limited rise
  Real Ugc = mUFilterGC.step(UPu, dt, false);
  Real lvpl = 9999.0;
  if (mP_.Lvplsw) {
    Real mlvpl = mP_.lvpl1 / (mP_.brkpt - mP_.zerox);
    lvpl = (Ugc < mP_.zerox)
               ? 0.0
               : ((Ugc > mP_.brkpt) ? 9999.0 : mlvpl * (Ugc - mP_.zerox));
  }
  Real idRef = clampR(idRamp, -9999.0, lvpl);

  Real iqRamp = mIqRegc.step(iqCmd, dt, false);
  Real iqhv = (UFilt > mP_.Volim) ? mP_.Khv * (UFilt - mP_.Volim) : 0.0;
  // REGC HV clamp, negated for the q-axis sign (towards absorption)
  Real iqRef = std::min(iqRamp + iqhv, -mP_.Iolim);

  **mIdRefPu = idRef;
  **mIqRefPu = iqRef;

  // ---- direct current injection (InjectorIDQ) ----
  // negative sequence: phase k carries Re{I2 a^k e^jwt}; above IMax the
  // active current goes first during the release
  const Complex rot = std::polar(1.0, theta - **mOmegaN * time);
  const auto lim = Signal::WindGen::NegativeSequenceSupport::phaseLimit(
      idRef * rot, Complex(0., iqRef) * rot, i2, mP_.IMaxPu,
      mNegSeq.released());
  idRef *= mNegSeq.activeFactor(lim.first, dt, mP_.FrtRamp);
  iqRef *= lim.second;
  i2 *= lim.second;
  **mIdRefPu = idRef;
  **mIqRefPu = iqRef;
  **mU2Pu = mNegSeq.mU2;
  **mI2Pu = i2;
  Matrix idqRef = Matrix::Zero(2, 1);
  idqRef(0, 0) = idRef * mIBaseDq;
  idqRef(1, 0) = iqRef * mIBaseDq;
  // generator dq command -> receptor terminal current (leading minus)
  **mIsref = -inverseParkTransform(theta, idqRef);
  for (int k = 0; k < 3; ++k)
    (**mIsref)(k, 0) -=
        std::sqrt(2. / 3.) *
        std::real(i2 * mIBaseDq *
                  std::polar(1.0, **mOmegaN * time + 2. * M_PI * k / 3.));

  // VDE HVRT/non-HVRT protection: mVrt.gate() scales the injection (0 when
  // disconnected, ramps to 1 on reconnection); a scheduled disconnect gives 0.
  // Sampled phase and line-line voltages
  Real vPeak = Vabc.cwiseAbs().maxCoeff() / (mUdqNom * sqrt(2. / 3.));
  Real uHalf = 0.0;
  for (int k = 0; k < 3; ++k) {
    Real vll = (Vabc(k, 0) - Vabc((k + 1) % 3, 0)) / mUdqNom;
    uHalf = std::max(uHalf, mUHalf[k].step(vll * vll, dt, **mOmegaN));
  }
  Real uProt;
  bool vtrip = mVrt.stepMeasured(UFilt, vPeak, uHalf, false, **mOmegaN, dt, uProt);
  **mUProtPu = uProt;
  bool alm = (mTDisconnect > 0 && time >= mTDisconnect);
  Real g = alm ? 0.0 : mVrt.gate();
  **mTripped = vtrip || alm;
  if (g < 1.0) {
    // reported P/Q and current commands follow the scaled injection
    **mIsref = **mIsref * g;
    **mP = **mP * g; **mQ = **mQ * g;
    **mIdRefPu = **mIdRefPu * g; **mIqRefPu = **mIqRefPu * g;
    **mI2Pu = **mI2Pu * g;
  }

  mThetaN += dt * **mOmegaN;
}

void EMT::Ph3::WECC_WT4B::mnaCompAddPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaAddPreStepDependencies(prevStepDependencies,
                                            attributeDependencies,
                                            modifiedAttributes);
  prevStepDependencies.push_back(attribute("Isref"));
  prevStepDependencies.push_back(attribute("i_intf"));
  prevStepDependencies.push_back(attribute("v_intf"));
  modifiedAttributes.push_back(attribute("right_vector"));
}

void EMT::Ph3::WECC_WT4B::mnaCompPreStep(Real time, Int timeStepCount) {
  // Norton compensation: the shunt current of the previous step is added to
  // the source, so the net injection stays mIsref and the shunt does not load
  // a weak bus; the one-step delay leaves only a -G*dv damping term.
  **mSubCtrledCurrentSource->mCurrentRef =
      **mIsref + mSubShuntResistor->attributeTyped<Matrix>("i_intf")->get();
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void EMT::Ph3::WECC_WT4B::mnaCompAddPostStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes,
    Attribute<Matrix>::Ptr &leftVector) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaAddPostStepDependencies(prevStepDependencies,
                                             attributeDependencies,
                                             modifiedAttributes, leftVector);
  attributeDependencies.push_back(leftVector);
  modifiedAttributes.push_back(attribute("v_intf"));
  modifiedAttributes.push_back(attribute("i_intf"));
}

void EMT::Ph3::WECC_WT4B::mnaCompPostStep(Real time, Int timeStepCount,
                                          Attribute<Matrix>::Ptr &leftVector) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void EMT::Ph3::WECC_WT4B::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mWithConnectionTransformer)
    **mIntfCurrent = mConnectionTransformer->attributeTyped<Matrix>("i_intf")->get();
  else
    **mIntfCurrent =
        mSubCtrledCurrentSource->attributeTyped<Matrix>("i_intf")->get();
}

void EMT::Ph3::WECC_WT4B::mnaCompUpdateVoltage(const Matrix &leftVector) {
  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);
  (**mIntfVoltage)(0, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 0));
  (**mIntfVoltage)(1, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 1));
  (**mIntfVoltage)(2, 0) =
      Math::realFromVectorElement(leftVector, matrixNodeIndex(0, 2));
}
