/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/DP/DP_Ph1_WECC_WT4B.h>
#include <cstdlib>

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

DP::Ph1::WECC_WT4B::WECC_WT4B(String uid, String name, Logger::Level logLevel,
                              Bool withTrafo)
    : MNASimPowerComp<Complex>(uid, name, true, true, logLevel),
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
      mIsref(mAttributes->create<Complex>("Isref", Complex(0, 0))),
      mIs(mAttributes->createDynamic<MatrixComp>("Is")),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")) {
  mPhaseType = PhaseType::Single;
  mWithConnectionTransformer = withTrafo;
  if (withTrafo) {
    setVirtualNodeNumber(1); // VN0 = LV injection node
    mConnectionTransformer = DP::Ph1::Transformer::make(
        **mName + "_trans", **mName + "_trans", mLogLevel, false);
    mSubComponents.push_back(mConnectionTransformer);
  } else {
    setVirtualNodeNumber(0); // inject directly at terminal
  }
  setTerminalNumber(1);

  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  mSubCtrledCurrentSource =
      DP::Ph1::CurrentSource::make(**mName + "_src", mLogLevel);
  mSubShuntResistor = DP::Ph1::Resistor::make(**mName + "_resShunt", mLogLevel);
  mSubComponents.push_back(mSubCtrledCurrentSource);
  mSubComponents.push_back(mSubShuntResistor);

  mPLL = Signal::PLL_Enhanced::make(**mName + "_PLL", mLogLevel);

  mIs->setReference(mSubCtrledCurrentSource->mIntfCurrent);
  mPLL->mInputRef->setReference(mVcq);
  mPLL->mInputRefD->setReference(mVcd);
  mPllOutput->setReference(mPLL->mOutputCurr);

  mSLog->info("Create {} {}", type(), name);
}

void DP::Ph1::WECC_WT4B::setParameters(Real sysOmega, Real sysVoltNom,
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

void DP::Ph1::WECC_WT4B::setControllerParameters(Real Kp_pll, Real Ki_pll,
                                                 Real Omega_cutoff) {
  mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
  mPLL->composeStateSpaceMatrices();
  mSLog->info("PLL: Kp={} Ki={} Omega={}", Kp_pll, Ki_pll, Omega_cutoff);
}

void DP::Ph1::WECC_WT4B::setTransformerParameters(
    Real nomVoltageEnd1, Real nomVoltageEnd2, Real ratedPower, Real ratioAbs,
    Real ratioPhase, Real resistance, Real inductance, Real omega) {
  Base::AvVoltageSourceInverterDQ::setTransformerParameters(
      nomVoltageEnd1, nomVoltageEnd2, ratedPower, ratioAbs, ratioPhase,
      resistance, inductance);
  if (mWithConnectionTransformer)
    mConnectionTransformer->setParameters(
        mTransformerNominalVoltagePrimary, mTransformerNominalVoltageSecondary,
        mTransformerRatedPower, mTransformerRatioAbs, mTransformerRatioPhase,
        mTransformerResistance, mTransformerInductance);
}

void DP::Ph1::WECC_WT4B::setFilterParameters(Real Lf, Real Cf, Real Rf,
                                             Real Rc) {
  // direct current injection: no series filter
  Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);
  mCf = 0;
}

Complex DP::Ph1::WECC_WT4B::toDQ(Complex phasor) {
  // node phasor rotated by the tracked terminal angle; returns vd + j vq
  return Math::rotatingFrame2to1(phasor, mThetaT, 0.);
}

Complex DP::Ph1::WECC_WT4B::fromDQ(Real d, Real q) {
  // generator-convention dq command -> current phasor (d + j q) e^(j thetaT)
  return Math::rotatingFrame2to1(Complex(d, q), 0., mThetaT);
}

void DP::Ph1::WECC_WT4B::initializeFromNodesAndTerminals(Real frequency) {
  if (mWithConnectionTransformer)
    mVnomHV = mVnom * mTransformerRatioAbs;

  // complex single-phase terminal (HV) quantities from power flow
  Complex V0 = (std::abs(initialSingleVoltage(0)) > 0)
                   ? initialSingleVoltage(0)
                   : Complex(mVnomHV, 0);
  // receptor terminal current (DP Ph1 convention: S = V * conj(I))
  Complex I0 = -std::conj(Complex(**mPref, **mQref) / V0);

  // injection-side (LV) quantities
  Complex Vlv, Ilv;
  SimNode::Ptr injNode;
  if (mWithConnectionTransformer) {
    Vlv = (V0 - Complex(mTransformerResistance,
                        mTransformerInductance * **mOmegaN) *
                    I0) /
          Complex(mTransformerRatioAbs, mTransformerRatioPhase);
    Ilv = I0 * Complex(mTransformerRatioAbs, mTransformerRatioPhase);
    mVirtualNodes[0]->setInitialVoltage(Vlv);
    mConnectionTransformer->connect({mTerminals[0]->node(), mVirtualNodes[0]});
    injNode = mVirtualNodes[0];
  } else {
    Vlv = V0;
    Ilv = I0;
    injNode = mTerminals[0]->node();
  }

  (**mIntfVoltage)(0, 0) = V0;
  (**mIntfCurrent)(0, 0) = I0;

  // DP::Ph1::CurrentSource injects +I_ref into node2, opposite to
  // EMT::Ph3::ControlledCurrentSource; {injNode, GND} gives -I_ref into injNode
  mSubCtrledCurrentSource->connect({injNode, SimNode::GND});
  mSubShuntResistor->connect({injNode, SimNode::GND});
  // damping shunt on the node impedance base, Norton-compensated in
  // mnaCompPreStep so it does not shift the operating point
  Real zBase = (mVnomHV * mVnomHV) / mSNom;
  Real rShuntFactor = mShuntResistanceFactor;
  if (const char *rs = std::getenv("DPSIM_WT4B_RSHUNT"))
    try { rShuntFactor = std::stod(rs); } catch (...) {}
  Real rShunt = rShuntFactor * zBase;
  mSubShuntResistor->setParameters(rShunt);

  // dq bases on the nominal voltage (Dynawo); UPu0 is an initial value
  mTheta0 = std::arg(injNode->initialSingleVoltage());
  mThetaN = mTheta0;
  mThetaT = mTheta0;
  if (const char *tp = std::getenv("DPSIM_WT4B_TPLL"))
    try { mTPll = std::stod(tp); } catch (...) {}
  if (std::getenv("DPSIM_WT4B_FROZEN_FRAME"))
    mTPll = 0.0;
  Complex vcdq0 = toDQ(Vlv);
  Complex idq0 = toDQ(Ilv);
  mUdqNom = mVnom; // LV nominal, DP LL-RMS scale
  mIBaseDq = mSNom / mUdqNom;
  Real UPu0 = std::abs(vcdq0) / mUdqNom;
  if (UPu0 < 1e-6)
    UPu0 = 1.0;
  // generator convention; the terminal current is receptor-signed
  mId0Pu = -idq0.real() / mIBaseDq;
  mIq0Pu = -idq0.imag() / mIBaseDq;

  **mVcd = vcdq0.real();
  **mVcq = vcdq0.imag();
  **mIrcd = idq0.real();
  **mIrcq = idq0.imag();
  **mIdRefPu = mId0Pu;
  **mIqRefPu = mIq0Pu;
  **mUFilteredPu = UPu0;
  // protection starts at the pre-fault voltage
  mVrt.initialize(UPu0);
  for (auto &h : mUHalf)
    h.initialize(UPu0 * UPu0);
  mVNodeValid = false;
  for (int k = 0; k < 3; ++k)
    mCompV[k] = mCompVPrev[k] = 0;
  **mUProtPu = UPu0;
  mFrt.setParameters(mP_.DroopMode, mP_.Kqv, mP_.FrtRamp, mP_.UMinPu, mP_.UMaxPu);
  mFrt.initialize(UPu0);
  **mTripped = false;
  // seed the source with the power-flow receptor current at the injection node
  **mIsref = Ilv;

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
      (vcdq0.real() * idq0.real() + vcdq0.imag() * idq0.imag()) / mSNom);
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

  // PLL init (open-loop; stepped only for logging)
  Matrix stateInit = Matrix::Zero(2, 1);
  Matrix outputInit = Matrix::Zero(2, 1);
  stateInit(0, 0) = mTheta0;
  outputInit(0, 0) = mTheta0;
  mPLL->setInitialValues(**mVcq, stateInit, outputInit);

  mSLog->info("\n--- WT4B (DP Ph1) init --- UdqNom={} IBaseDq={} Id0Pu={} "
              "Iq0Pu={} theta0={}",
              mUdqNom, mIBaseDq, mId0Pu, mIq0Pu, mTheta0);
}

void DP::Ph1::WECC_WT4B::mnaCompInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;

  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaInitialize(omega, timeStep, leftVector);

  mPLL->setSimulationParameters(timeStep);

  mRightVectorStamps.push_back(&**mSubCtrledCurrentSource->mRightVector);
  if (mWithConnectionTransformer)
    mRightVectorStamps.push_back(&**mConnectionTransformer->mRightVector);

  mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));

  **mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void DP::Ph1::WECC_WT4B::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
}

void DP::Ph1::WECC_WT4B::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  rightVector.setZero();
  for (auto stamp : mRightVectorStamps)
    rightVector += *stamp;
}

void DP::Ph1::WECC_WT4B::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies,
                                     modifiedAttributes);
}

void DP::Ph1::WECC_WT4B::controlPreStep(Real time, Int timeStepCount) {
  mPLL->signalPreStep(time, timeStepCount);
}

void DP::Ph1::WECC_WT4B::addControlStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);
  attributeDependencies.push_back(attribute("i_intf"));
  attributeDependencies.push_back(attribute("v_intf"));
  if (!mCompCorrection.isNull()) {
    attributeDependencies.push_back(mCompCorrection);
    attributeDependencies.push_back(mCompActive);
    if (!mCompHandedOver.isNull())
      attributeDependencies.push_back(mCompHandedOver);
  }
  modifiedAttributes.push_back(attribute("Isref"));
  // control outputs of this step
  for (auto &attr : {mP, mQ, mUFilteredPu, mIdRefPu, mIqRefPu, mUProtPu})
    modifiedAttributes.push_back(attr);
  modifiedAttributes.push_back(mTripped);
}

Complex DP::Ph1::WECC_WT4B::phaseEnvelope(int k, Real lag) const {
  // phase k envelope (phase-a frame), interpolated back by lag <= dt
  Real a = std::min(lag / mTimeStep, 1.0);
  return (1. - a) * (mVNode + mCompV[k]) + a * (mVNodePrev + mCompVPrev[k]);
}

Complex DP::Ph1::WECC_WT4B::peakEnvelope(int k, Real lag) const {
  Real a = std::min(lag / mTimeStep, 1.0);
  return (1. - a) * mPeakEnv[k] + a * mPeakEnvPrev[k];
}

void DP::Ph1::WECC_WT4B::controlStep(Real time, Int timeStepCount) {
  Real dt = mTimeStep;

  // injection-node voltage into the dq frame; the first-order angle lag
  // avoids the one-step V-I loop a direct arg(V) would close
  Complex Vnode = mWithConnectionTransformer
                      ? mVirtualNodes[0]->mVoltage->get()(0, 0)
                      : (**mIntfVoltage)(0, 0);
  Complex envelope = Vnode;
  if (!mCompCorrection.isNull()) {
    mVNodePrev = mVNodeValid ? mVNode : Vnode;
    mVNode = Vnode;
    mVNodeValid = true;
    Bool active = **mCompActive;
    for (int k = 0; k < 3; ++k) {
      mCompVPrev[k] = mCompV[k];
      mCompV[k] = active ? (**mCompCorrection)(k, 0) : Complex(0, 0);
    }
    // space vector of the corrected phases, positive and negative sequence
    Complex sv = Vnode;
    Complex v1(0, 0);
    if (active) {
      Complex v2(0, 0);
      for (int k = 0; k < 3; ++k) {
        Complex u = phaseEnvelope(k, 0.0);
        v1 += u / 3.;
        v2 += std::conj(u) / 3. *
              std::polar(1.0, 4. * PI * k / 3. - 2. * **mOmegaN * time);
      }
      sv = v1 + v2;
      envelope = sv;
    }
    const auto delay = static_cast<std::size_t>(
        std::max(1.0, std::round(0.5 * PI / **mOmegaN / dt)));
    mSpaceVectorHistory.push_back(sv);
    while (mSpaceVectorHistory.size() > delay + 1)
      mSpaceVectorHistory.pop_front();
    for (int k = 0; k < 3; ++k) {
      mPeakEnvPrev[k] = mPeakEnv[k];
      mPeakEnv[k] = phaseEnvelope(k, 0.0);
    }
    if (active && !mCompHandedOver.isNull() && **mCompHandedOver &&
        mSpaceVectorHistory.size() == delay + 1) {
      const Complex earlier = mSpaceVectorHistory.front();
      v1 = (sv + earlier) / 2.;
      const Complex v2 = std::conj((sv - earlier) / 2. *
                                   std::polar(1.0, 2. * **mOmegaN * time));
      for (int k = 0; k < 3; ++k)
        mPeakEnv[k] = v1 + v2 * std::polar(1.0, 4. * PI * k / 3.) + mCompV[k];
    }
    if (active)
      Vnode = mCompSpaceVector ? sv : v1;
  }
  if (mTPll > 0 && std::abs(Vnode) > 1e-6 * mUdqNom) {
    Real err = std::arg(Vnode * std::polar(1.0, -mThetaT));
    mThetaT += (dt / mTPll) * err;
  }
  Complex vcdq = toDQ(Vnode);
  **mVcd = vcdq.real();
  **mVcq = vcdq.imag();

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
  Complex i2 = mNegSeq.step(envelope, **mOmegaN, time, dt, mUdqNom,
                            frt || mP_.DroopMode == 2, mP_.FrtRamp);
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
  // QrefSig is generation-signed, PeFilt receptor-signed (hence the minus)
  Real PeFilt = mPmeasFilter.step(Pinj, dt, false); // tP-filtered P (pu)
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
  // generator dq command -> receptor terminal current (leading minus);
  // negative sequence conj(I2) e^-2jwt, stamped in the next step
  const auto lim = Signal::WindGen::NegativeSequenceSupport::phaseLimit(
      fromDQ(idRef, 0.), fromDQ(0., iqRef), i2, mP_.IMaxPu,
      mNegSeq.released());
  idRef *= mNegSeq.activeFactor(lim.first, dt, mP_.FrtRamp);
  iqRef *= lim.second;
  i2 *= lim.second;
  **mIdRefPu = idRef;
  **mIqRefPu = iqRef;
  **mU2Pu = mNegSeq.mU2;
  **mI2Pu = i2;
  **mIsref = -(fromDQ(idRef * mIBaseDq, iqRef * mIBaseDq) +
               std::conj(i2 * mIBaseDq) *
                   std::polar(1.0, -2. * **mOmegaN * (time + dt)));

  // VDE HVRT/non-HVRT protection: mVrt.gate() scales the injection (0 when
  // disconnected, ramps to 1 on reconnection); a scheduled disconnect gives 0
  Real uProt;
  bool vtrip = false;
  if (!mCompCorrection.isNull()) {
    // instantaneous phases on sub-steps of at most 50 us
    int m = std::max(1, static_cast<int>(std::ceil(dt / 50e-6 - 1e-9)));
    Real h = dt / m;
    for (int i = 0; i < m; ++i) {
      Real lag = (m - 1 - i) * h;
      Real y[3];
      for (int k = 0; k < 3; ++k)
        y[k] = std::real(phaseEnvelope(k, lag) *
                         std::polar(1.0, **mOmegaN * (time - lag) -
                                             2. * PI * k / 3.));
      // above 50 us the peak criterion runs on the phase envelopes, as in DP3
      Real vPeak = 0.0, uHalf = 0.0;
      for (int k = 0; k < 3; ++k) {
        vPeak = std::max(
            vPeak, (m > 1 ? std::abs(peakEnvelope(k, lag))
                          : std::abs(y[k])) /
                       mUdqNom);
        Real vll = std::sqrt(2. / 3.) * (y[k] - y[(k + 1) % 3]) / mUdqNom;
        uHalf = std::max(uHalf, mUHalf[k].step(vll * vll, h, **mOmegaN));
      }
      vtrip = mVrt.stepMeasured(UFilt, vPeak, uHalf, m > 1, **mOmegaN, h,
                                uProt) ||
              vtrip;
    }
  } else {
    Real vPeak = UPu;
    Real uHalf = mUHalf[0].step(UPu * UPu, dt, **mOmegaN);
    vtrip = mVrt.stepMeasured(UFilt, vPeak, uHalf, true, **mOmegaN, dt, uProt);
  }
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

void DP::Ph1::WECC_WT4B::mnaCompAddPreStepDependencies(
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

void DP::Ph1::WECC_WT4B::mnaCompPreStep(Real time, Int timeStepCount) {
  // Norton compensation: the shunt current of the previous step (i_intf =
  // -G*V) is added to the source, so the net injection stays -mIsref; the
  // one-step delay leaves only a -G*dV damping term.
  **mSubCtrledCurrentSource->mCurrentRef =
      **mIsref +
      mSubShuntResistor->attributeTyped<MatrixComp>("i_intf")->get()(0, 0);
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void DP::Ph1::WECC_WT4B::mnaCompAddPostStepDependencies(
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

void DP::Ph1::WECC_WT4B::mnaCompPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void DP::Ph1::WECC_WT4B::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mWithConnectionTransformer)
    **mIntfCurrent = **mConnectionTransformer->mIntfCurrent;
  else
    **mIntfCurrent = **mSubCtrledCurrentSource->mIntfCurrent;
}

void DP::Ph1::WECC_WT4B::mnaCompUpdateVoltage(const Matrix &leftVector) {
  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);
  (**mIntfVoltage)(0, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0, 0));
}
