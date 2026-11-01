/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <dpsim-models/SP/SP_Ph1_WECC_WT4B.h>
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

SP::Ph1::WECC_WT4B::WECC_WT4B(String uid, String name, Logger::Level logLevel,
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
      mIsref(mAttributes->create<Complex>("Isref", Complex(0, 0))),
      mIs(mAttributes->createDynamic<MatrixComp>("Is")),
      mPllOutput(mAttributes->createDynamic<Matrix>("pll_output")) {
  mPhaseType = PhaseType::Single;
  mWithConnectionTransformer = withTrafo;
  if (withTrafo) {
    setVirtualNodeNumber(1); // VN0 = LV injection node
    mConnectionTransformer = SP::Ph1::Transformer::make(
        **mName + "_trans", **mName + "_trans", mLogLevel, false);
    mSubComponents.push_back(mConnectionTransformer);
  } else {
    setVirtualNodeNumber(0); // inject directly at terminal
  }
  setTerminalNumber(1);

  **mIntfVoltage = MatrixComp::Zero(1, 1);
  **mIntfCurrent = MatrixComp::Zero(1, 1);

  mSubShuntResistor = SP::Ph1::Resistor::make(**mName + "_resShunt", mLogLevel);
  mSubComponents.push_back(mSubShuntResistor);

  mPLL = Signal::PLL_Enhanced::make(**mName + "_PLL", mLogLevel);

  mIs->setReference(mIntfCurrent);
  mPLL->mInputRef->setReference(mVcq);
  mPLL->mInputRefD->setReference(mVcd);
  mPllOutput->setReference(mPLL->mOutputCurr);

  mSLog->info("Create {} {}", type(), name);
}

void SP::Ph1::WECC_WT4B::setParameters(Real sysOmega, Real sysVoltNom, Real Pref,
                                       Real Qref, Real Sn, Real cosPhi) {
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

void SP::Ph1::WECC_WT4B::setControllerParameters(Real Kp_pll, Real Ki_pll,
                                                 Real Omega_cutoff) {
  mPLL->setParameters(Kp_pll, Ki_pll, Omega_cutoff);
  mPLL->composeStateSpaceMatrices();
  mSLog->info("PLL: Kp={} Ki={} Omega={}", Kp_pll, Ki_pll, Omega_cutoff);
}

void SP::Ph1::WECC_WT4B::setTransformerParameters(
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

void SP::Ph1::WECC_WT4B::setFilterParameters(Real Lf, Real Cf, Real Rf,
                                             Real Rc) {
  // current injection, no series filter
  Base::AvVoltageSourceInverterDQ::setFilterParameters(Lf, Cf, Rf, Rc);
  mCf = 0;
}

Complex SP::Ph1::WECC_WT4B::toDQ(Complex phasor) {
  if (mPllClosed)
    return Math::rotatingFrame2to1(phasor, mThetaPll, 0.);
  // rotation into the control frame at the tracked angle mThetaT; SP phasor
  // convention S = V*conj(I), no extra factor
  return Math::rotatingFrame2to1(phasor, mThetaT, 0.);
}

Complex SP::Ph1::WECC_WT4B::fromDQ(Real d, Real q) {
  if (mPllClosed)
    return Math::rotatingFrame2to1(Complex(d, q), 0., mThetaPll);
  // dq command (generator convention) -> generated current phasor.
  return Math::rotatingFrame2to1(Complex(d, q), 0., mThetaT);
}

void SP::Ph1::WECC_WT4B::initializeFromNodesAndTerminals(Real frequency) {
  if (mWithConnectionTransformer)
    mVnomHV = mVnom * mTransformerRatioAbs;

  // complex single-phase terminal (HV) quantities from power flow
  Complex V0 = (std::abs(initialSingleVoltage(0)) > 0)
                   ? initialSingleVoltage(0)
                   : Complex(mVnomHV, 0);
  // receptor terminal current (S = V * conj(I))
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

  // current phasor stamped into the right-hand side at injNode; mIsref is the
  // generated current
  mSubShuntResistor->connect({injNode, SimNode::GND});
  // damping shunt, Norton-compensated in the right-hand side; env override
  // DPSIM_WT4B_RSHUNT
  Real zBase = (mVnomHV * mVnomHV) / mSNom;
  Real rShuntFactor = mShuntResistanceFactor;
  if (const char *rs = std::getenv("DPSIM_WT4B_RSHUNT"))
    try { rShuntFactor = std::stod(rs); } catch (...) {}
  Real rShunt = rShuntFactor * zBase;
  mSubShuntResistor->setParameters(rShunt);

  // dq bases from the initial operating point (UPu0 := 1 by construction)
  mTheta0 = std::arg(injNode->initialSingleVoltage());
  mThetaN = mTheta0;
  mThetaPll = mTheta0;
  mThetaT = mTheta0;
  mPllClosed = std::getenv("DPSIM_WT4B_PLL_CLOSED") != nullptr;
  // DPSIM_WT4B_TPLL: tracker time constant; DPSIM_WT4B_FROZEN_FRAME: frame
  // held at mTheta0
  if (const char *tp = std::getenv("DPSIM_WT4B_TPLL"))
    try { mTPll = std::stod(tp); } catch (...) {}
  if (std::getenv("DPSIM_WT4B_FROZEN_FRAME"))
    mTPll = 0.0;
  Complex vcdq0 = toDQ(Vlv);
  Complex idq0 = toDQ(Ilv);
  // nominal voltage base (Dynawo convention), not the initial voltage
  mUdqNom = mVnom;
  mIBaseDq = mSNom / mUdqNom;
  Real UPu0 = std::abs(vcdq0) / mUdqNom;
  if (UPu0 < 1e-6)
    UPu0 = 1.0;
  // generator convention (positive = generation); the terminal current is in
  // receptor convention, hence the negation
  mId0Pu = -idq0.real() / mIBaseDq;
  mIq0Pu = -idq0.imag() / mIBaseDq;

  **mVcd = vcdq0.real();
  **mVcq = vcdq0.imag();
  **mIrcd = idq0.real();
  **mIrcq = idq0.imag();
  **mIdRefPu = mId0Pu;
  **mIqRefPu = mIq0Pu;
  **mUFilteredPu = UPu0;
  // seed the voltage ride-through protection with the pre-fault operating point
  mVrt.initialize(UPu0);
  for (auto &h : mUHalf)
    h.initialize(UPu0 * UPu0);
  **mUProtPu = UPu0;
  mFrt.setParameters(mP_.DroopMode, mP_.Kqv, mP_.FrtRamp, mP_.UMinPu, mP_.UMaxPu);
  mFrt.initialize(UPu0);
  **mTripped = false;
  // initial injection: power-flow generated current (= -receptor current)
  **mIsref = -Ilv;

  for (auto subcomp : mSubComponents) {
    subcomp->initialize(mFrequencies);
    subcomp->initializeFromNodesAndTerminals(frequency);
  }

  // env overrides of the Q/V control gains and time constants, to detune the
  // collective mode of many identical parks
  auto envOverride = [](const char *name, Real &val) {
    if (const char *s = std::getenv(name))
      try { val = std::stod(s); } catch (...) {}
  };
  envOverride("DPSIM_WT4B_KQP", mP_.Kqp);
  envOverride("DPSIM_WT4B_KQI", mP_.Kqi); // outer Q-PI: Ti = Kqp/Kqi
  envOverride("DPSIM_WT4B_KVP", mP_.Kvp);
  envOverride("DPSIM_WT4B_KVI", mP_.Kvi); // inner V-PI: Ti = Kvp/Kvi
  envOverride("DPSIM_WT4B_TIQ", mP_.Tiq);
  envOverride("DPSIM_WT4B_TRV", mP_.tRv);
  envOverride("DPSIM_WT4B_TG", mP_.tG);
  // DPSIM_WT4B_CTRL_SCATTER=<frac>: per-park detuning of the Q/V PI gains,
  // derived from the component name, applied after the overrides above
  if (const char *sEnv = std::getenv("DPSIM_WT4B_CTRL_SCATTER")) {
    Real frac = 0;
    try { frac = std::stod(sEnv); } catch (...) {}
    if (frac > 0) {
      unsigned long long h = 1469598103934665603ull;
      for (unsigned char c : this->name()) { h ^= c; h *= 1099511628211ull; }
      auto fScat = [&](int g) {
        unsigned long long z = h + 0x9E3779B97F4A7C15ull * (unsigned long long)(g + 1);
        z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
        z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
        z ^= z >> 31;
        return 1.0 + frac * (2.0 * (Real)(z % 10007ull) / 10006.0 - 1.0);
      };
      mP_.Kqp *= fScat(0);
      mP_.Kqi *= fScat(1);
      mP_.Kvp *= fScat(2);
      mP_.Kvi *= fScat(3);
      mP_.Tiq *= fScat(4);
      mSLog->info("WT4B ctrl scatter ({}): Kqp={} Kqi={} Kvp={} Kvi={} Tiq={}",
                  this->name(), mP_.Kqp, mP_.Kqi, mP_.Kvp, mP_.Kvi, mP_.Tiq);
    }
  }

  // seed control blocks at the operating point (no transient at t=0)
  mUFilter.setParameters(1.0, mP_.tRv, -9999, 9999);
  mUFilter.initialize(UPu0);
  mUFilterGC.setParameters(1.0, mP_.tFilterGC, -9999, 9999);
  mUFilterGC.initialize(UPu0);
  mPmeasFilter.setParameters(1.0, mP_.tP, -9999, 9999);
  // PeFilt is receptor-signed (negative when generating)
  mPmeasFilter.initialize(
      (vcdq0.real() * idq0.real() + vcdq0.imag() * idq0.imag()) / mSNom);
  mIqFilter.setParameters(1.0, mP_.Tiq, -9999, 9999);
  mIqFilter.initialize(mIq0Pu);
  mPfaRef = std::atan2(**mQref, **mPref);
  mPSlew.setParameters(mP_.DPMin, mP_.DPMax, -9999, 9999);
  mPSlew.initialize(mId0Pu * UPu0); // Pord path carries P (=id*U)
  mPordFilter.setParameters(1.0, mP_.tPord, -9999, 9999);
  mPordFilter.initialize(mId0Pu * UPu0);
  mQPid.setParameters(mP_.Kqp, (mP_.Kqi != 0) ? mP_.Kqp / mP_.Kqi : 1e6, 1.0,
                      mP_.Ni, false, 1.0);
  mQPid.initialize((mP_.Kqp != 0) ? UPu0 / mP_.Kqp : 0.0);
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

  mSLog->info("\n--- WT4B (SP Ph1) init --- UdqNom={} IBaseDq={} Id0Pu={} "
              "Iq0Pu={} theta0={}",
              mUdqNom, mIBaseDq, mId0Pu, mIq0Pu, mTheta0);
}

void SP::Ph1::WECC_WT4B::mnaCompInitialize(Real omega, Real timeStep,
                                           Attribute<Matrix>::Ptr leftVector) {
  updateMatrixNodeIndices();
  mTimeStep = timeStep;

  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaInitialize(omega, timeStep, leftVector);

  mPLL->setSimulationParameters(timeStep);

  // matrix index of the injection node (LV virtual node with trafo, else the
  // terminal) for direct phasor-current RHS stamping
  mInjIdx = mWithConnectionTransformer ? mVirtualNodes[0]->matrixNodeIndex()
                                       : matrixNodeIndex(0);

  if (mWithConnectionTransformer)
    mRightVectorStamps.push_back(&**mConnectionTransformer->mRightVector);

  mMnaTasks.push_back(std::make_shared<MnaPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<MnaPostStep>(*this, leftVector));
  mMnaTasks.push_back(std::make_shared<ControlPreStep>(*this));
  mMnaTasks.push_back(std::make_shared<ControlStep>(*this));

  **mRightVector = Matrix::Zero(leftVector->get().rows(), 1);
}

void SP::Ph1::WECC_WT4B::mnaCompApplySystemMatrixStamp(
    SparseMatrixRow &systemMatrix) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaApplySystemMatrixStamp(systemMatrix);
}

void SP::Ph1::WECC_WT4B::mnaCompApplyRightSideVectorStamp(Matrix &rightVector) {
  rightVector.setZero();
  for (auto stamp : mRightVectorStamps)
    rightVector += *stamp;
  // current injection, Norton-compensated for the shunt: net injection stays
  // mIsref, the one-step lag leaves a -G*dv damping term
  Math::addToVectorElement(
      rightVector, mInjIdx,
      **mIsref - mSubShuntResistor->attributeTyped<MatrixComp>("i_intf")->get()(0, 0));
}

void SP::Ph1::WECC_WT4B::addControlPreStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddPreStepDependencies(prevStepDependencies, attributeDependencies,
                                     modifiedAttributes);
}

void SP::Ph1::WECC_WT4B::controlPreStep(Real time, Int timeStepCount) {
  mPLL->signalPreStep(time, timeStepCount);
}

void SP::Ph1::WECC_WT4B::addControlStepDependencies(
    AttributeBase::List &prevStepDependencies,
    AttributeBase::List &attributeDependencies,
    AttributeBase::List &modifiedAttributes) {
  mPLL->signalAddStepDependencies(prevStepDependencies, attributeDependencies,
                                  modifiedAttributes);
  attributeDependencies.push_back(attribute("i_intf"));
  attributeDependencies.push_back(attribute("v_intf"));
  modifiedAttributes.push_back(attribute("Isref"));
}

void SP::Ph1::WECC_WT4B::controlStep(Real time, Int timeStepCount) {
  Real dt = mTimeStep;

  // injection-node voltage into the tracked control frame; the first-order
  // angle tracker avoids the single-step V-I loop of a direct arg(V)
  Complex Vnode = mWithConnectionTransformer
                      ? mVirtualNodes[0]->mVoltage->get()(0, 0)
                      : (**mIntfVoltage)(0, 0);
  if (!mPllClosed && mTPll > 0 && std::abs(Vnode) > 1e-6 * mUdqNom) {
    Real err = std::arg(Vnode * std::polar(1.0, -mThetaT));
    mThetaT += (dt / mTPll) * err;
  }
  Complex vcdq = toDQ(Vnode);
  **mVcd = vcdq.real();
  **mVcq = vcdq.imag();

  // injected current (previous command) for power feedback
  Real idPrevPu = **mIdRefPu;
  Real iqPrevPu = **mIqRefPu;
  **mIrcd = -idPrevPu * mIBaseDq;
  **mIrcq = -iqPrevPu * mIBaseDq;

  mPLL->signalStep(time, timeStepCount);
  // closed-PLL: dq frame follows the PLL angle from the next step on
  if (mPllClosed)
    mThetaPll = mPLL->mOutputCurr->get()(0, 0);

  Real Umag = std::hypot(**mVcd, **mVcq);
  Real UPu = Umag / mUdqNom;
  Real Pinj = (**mVcd * **mIrcd + **mVcq * **mIrcq) / mSNom;
  Real Qinj = (**mVcq * **mIrcd - **mVcd * **mIrcq) / mSNom;
  **mP = Pinj * mSNom;
  **mQ = Qinj * mSNom;

  Real UFilt = mUFilter.step(UPu, dt, false);
  **mUFilteredPu = UFilt;
  Real Uden = std::max(UFilt, 0.01);
  // FRT detection on the tRv-filtered voltage (WECC REEC Vt_filt)
  bool frt = mFrt.step(UFilt, dt);
  **mFaultState = frt;

  // ---- active power channel ----
  Real omegaGen = 1.0;
  Real PrefPu = (**mPref / mSNom) * (mP_.PFlag ? omegaGen : 1.0);
  Real Pslew = mPSlew.step(PrefPu, dt, false);
  Real Pord = mPordFilter.step(Pslew, dt, frt);
  Pord = clampR(Pord, mP_.PMinPu, mP_.PMaxPu);
  Real idCmd = Pord / Uden;

  // ---- reactive control (ElectricalControlCommon) ----
  bool freezePI = frt;
  Real PeFilt = mPmeasFilter.step(Pinj, dt, false);
  // QrefSig is generator-signed; PeFilt is receptor-signed, hence negated
  Real QrefSig =
      mP_.PfFlag ? -PeFilt * std::tan(mPfaRef) : (**mQref / mSNom);
  Real VRefConst = (mP_.VRef0Pu < 0.5) ? 1.0 : mP_.VRef0Pu;

  Real QrefLim = clampR(QrefSig, mP_.QMinPu, mP_.QMaxPu);
  // Qinj is receptor-signed, so the reference enters negated
  Real Vqcmd = mQPid.step(QrefLim, -Qinj, dt, freezePI, mP_.VMinPu, mP_.VMaxPu);

  Real vInnerRef = clampR(mP_.VFlag ? Vqcmd : VRefConst, mP_.VMinPu, mP_.VMaxPu);
  Real iqVloop =
      -mVPid.step(vInnerRef, UFilt, dt, freezePI, -mP_.IMaxPu, mP_.IMaxPu);

  // port q-convention: iq<0 injects, so +Qgen needs -QrefSig/Uden
  Real iqDirect = mIqFilter.step(-QrefSig / Uden, dt, freezePI);

  Real iqMain = mP_.QFlag ? iqVloop : iqDirect;

  Real db = deadZone(VRefConst - UFilt, mP_.Dbd1, mP_.Dbd2);
  // q-axis convention: iq > 0 absorbs, iq < 0 raises V; droop and clamps
  // enter negated
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
  Real idRamp = mIdRegc.step(idCmd, dt, false);
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
  // REGC HV clamp, negated for the q-axis convention
  Real iqRef = std::min(iqRamp + iqhv, -mP_.Iolim);

  **mIdRefPu = idRef;
  **mIqRefPu = iqRef;

  // ---- direct current injection (InjectorIDQ) ----
  // dq command in generator convention -> generated current phasor
  **mIsref = fromDQ(idRef * mIBaseDq, iqRef * mIBaseDq);

  // VDE protection: gate() scales the current (0 = disconnected, ramps to 1
  // on reconnection); a scheduled disconnect forces 0 without reconnection
  Real vPeak = UPu;
  Real uHalf = mUHalf[0].step(UPu * UPu, dt, **mOmegaN);
  Real uProt;
  bool vtrip = mVrt.stepMeasured(UFilt, vPeak, uHalf, true, **mOmegaN, dt, uProt);
  **mUProtPu = uProt;
  bool alm = (mTDisconnect > 0 && time >= mTDisconnect);
  Real g = alm ? 0.0 : mVrt.gate();
  **mTripped = vtrip || alm;
  if (g < 1.0) {
    // scale injection, reported P/Q and current commands
    **mIsref = **mIsref * g;
    **mP = **mP * g; **mQ = **mQ * g;
    **mIdRefPu = **mIdRefPu * g; **mIqRefPu = **mIqRefPu * g;
  }

  mThetaN += dt * **mOmegaN;
}

void SP::Ph1::WECC_WT4B::mnaCompAddPreStepDependencies(
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

void SP::Ph1::WECC_WT4B::mnaCompPreStep(Real time, Int timeStepCount) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaPreStep(time, timeStepCount);
  mnaCompApplyRightSideVectorStamp(**mRightVector);
}

void SP::Ph1::WECC_WT4B::mnaCompAddPostStepDependencies(
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

void SP::Ph1::WECC_WT4B::mnaCompPostStep(Real time, Int timeStepCount,
                                         Attribute<Matrix>::Ptr &leftVector) {
  for (auto subcomp : mSubComponents)
    if (auto mnasubcomp = std::dynamic_pointer_cast<MNAInterface>(subcomp))
      mnasubcomp->mnaPostStep(time, timeStepCount, leftVector);
  mnaCompUpdateCurrent(**leftVector);
  mnaCompUpdateVoltage(**leftVector);
}

void SP::Ph1::WECC_WT4B::mnaCompUpdateCurrent(const Matrix &leftVector) {
  if (mWithConnectionTransformer)
    **mIntfCurrent = **mConnectionTransformer->mIntfCurrent;
  else
    // injected (generated) terminal current
    (**mIntfCurrent)(0, 0) = **mIsref;
}

void SP::Ph1::WECC_WT4B::mnaCompUpdateVoltage(const Matrix &leftVector) {
  for (auto virtualNode : mVirtualNodes)
    virtualNode->mnaUpdateVoltage(leftVector);
  (**mIntfVoltage)(0, 0) =
      Math::complexFromVectorElement(leftVector, matrixNodeIndex(0));
}
