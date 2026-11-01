/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2025 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNAInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/DP/DP_Ph3_Resistor.h>
#include <dpsim-models/DP/DP_Ph3_Transformer.h>
#include <dpsim-models/Base/Base_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/Signal/PLL_Enhanced.h>
#include <dpsim-models/Signal/WindGenericBlocks.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// \brief WECC WT4B generic wind turbine (REGC_A current-source interface),
/// dynamic-phasor three-phase model.
///
/// Per-unit control of the Dynawo WECC WT4B model, same as EMT::Ph3 and
/// DP::Ph1::WECC_WT4B. A balanced positive-sequence current set is injected
/// directly at the terminal (no series Lf/Rf filter) with a high shunt
/// resistance. The dq frame is the phase-A phasor rotated by the terminal
/// angle and scaled by sqrt(3/2), giving the EMT Park (k=sqrt(2/3)) power.
class WECC_WT4B : public Base::AvVoltageSourceInverterDQ,
                  public MNASimPowerComp<Complex>,
                  public MNAVariableCompInterface,
                  public SharedFactory<WECC_WT4B> {
public:
  /// Tunable parameter set (REEC + REGC_A), with WECC-typical defaults.
  struct Parameters {
    // --- REEC mode flags ---
    bool QFlag = false; // false: constant pf / Q ref; true: Q/V PI
    bool VFlag = false; // false: inner V loop; true: local Q control
    bool PfFlag = false; // true: power-factor mode (Q = P*tan(phi))
    // true: P priority, Q priority during FRT (VDE-AR-N 4110/4120);
    // false: Q priority always
    bool PPriority = true;
    // --- voltage filtering / FRT detection ---
    Real tRv = 0.006;
    Real UMinPu = 0.9; // FRT under-voltage threshold
    Real UMaxPu = 1.1; // FRT over-voltage threshold
    Real VRef0Pu = 1.0; // voltage setpoint (< 0.5: 1.0 pu)
    Real VRef1Pu = 0.0; // inner-loop reference bias
    // --- reactive droop / deadband ---
    Real Dbd1 = -0.05; // overvoltage deadband
    Real Dbd2 = 0.05;  // undervoltage deadband
    Real Kqv = 5.0;    // reactive droop gain (VDE modes: k, 4120 default 5)
    int DroopMode = 1; // 0 REEC deadband, 1 VDE triggered, 2 VDE continuous
    Real FrtRamp = 0.1; // support-current ramp-out after fault end [s]
    Real Iqh1Pu = 1.05;
    Real Iql1Pu = -1.05;
    // --- Q control PI (limPIDFreeze) ---
    Real tP = 0.05;
    Real QMaxPu = 0.4;
    Real QMinPu = -0.4;
    Real Kqp = 1.0;
    Real Kqi = 10.0;
    // --- V control PI (varLimPIDFreeze) ---
    Real VMaxPu = 1.1;
    Real VMinPu = 0.9;
    Real Kvp = 1.0;
    Real Kvi = 10.0;
    Real Ni = 20.0; // anti-windup gain
    // --- reactive current filter ---
    Real Tiq = 0.01;
    // --- active power channel ---
    Real tPord = 0.02;
    Real PMaxPu = 1.0;
    Real PMinPu = 0.0;
    Real DPMax = 999.0;
    Real DPMin = -999.0;
    bool PFlag = false; // true: scale Pref by generator speed
    // --- current limit ---
    Real IMaxPu = 1.1;
    // --- FRT reactive injection ---
    Real HoldIpMax = 0.0; // not implemented
    Real HoldIq = 0.0; // not implemented
    Real IqFrzPu = 0.0; // not implemented
    // --- REGC_A ---
    Real tG = 0.012;
    Real TPRec = 1.0; // active-current recovery ramp after the fault end [s]
    Real tFilterGC = 0.02;
    Real IqrMaxPu = 999.0;
    Real IqrMinPu = -999.0;
    Real Rrpwr = 10.0;
    bool RateFlag = false;
    bool Lvplsw = false;
    Real zerox = 0.4;
    Real brkpt = 0.9;
    Real lvpl1 = 1.22;
    // --- REGC_A HV reactive current management ---
    Real Volim = 1.2;
    Real Iolim = -1.05;
    Real Khv = 0.7;
  };

protected:
  // ### General parameters ###
  Real mTimeStep = 0;
  Real mThetaN = 0;
  /// Initial phase-A voltage angle
  Real mTheta0 = 0;
  /// dq frame angle, first-order lag on the terminal-voltage angle (mTPll)
  Real mThetaT = 0;
  /// Frame time constant [s]; <= 0 freezes the frame at mTheta0
  /// (env DPSIM_WT4B_TPLL, DPSIM_WT4B_FROZEN_FRAME).
  Real mTPll = 0.02;
  Real mVnom = 0;
  Real mVnomHV = 0;
  Real mSn = 0;
  /// Apparent-power base for the per-unit control (= mSn)
  Real mSNom = 0;
  /// dq current base: SNom / |Vcdq| at nominal voltage
  Real mIBaseDq = 1.0;
  /// dq voltage magnitude at nominal voltage (1 pu)
  Real mUdqNom = 1.0;

  Parameters mP_;

  // ### Electrical subcomponents (direct injection) ###
  // Current phasor stamped directly into the source vector at the injection
  // node; DP::Ph3::CurrentSource is sine-modulated, not a constant phasor.
  std::shared_ptr<DP::Ph3::Resistor> mSubShuntResistor;
  /// rShunt = factor * zBase; 1e5 negligible, ~1e3 damps the injection node
  /// (env DPSIM_WT4B_RSHUNT).
  Real mShuntResistanceFactor = 1e5;
  /// Optional connection transformer
  std::shared_ptr<DP::Ph3::Transformer> mConnectionTransformer;
  Bool mWithConnectionTransformer = false;
  /// Matrix indices of the injection node, per phase
  Matrix::Index mInjIdx[3] = {0, 0, 0};

  // ### Control subcomponents ###
  std::shared_ptr<Signal::PLL_Enhanced> mPLL;

  // --- REEC signal blocks ---
  Signal::WindGen::RateLimFirstOrderFreeze mPordFilter;     // tPord PT1 on Pord
  Signal::WindGen::AbsLimRateLimFeedthroughFreeze mPSlew;   // Pref slew
  Signal::WindGen::RateLimFirstOrderFreeze mUFilter;        // tRv U filter
  Signal::WindGen::RateLimFirstOrderFreeze mPmeasFilter;    // tP P filter
  Signal::WindGen::RateLimFirstOrderFreeze mIqFilter;       // Tiq iq filter
  Signal::WindGen::LimPIDFreeze mQPid;                      // Q control PI
  Signal::WindGen::VarLimPIDFreeze mVPid;                   // V control PI
  // --- REGC signal blocks ---
  Signal::WindGen::RateLimFirstOrderFreeze mIdRegc;         // tG, Rrpwr id ramp
  Signal::WindGen::ActiveCurrentRecoveryRamp mPRec;
  Signal::WindGen::RateLimFirstOrderFreeze mIqRegc;         // tG iq ramp
  Signal::WindGen::RateLimFirstOrderFreeze mUFilterGC;      // tFilterGC filter
  // --- VDE-AR-N 4110/4120 HVRT / non-HVRT overvoltage protection ---
  Signal::WindGen::VoltageRideThrough mVrt;                 // latched trip
  Signal::WindGen::HalfCycleRms mUHalf[3];
  Signal::WindGen::VdeFaultRideThrough mFrt;

  // initial (seed) per-unit current commands
  Real mId0Pu = 0;
  Real mIq0Pu = 0;
  /// power-factor angle for PfFlag mode (= atan2(Qref, Pref))
  Real mPfaRef = 0;

  std::vector<const Matrix *> mRightVectorStamps;

  /// Disconnect (trip) event time; <0 = no event scheduled
  Real mTDisconnect = -1;

  Bool hasParameterChanged() override { return false; };

public:
  // ### Attributes ###
  const Attribute<Real>::Ptr mOmegaN;
  const Attribute<Real>::Ptr mPref;
  const Attribute<Real>::Ptr mQref;
  const Attribute<Real>::Ptr mP;
  const Attribute<Real>::Ptr mQ;
  /// Filtered terminal voltage magnitude (pu)
  const Attribute<Real>::Ptr mUFilteredPu;
  /// d/q current commands to the injector (pu, base SNom)
  const Attribute<Real>::Ptr mIdRefPu;
  const Attribute<Real>::Ptr mIqRefPu;
  /// Measured dq voltage / current (SI dq frame)
  const Attribute<Real>::Ptr mVcd;
  const Attribute<Real>::Ptr mVcq;
  const Attribute<Real>::Ptr mIrcd;
  const Attribute<Real>::Ptr mIrcq;
  /// Fault ride-through active
  const Attribute<Bool>::Ptr mFaultState;
  /// Latched HVRT/non-HVRT overvoltage trip (true = disconnected)
  const Attribute<Bool>::Ptr mTripped;
  /// voltage measured by the ride-through protection, p.u.
  const Attribute<Real>::Ptr mUProtPu;
  /// Balanced complex current reference injected by the controlled source
  const Attribute<MatrixComp>::Ptr mIsref;
  const Attribute<MatrixComp>::Ptr mIs;
  const Attribute<Matrix>::Ptr mPllOutput;

  WECC_WT4B(String name, Logger::Level logLevel = Logger::Level::off)
      : WECC_WT4B(name, name, logLevel) {}
  WECC_WT4B(String uid, String name, Logger::Level logLevel = Logger::Level::off,
            Bool withTrafo = false);

  // #### General ####
  void initializeFromNodesAndTerminals(Real frequency) override;
  void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref,
                     Real Sn = 0, Real cosPhi = 0);
  void setControlParameters(const Parameters &params) { mP_ = params; }
  /// Set the injection-node shunt factor (rShunt = factor * zBase).
  void setShuntResistanceFactor(Real factor) { mShuntResistanceFactor = factor; }
  /// Disconnect at eventTime: injected current forced to zero; the
  /// Norton-compensated shunt stays and keeps damping the node.
  void setDisconnectEvent(Real eventTime) { mTDisconnect = eventTime; }
  /// Enable VDE-AR-N 4110/4120 voltage ride-through protection: hvrt=false =
  /// old / non-HVRT unit (trips above ~1.11 p.u.), hvrt=true = HVRT-capable
  /// (level 1 = EHV/HV 4120, 2 = MV 4110); enabled=false leaves it inert.
  void setProtectionParameters(bool hvrt, int level, bool enabled = true,
                               bool reconnect = false, Real recDelay = 1.0,
                               Real recRamp = 2.0) {
    mVrt.setParameters(hvrt, level, enabled);
    mVrt.setReconnection(reconnect, recDelay, recRamp);
  }
  /// Protection measurand: 0 = filtered positive-sequence magnitude, 1 = phase
  /// voltage peak > 1.25 p.u. for 0.2 ms, 2 = half-cycle RMS of the largest
  /// line-line voltage against the envelope.
  void setProtectionMeasurement(int measurement) { mVrt.setMeasurement(measurement); }
  void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Omega_cutoff);
  void setTransformerParameters(Real nomVoltageEnd1, Real nomVoltageEnd2,
                                Real ratedPower, Real ratioAbs, Real ratioPhase,
                                Real resistance, Real inductance, Real omega);
  void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);

  /// Phase-A phasor -> EMT-Park dq, vd + j vq, scaled by sqrt(3/2)
  Complex toDQ(Complex phasorA);
  /// dq command (d,q) -> generator phase-A current phasor.
  Complex fromDQ(Real d, Real q);

  // #### MNA ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaCompPreStep(Real time, Int timeStepCount) override;
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void mnaCompAddPostStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes,
      Attribute<Matrix>::Ptr &leftVector) override;

  // #### Control ####
  void controlPreStep(Real time, Int timeStepCount);
  void controlStep(Real time, Int timeStepCount);
  void addControlPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes);
  void addControlStepDependencies(AttributeBase::List &prevStepDependencies,
                                  AttributeBase::List &attributeDependencies,
                                  AttributeBase::List &modifiedAttributes);

  class ControlPreStep : public CPS::Task {
  public:
    ControlPreStep(WECC_WT4B &comp)
        : Task(**comp.mName + ".ControlPreStep"), mComp(comp) {
      mComp.addControlPreStepDependencies(mPrevStepDependencies,
                                          mAttributeDependencies,
                                          mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mComp.controlPreStep(time, timeStepCount);
    };

  private:
    WECC_WT4B &mComp;
  };

  class ControlStep : public CPS::Task {
  public:
    ControlStep(WECC_WT4B &comp)
        : Task(**comp.mName + ".ControlStep"), mComp(comp) {
      mComp.addControlStepDependencies(mPrevStepDependencies,
                                       mAttributeDependencies,
                                       mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mComp.controlStep(time, timeStepCount);
    };

  private:
    WECC_WT4B &mComp;
  };

  class MnaPreStep : public CPS::Task {
  public:
    MnaPreStep(WECC_WT4B &comp)
        : Task(**comp.mName + ".MnaPreStep"), mComp(comp) {
      mComp.mnaAddPreStepDependencies(mPrevStepDependencies,
                                      mAttributeDependencies,
                                      mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mComp.mnaPreStep(time, timeStepCount);
    };

  private:
    WECC_WT4B &mComp;
  };

  class MnaPostStep : public CPS::Task {
  public:
    MnaPostStep(WECC_WT4B &comp, Attribute<Matrix>::Ptr leftVector)
        : Task(**comp.mName + ".MnaPostStep"), mComp(comp),
          mLeftVector(leftVector) {
      mComp.mnaAddPostStepDependencies(mPrevStepDependencies,
                                       mAttributeDependencies,
                                       mModifiedAttributes, mLeftVector);
    }
    void execute(Real time, Int timeStepCount) {
      mComp.mnaPostStep(time, timeStepCount, mLeftVector);
    };

  private:
    WECC_WT4B &mComp;
    Attribute<Matrix>::Ptr mLeftVector;
  };
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
