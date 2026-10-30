// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph3_Capacitor.h>
#include <dpsim-models/DP/DP_Ph3_Inductor.h>
#include <dpsim-models/DP/DP_Ph3_Resistor.h>
#include <dpsim-models/DP/DP_Ph3_VoltageSource.h>
#include <dpsim-models/Signal/PLL.h>
#include <dpsim-models/Signal/PowerControllerVSI.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// @brief Average grid-following inverter, three phase dynamic phasors
///
/// DP counterpart of EMT::Ph3::AvVoltageSourceInverterDQ without connection
/// transformer. PLL and power controller see the instantaneous phase values
/// Re{X e^{jwt}}, so an unbalanced terminal reaches them as in EMT.
class AvVoltageSourceInverterDQ
    : public CompositePowerComp<Complex>,
      public Base::AvVoltageSourceInverterDQ,
      public SharedFactory<AvVoltageSourceInverterDQ> {
protected:
  /// Nominal voltage
  Real mVnom;
  /// Simulation step
  Real mTimeStep;
  /// Shift frequency of the envelopes
  Real mSystemOmega = 0;
  /// Instant the last control output belongs to
  Real mControlTime = 0;
  /// PLL angle the last control output was transformed with
  Real mControlAngle = 0;

  // ### Control Subcomponents ###
  std::shared_ptr<Signal::PLL> mPLL;
  std::shared_ptr<Signal::PowerControllerVSI> mPowerControllerVSI;

  // ### Electrical Subcomponents ###
  std::shared_ptr<DP::Ph3::VoltageSource> mSubCtrledVoltageSource;
  std::shared_ptr<DP::Ph3::Resistor> mSubResistorF;
  std::shared_ptr<DP::Ph3::Capacitor> mSubCapacitorF;
  std::shared_ptr<DP::Ph3::Inductor> mSubInductorF;
  std::shared_ptr<DP::Ph3::Resistor> mSubResistorC;

  /// Flag for controller usage
  Bool mWithControl = true;

public:
  // ### General Parameters ###
  const Attribute<Real>::Ptr mOmegaN;
  const Attribute<Real>::Ptr mPref;
  const Attribute<Real>::Ptr mQref;

  // ### Inverter Interfacing Variables ###
  /// Measured voltage d-axis in local reference frame
  const Attribute<Real>::Ptr mVcd;
  /// Measured voltage q-axis in local reference frame
  const Attribute<Real>::Ptr mVcq;
  /// Measured current d-axis in local reference frame
  const Attribute<Real>::Ptr mIrcd;
  /// Measured current q-axis in local reference frame
  const Attribute<Real>::Ptr mIrcq;
  /// Control output in the dq frame of the PLL
  const Attribute<Matrix>::Ptr mVsrefDq;
  /// Control output as phase envelopes
  const Attribute<MatrixComp>::Ptr mVsref;
  /// PLL output
  const Attribute<Matrix>::Ptr mPllOutput;
  /// Power controller inputs, states and outputs for logging
  const Attribute<Matrix>::Ptr mPowerctrlInputs;
  const Attribute<Matrix>::Ptr mPowerctrlStates;
  const Attribute<Matrix>::Ptr mPowerctrlOutputs;

  AvVoltageSourceInverterDQ(String name,
                            Logger::Level logLevel = Logger::Level::off)
      : AvVoltageSourceInverterDQ(name, name, logLevel) {}
  AvVoltageSourceInverterDQ(String uid, String name,
                            Logger::Level logLevel = Logger::Level::off);

  // #### General ####
  void initializeParentFromNodesAndTerminals(Real frequency) override;
  void setParameters(Real sysOmega, Real sysVoltNom, Real Pref, Real Qref);
  void setControllerParameters(Real Kp_pll, Real Ki_pll, Real Kp_powerCtrl,
                               Real Ki_powerCtrl, Real Kp_currCtrl,
                               Real Ki_currCtrl, Real Omega_cutoff);
  void setFilterParameters(Real Lf, Real Cf, Real Rf, Real Rc);
  void setInitialStateValues(Real pInit, Real qInit, Real phi_dInit,
                             Real phi_qInit, Real gamma_dInit,
                             Real gamma_qInit);
  void withControl(Bool controlOn) { mWithControl = controlOn; };

  Matrix parkTransformPowerInvariant(Real theta, const Matrix &fabc);
  /// Phase envelopes whose instantaneous values at `time` are the inverse
  /// power invariant Park transformation of fdq at `theta`
  MatrixComp inverseParkToEnvelope(Real theta, Real time, const Matrix &fdq);
  /// Instantaneous phase values of envelopes at `time`
  Matrix instantaneous(const MatrixComp &envelope, Real time);

  // #### MNA section ####
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  // #### Control section ####
  void controlPreStep(Real time, Int timeStepCount);
  void controlStep(Real time, Int timeStepCount);
  void addControlPreStepDependencies(AttributeBase::List &prevStepDependencies,
                                     AttributeBase::List &attributeDependencies,
                                     AttributeBase::List &modifiedAttributes);
  void addControlStepDependencies(AttributeBase::List &prevStepDependencies,
                                  AttributeBase::List &attributeDependencies,
                                  AttributeBase::List &modifiedAttributes);

  class ControlPreStep : public CPS::Task {
  public:
    ControlPreStep(AvVoltageSourceInverterDQ &inverter)
        : Task(**inverter.mName + ".ControlPreStep"), mInverter(inverter) {
      mInverter.addControlPreStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mInverter.controlPreStep(time, timeStepCount);
    };

  private:
    AvVoltageSourceInverterDQ &mInverter;
  };

  class ControlStep : public CPS::Task {
  public:
    ControlStep(AvVoltageSourceInverterDQ &inverter)
        : Task(**inverter.mName + ".ControlStep"), mInverter(inverter) {
      mInverter.addControlStepDependencies(
          mPrevStepDependencies, mAttributeDependencies, mModifiedAttributes);
    }
    void execute(Real time, Int timeStepCount) {
      mInverter.controlStep(time, timeStepCount);
    };

  private:
    AvVoltageSourceInverterDQ &mInverter;
  };
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
