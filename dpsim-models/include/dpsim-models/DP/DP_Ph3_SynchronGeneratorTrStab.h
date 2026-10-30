// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <dpsim-models/Base/Base_SynchronGenerator.h>
#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph3_Inductor.h>
#include <dpsim-models/DP/DP_Ph3_VoltageSource.h>
#include <dpsim-models/Solver/MNAInterface.h>

namespace CPS {
namespace DP {
namespace Ph3 {
/// @brief Synchronous generator model for transient stability analysis
///
/// DP counterpart of EMT::Ph3::SynchronGeneratorTrStab: an emf of constant
/// magnitude behind X'd per phase, and a swing equation driven by the
/// instantaneous three-phase power sum_k Re{V_k e^{jwt}} Re{I_k e^{jwt}}.
class SynchronGeneratorTrStab : public CompositePowerComp<Complex>,
                                public Base::SynchronGenerator,
                                public SharedFactory<SynchronGeneratorTrStab> {
protected:
  /// Absolute d-axis transient reactance X'd
  Real mXpd;
  /// Absolute d-axis transient inductance
  Real mLpd;
  /// Absolute damping coefficient
  Real mKd;
  /// Equivalent impedance for initialization
  Complex mImpedance;
  /// Shift frequency of the envelopes
  Real mSystemOmega = 0;
  /// Inner voltage source that represents the generator
  std::shared_ptr<VoltageSource> mSubVoltageSource;
  /// Inner inductor that represents the generator impedance
  std::shared_ptr<Inductor> mSubInductor;

public:
  /// emf behind transient reactance, phase a
  const Attribute<Complex>::Ptr mEp;
  /// fixed absolute value of emf behind transient reactance
  const Attribute<Real>::Ptr mEp_abs;
  /// Angle by which the emf Ep is leading the system reference frame
  const Attribute<Real>::Ptr mDelta_p;

  SynchronGeneratorTrStab(String uid, String name,
                          Logger::Level logLevel = Logger::Level::off);

  SynchronGeneratorTrStab(String name,
                          Logger::Level logLevel = Logger::Level::off)
      : SynchronGeneratorTrStab(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  // #### General Functions ####
  /// Three-phase electrical power and mechanical power at the operating point
  void setInitialValues(Complex elecPower, Real mechPower);
  /// \brief Initializes the machine parameters
  void setStandardParametersSI(Real nomPower, Real nomVolt, Real nomFreq,
                               Int polePairNumber, Real Rs, Real Lpd,
                               Real inertiaJ, Real Kd = 0);
  /// \brief Initializes the machine parameters
  void setStandardParametersPU(Real nomPower, Real nomVolt, Real nomFreq,
                               Real Xpd, Real inertia, Real Rs = 0, Real D = 0);
  ///
  void step(Real time);
  /// Constructs and registers MNA subcomponents without emf value; idempotent.
  void createSubComponents() override;
  ///
  void initializeParentFromNodesAndTerminals(Real frequency) override;

  // #### MNA Functions ####
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override;
  void mnaParentPreStep(Real time, Int timeStepCount) override;
  void mnaParentPostStep(Real time, Int timeStepCount,
                         Attribute<Matrix>::Ptr &leftVector) override;
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;

  void mnaParentAddPreStepDependencies(
      AttributeBase::List &prevStepDependencies,
      AttributeBase::List &attributeDependencies,
      AttributeBase::List &modifiedAttributes) override;
  void
  mnaParentAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                   AttributeBase::List &attributeDependencies,
                                   AttributeBase::List &modifiedAttributes,
                                   Attribute<Matrix>::Ptr &leftVector) override;

  class AddBStep : public Task {
  public:
    AddBStep(SynchronGeneratorTrStab &generator)
        : Task(**generator.mName + ".AddBStep"), mGenerator(generator) {
      mAttributeDependencies.push_back(
          generator.mSubVoltageSource->mRightVector);
      mAttributeDependencies.push_back(generator.mSubInductor->mRightVector);
      mModifiedAttributes.push_back(generator.mRightVector);
    }

    void execute(Real time, Int timeStepCount);

  private:
    SynchronGeneratorTrStab &mGenerator;
  };
};
} // namespace Ph3
} // namespace DP
} // namespace CPS
