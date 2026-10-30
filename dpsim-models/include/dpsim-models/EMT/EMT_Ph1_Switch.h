/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/Base/Base_Ph1_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace EMT {
namespace Ph1 {
/// \brief One phase EMT switch
///
/// The switch can be opened and closed.
/// Each state has a specific resistance value.
class Switch : public MNASimPowerComp<Real>,
               public Base::Ph1::Switch,
               public SharedFactory<Switch>,
               public MNAVariableCompInterface,
               public MNASwitchInterface {
public:
  /// Ideal opens at the command, CurrentZero at the next sampled current zero
  enum class SwitchingMode {
    Ideal = 0,
    CurrentZero = 1,
  };

  const Attribute<Bool>::Ptr mPoleClosed;
  const Attribute<Real>::Ptr mZeroCrossingTime;

  /// Defines UID, name, component parameters and logging level
  Switch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Switch(String name, Logger::Level logLevel = Logger::Level::off)
      : Switch(name, name, logLevel) {}

  SimPowerComp<Real>::Ptr clone(String name) override;

  void setSwitchingMode(SwitchingMode mode);
  SwitchingMode switchingMode() const { return mSwitchingMode; }
  void setZeroCrossingTolerance(Real tolerance) {
    mZeroCrossingTolerance = tolerance;
  }

  void openSwitch() override;
  void closeSwitch() override;

  // #### General ####
  /// Initializes component from power flow data
  void initializeFromNodesAndTerminals(Real frequency) override;

  // #### General MNA section ####
  void mnaCompInitialize(Real omega, Real timeStep,
                         Attribute<Matrix>::Ptr leftVector) override;
  /// Stamps system matrix
  void mnaCompApplySystemMatrixStamp(SparseMatrixRow &systemMatrix) override;
  /// Stamps right side (source) vector
  void mnaCompApplyRightSideVectorStamp(Matrix &rightVector) override;
  /// Update interface voltage from MNA system result
  void mnaCompUpdateVoltage(const Matrix &leftVector) override;
  /// Update interface current from MNA system result
  void mnaCompUpdateCurrent(const Matrix &leftVector) override;

  // #### MNA section for switches ####
  /// Check if switch is closed
  Bool mnaIsClosed() override;
  Bool supportsPrecomputedSystemMatrices() const override;
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;

  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;
  // #### MNA section for variable component ####
  Bool hasParameterChanged() override;

private:
  SwitchingMode mSwitchingMode = SwitchingMode::Ideal;
  Real mZeroCrossingTolerance = 1e-6;
  Bool mOpeningRequested = false;
  Bool mPreviousCurrentValid = false;
  Real mPreviousCurrent = 0.0;
  Real mPreviousTime = 0.0;

  void updateZeroCrossingState(Real time);
};
} // namespace Ph1
} // namespace EMT
} // namespace CPS
