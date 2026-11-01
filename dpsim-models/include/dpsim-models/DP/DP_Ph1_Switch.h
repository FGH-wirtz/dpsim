/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <array>

#include <dpsim-models/Base/Base_Ph1_Switch.h>
#include <dpsim-models/Definitions.h>
#include <dpsim-models/Logger.h>
#include <dpsim-models/MNASimPowerComp.h>
#include <dpsim-models/Solver/MNACompensatedInterface.h>
#include <dpsim-models/Solver/MNASwitchInterface.h>
#include <dpsim-models/Solver/MNAVariableCompInterface.h>

namespace CPS {
namespace DP {
namespace Ph1 {
class ZeroSequenceEquivalent;
/// \brief Dynamic phasor switch
///
/// The switch can be opened and closed.
/// Each state has a specific resistance value.
///
/// Switching modes:
///
/// - Ideal:
///   conventional binary switch.
///
/// - CurrentZero:
///   reconstruct the physical instantaneous current from the DP envelope,
///
///       i(t) = Re{ I(t) exp(j omega_s t) },
///
///   and open at its current zero.
///
/// - ExponentialZCSEmulation:
///   increase the switch resistance continuously according to
///
///       R(t) = R_closed * (R_open/R_closed)^alpha,
///
///   alpha in [0,1], over a user-defined switching duration. Opening runs
///   alpha from 0 to 1, closing runs it from 1 to 0, since the envelope
///   cannot represent a discontinuity in either direction. This is the
///   variable-resistance ZCS-emulation approach and is not an arc model.
///
/// - Compensated:
///   the network pole stays closed; compensation currents across the breaker
///   open the poles one by one at their current zeros (Dommel). The network
///   enters through its time-domain Thevenin equivalents across the breaker,
///   positive and zero sequence. Assumes a linear, symmetric network.
///
/// - SpaceVector:
///   the network pole stays open; the solver injects the current as space
///   vector (MNACompensatedInterface), the zero sequence through a shared
///   zero-sequence network. One condition per pole on instantaneous values:
///   closed v + R i = 0, open i = 0. A pole mask gives unsymmetrical faults.
class Switch : public MNASimPowerComp<Complex>,
               public Base::Ph1::Switch,
               public SharedFactory<Switch>,
               public MNASwitchInterface,
               public MNAVariableCompInterface,
               public MNACompensatedInterface {
public:
  enum class SwitchingMode {
    Ideal = 0,
    CurrentZero = 1,
    ExponentialZCSEmulation = 2,
    Compensated = 3,
    SpaceVector = 4,
  };

  const Attribute<Bool>::Ptr mOpeningRequested;

  const Attribute<Bool>::Ptr mPoleClosed;

  /// Reconstructed physical instantaneous current [A].
  const Attribute<Real>::Ptr mInstantCurrent;

  const Attribute<Real>::Ptr mZeroCrossingTime;

  /// Exponential-transition diagnostics.
  const Attribute<Bool>::Ptr mExponentialTransitionActive;
  /// True while the active exponential transition is a closing transition.
  const Attribute<Bool>::Ptr mExponentialTransitionClosing;
  /// Position on the resistance path, 0 at R_closed and 1 at R_open. Runs
  /// 0 -> 1 while opening and 1 -> 0 while closing.
  const Attribute<Real>::Ptr mExponentialProgress;
  const Attribute<Real>::Ptr mExponentialTransitionStartTime;
  const Attribute<Real>::Ptr mExponentialTransitionEndTime;

  /// Effective resistance currently stamped into the MNA matrix [ohm].
  const Attribute<Real>::Ptr mEffectiveResistance;

  /// Compensated: true from the first opening command on
  const Attribute<Bool>::Ptr mCompensationActive;
  /// Compensated: compensation currents D of the three poles (3x1 envelopes)
  const Attribute<MatrixComp>::Ptr mCompensationCurrent;
  /// Compensated: index of the equivalents in use (topology behind the breaker)
  const Attribute<Int>::Ptr mTheveninIndex;
  /// Compensated: true while the network carries the states handed over
  const Attribute<Bool>::Ptr mCompHandedOver;

  /// Defines UID, name, component parameters and logging level
  Switch(String uid, String name, Logger::Level loglevel = Logger::Level::off);
  /// Defines name, component parameters and logging level
  Switch(String name, Logger::Level logLevel = Logger::Level::off)
      : Switch(name, name, logLevel) {}

  SimPowerComp<Complex>::Ptr clone(String name) override;

  void setSwitchingMode(SwitchingMode mode);
  SwitchingMode switchingMode() const { return mSwitchingMode; }

  void setZeroCrossingTolerance(Real tolerance);

  void setExponentialSwitchingTime(Real switchingTime);
  Real exponentialSwitchingTime() const { return mExponentialSwitchingTime; }

  /// Storage element of the equivalent: name, nodes, L or C
  using StorageElement = MNASwitchInterface::StorageElement;
  /// Time-domain Thevenin equivalent across the open breaker: input the current
  /// (re, im), outputs v0 - v1 and the observed node voltages;
  /// x[k] = A x[k-1] + B u[k], y[k] = C x[k-1] + D u[k].
  /// Handover: voltage and current of each storage element at the end of the
  /// step, s[k] = Ex x[k-1] + Eu u[k].
  struct TheveninEquivalent {
    Matrix A, B, C, D;
    std::vector<String> stateNames;
    std::vector<StorageElement> elements;
    Matrix Ex, Eu;
  };
  /// Observed node; phaseShift: positive-sequence shift on the path (e.g.
  /// -150 deg behind Yd5); zeroSequence false: the path blocks it (delta).
  struct ObservedNode {
    String name;
    Real phaseShift = 0.;
    Bool zeroSequence = true;
  };
  /// Compensated: positive- and zero-sequence equivalents and the observed
  /// nodes; each node gets compensation_v_<name>, its phase voltage
  /// corrections.
  void setTheveninEquivalents(const TheveninEquivalent &positive,
                              const TheveninEquivalent &zero,
                              const std::vector<ObservedNode> &nodes);
  /// Compensated: equivalents for another topology behind the breaker, same
  /// storage elements. Returns its thevenin_index.
  Int addTheveninEquivalents(const TheveninEquivalent &positive,
                             const TheveninEquivalent &zero);
  /// Compensated: once all poles are open, hand the states of the equivalents
  /// to the network storage elements as space vector and open the network
  /// pole; the zero sequence stays in its equivalent.
  void setNetworkHandover(Bool enabled) { mCompHandover = enabled; }

  /// SpaceVector: zero-sequence network and the port of this switch in it
  void setZeroSequenceNetwork(const std::shared_ptr<ZeroSequenceEquivalent> &network,
                              UInt port) {
    mZeroNetwork = network;
    mZeroPort = port;
  }
  /// SpaceVector: poles that exist; the others stay open, e.g. a
  /// single-phase fault {true, false, false}
  void setPoleMask(const std::array<Bool, 3> &mask) { mPoleMask = mask; }
  /// SpaceVector: an opening interrupts each pole at its current zero
  /// (default) or all poles at once
  void setCurrentZeroOpening(Bool enabled) { mCurrentZeroOpening = enabled; }
  /// SpaceVector: a pole whose current zero is detected interrupts in the same
  /// step (default) instead of the next one
  void setSameStepOpening(Bool enabled) { mSameStepOpening = enabled; }

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
  /// MNA post step operations
  void mnaCompPostStep(Real time, Int timeStepCount,
                       Attribute<Matrix>::Ptr &leftVector) override;
  /// Add MNA post step dependencies
  void
  mnaCompAddPostStepDependencies(AttributeBase::List &prevStepDependencies,
                                 AttributeBase::List &attributeDependencies,
                                 AttributeBase::List &modifiedAttributes,
                                 Attribute<Matrix>::Ptr &leftVector) override;

  // #### MNA section for switch ####
  /// Check if switch is closed
  Bool mnaIsClosed() override;
  /// Stamps system matrix considering the defined switch position
  void mnaCompApplySwitchSystemMatrixStamp(Bool closed,
                                           SparseMatrixRow &systemMatrix,
                                           Int freqIdx) override;

  Bool supportsPrecomputedSystemMatrices() const override;
  /// True for the current-zero mode, which needs the quadrature correction
  Bool requestsQuadratureCorrection() const override;
  Bool takeSwitchingEvent() override;
  std::vector<SwitchedPole> takeSwitchedPoles() override;
  std::vector<StateHandover> takeStateHandover() override;

  // #### MNA section for compensation ####
  Bool mnaIsCompensated() const override {
    return mSwitchingMode == SwitchingMode::SpaceVector;
  }
  std::vector<std::pair<Int, Int>> mnaCompensationTerminals() override;
  MNAZeroSequenceNetwork::Ptr mnaZeroSequenceNetwork() override;
  std::vector<UInt> mnaZeroSequencePorts() override { return {mZeroPort}; }
  void mnaCompensationConditions(Real time, Matrix &a, Matrix &b,
                                 Matrix &c) override;
  Bool mnaCompensationResult(Real time, const Matrix &injection,
                             const Matrix &zeroCurrent,
                             const Matrix &phaseCurrent) override;

  // #### MNA section for variable component ####
  Bool hasParameterChanged() override;

private:
  SwitchingMode mSwitchingMode = SwitchingMode::Ideal;

  Real mZeroCrossingTolerance = 1e-6;
  Real mExponentialSwitchingTime = 0.01;
  Real mTimeStep = 0.0;

  /// Reference-frame angular frequency supplied by the solver [rad/s].
  Real mShiftOmega = 0.0;

  Bool mPoleClosedPrev = false;
  /// Set when a pole actually changed, cleared by takeSwitchingEvent()
  Bool mSwitchingEventPending = false;
  /// Set when the pole opened or closed, cleared by takeSwitchedPoles()
  Bool mSwitchedPolePending = false;
  Real mEffectiveResistancePrev = 0.0;

  Real mPreviousInstantaneousCurrent = 0.0;
  Real mPreviousCurrentTime = 0.0;
  Bool mPreviousCurrentValid = false;

  Bool mResetZeroCrossingHistory = false;
  Bool mExponentialTransitionStarted = false;

  // Compensated: Thevenin equivalents, their states (positive sequence per
  // phase, zero sequence) and the pole states
  using Phases = std::array<Complex, 3>;
  std::vector<TheveninEquivalent> mThevenin1, mThevenin0;
  std::vector<Attribute<MatrixComp>::Ptr> mCompNodeCorrections;
  std::vector<ObservedNode> mCompNodes;
  std::array<Matrix, 3> mCompX1;
  Matrix mCompX0;
  // instantaneous pole currents of the last two steps and the smoothed
  // value of the last step
  std::array<Real, 3> mCompInst1{}, mCompInst2{}, mCompSmooth1{};
  Int mCompInstCount = 0;
  // the last smoothed value is from a step after the opening command
  Bool mCompArmed = false;
  // SpaceVector: injected space vector and zero-sequence current
  Complex mSvCurrent{0., 0.};
  Real mSvZeroCurrent = 0.;
  Bool mSameStepOpening = true;
  Bool mCurrentZeroOpening = true;
  std::array<Bool, 3> mPoleMask{true, true, true};
  std::shared_ptr<ZeroSequenceEquivalent> mZeroNetwork;
  UInt mZeroPort = 0;
  // time of the last solve; a repeated solve in the same step keeps the pole
  // current history
  Real mSvResultTime = -1.;
  // currents of the latest solve, taken into the history in the next step
  std::array<Real, 3> mSvPendingCurrent{};
  Bool mSvArmedStep = false;
  std::array<Bool, 3> mCompOpen{};
  Bool mCompStarted = false;
  Bool mCompHandover = false;
  std::vector<StateHandover> mCompHandoverPending;

  void updateCompensation(Real time);
  /// Instantaneous pole currents, kept from before the opening command on;
  /// returns the smoothed values
  std::array<Real, 3> updatePoleCurrents(const std::array<Real, 3> &inst);
  void handOverToNetwork(const TheveninEquivalent &th,
                         const std::array<Matrix, 3> &xPrev,
                         const std::array<Matrix, 3> &u, Real time);

  void setPoleClosed(Bool closed);

  void resetZeroCrossingTime();

  void synchronizeEffectiveResistance(Bool closed);

  void resetExponentialTransition();
  void validateExponentialResistanceParameters() const;
  Real exponentialResistance(Real alpha) const;
  void updateExponentialTransition(Real time);

  Complex currentConductance() const;

  Real reconstructInstantaneousCurrent(Real time) const;

  Bool currentCrossedZero(Real previousCurrent, Real current) const;

  Real interpolateZeroCrossingTime(Real previousCurrent, Real current,
                                   Real previousTime, Real currentTime) const;

  void updateZeroCrossingState(Real time);
};

/// \brief Zero-sequence network seen from the ports of compensated switches,
/// a time-domain Thevenin equivalent; observed nodes get
/// compensation_v_<name>, the zero-sequence part of their phase voltages.
class ZeroSequenceEquivalent : public IdentifiedObject,
                               public MNAZeroSequenceNetwork,
                               public SharedFactory<ZeroSequenceEquivalent> {
public:
  const Attribute<Int>::Ptr mTheveninIndex;

  ZeroSequenceEquivalent(String name, UInt ports,
                         const Switch::TheveninEquivalent &equivalent,
                         const std::vector<Switch::ObservedNode> &nodes);
  /// A further set; returns its thevenin_index
  Int addEquivalent(const Switch::TheveninEquivalent &equivalent);

  Matrix zeroFreeVoltage() override;
  Matrix zeroVoltageResponse() override;
  void zeroAdvance(const Matrix &input) override;

private:
  UInt mPorts;
  std::vector<Switch::TheveninEquivalent> mSets;
  std::vector<Switch::ObservedNode> mNodes;
  std::vector<Attribute<MatrixComp>::Ptr> mCorrections;
  Matrix mState;
  const Switch::TheveninEquivalent &current() const;
};
} // namespace Ph1
} // namespace DP
} // namespace CPS
