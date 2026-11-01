/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <memory>
#include <utility>
#include <vector>

#include <dpsim-models/Definitions.h>

namespace CPS {
/// \brief Zero-sequence network seen from compensated ports, as a discrete
/// Thevenin equivalent shared by the components whose ports it connects.
class MNAZeroSequenceNetwork {
public:
  typedef std::shared_ptr<MNAZeroSequenceNetwork> Ptr;

  virtual ~MNAZeroSequenceNetwork() = default;

  /// Port voltages v1 - v0, (re, im) per port, without input in this step
  virtual Matrix zeroFreeVoltage() = 0;
  /// Map from the port inputs, (re, im) of the current into node1 per port,
  /// to the port voltages
  virtual Matrix zeroVoltageResponse() = 0;
  /// Advance by one step with the port inputs
  virtual void zeroAdvance(const Matrix &input) = 0;
};

/// \brief Component the MNA solver compensates (Dommel compensation).
///
/// Its ports stay open in the system matrix. Per step the solver fixes the
/// space-vector current I and the zero-sequence current i0 of all ports
/// jointly from one condition per phase on instantaneous values and adds the
/// network response. Phase p carries Re{a^-p I e^(j w t)} + i0. DP, single
/// frequency.
class MNACompensatedInterface {
public:
  typedef std::shared_ptr<MNACompensatedInterface> Ptr;
  typedef std::vector<Ptr> List;

  virtual ~MNACompensatedInterface() = default;

  /// Whether the solver compensates this component, fixed before the start
  virtual Bool mnaIsCompensated() const { return true; }

  /// Matrix node indices (node0, node1) of each port, negative for ground;
  /// the injection flows into node1 and out of node0
  virtual std::vector<std::pair<Int, Int>> mnaCompensationTerminals() = 0;

  /// Zero-sequence network of the ports and each port's index in it
  virtual MNAZeroSequenceNetwork::Ptr mnaZeroSequenceNetwork() = 0;
  virtual std::vector<UInt> mnaZeroSequencePorts() = 0;

  /// Per port and phase, 3 rows per port: a v + b i = c on the instantaneous
  /// phase voltage v1 - v0 and phase current into node1
  virtual void mnaCompensationConditions(Real time, Matrix &a, Matrix &b,
                                         Matrix &c) = 0;

  /// The solution: (re, im) of I per port, i0 per port, the instantaneous
  /// phase currents (3 per port). Called again in the same step if a
  /// component returns true because its conditions changed.
  virtual Bool mnaCompensationResult(Real time, const Matrix &injection,
                                     const Matrix &zeroCurrent,
                                     const Matrix &phaseCurrent) = 0;
};
} // namespace CPS
