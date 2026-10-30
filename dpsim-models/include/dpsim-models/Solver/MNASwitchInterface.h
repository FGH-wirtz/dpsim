/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <vector>

#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
/// \brief MNA interface to be used by switching devices.
class MNASwitchInterface {
public:
  typedef std::shared_ptr<MNASwitchInterface> Ptr;
  typedef std::vector<Ptr> List;

  virtual ~MNASwitchInterface() = default;

  /// Whether all system-matrix states of the switch can be represented by
  /// precomputed switch matrices.
  /// \note This is expected to be true for implementations of
  /// MNASwitchInterface. The capability is required because some existing
  /// switch components also vary matrix parameters continuously between their
  /// discrete switch states.
  virtual Bool supportsPrecomputedSystemMatrices() const { return true; }

  /// Whether a quadrature correction should follow a state change of this
  /// switch. Ideal switching keeps its previous behaviour, so this defaults to
  /// false.
  virtual Bool requestsQuadratureCorrection() const { return false; }

  /// Whether this switch has opened or closed a pole since this was last asked,
  /// and so wants a correction now. Reading it clears the record, because a
  /// change is corrected once. A switch reports its own events rather than the
  /// solver deducing them, because mnaIsClosed() on a three-phase switch is an
  /// aggregate and stops changing after the first pole. Defaults to false, so a
  /// switch that opts into the correction has to report.
  virtual Bool takeSwitchingEvent() { return false; }

  /// A pole that has just opened or closed, as the switch saw it on the last
  /// step before the event. An opening pole carries the current it interrupted,
  /// which at a current zero is an envelope with no instantaneous value, so the
  /// corrector can remove it from the states that carried it without changing
  /// the physical current. A closing pole carries the voltage it shorts, for
  /// the same reason.
  struct SwitchedPole {
    /// Matrix node indices of the pole's terminals, negative meaning grounded
    Int node0 = -1;
    Int node1 = -1;
    Real closedConductance = 0.0;
    Real openConductance = 0.0;
    /// True if the pole closed, false if it opened
    Bool closed = false;
    /// Branch current G * (v1 - v0) before the event
    Complex current = Complex(0.0, 0.0);
    /// Pole voltage v1 - v0 before the event
    Complex voltage = Complex(0.0, 0.0);
    /// Interpolated current zero of an opening pole, negative if unknown
    Real zeroTime = -1.0;
  };

  /// Poles opened or closed since this was last asked. Reading it clears the
  /// record, like takeSwitchingEvent(). Defaults to none.
  virtual std::vector<SwitchedPole> takeSwitchedPoles() { return {}; }

  // #### MNA section ####
  /// Check if switch is closed
  virtual Bool mnaIsClosed() = 0;
  /// Stamps system matrix considering the defined switch position
  virtual void mnaApplySwitchSystemMatrixStamp(Bool closed,
                                               SparseMatrixRow &systemMatrix,
                                               Int freqIdx) final {
    this->mnaCompApplySwitchSystemMatrixStamp(closed, systemMatrix, freqIdx);
    systemMatrix.makeCompressed();
  }
  virtual void mnaCompApplySwitchSystemMatrixStamp(
      Bool closed, SparseMatrixRow &systemMatrix, Int freqIdx) {}
};
} // namespace CPS
