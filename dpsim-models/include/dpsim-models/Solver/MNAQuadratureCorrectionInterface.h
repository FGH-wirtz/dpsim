// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>
#include <type_traits>
#include <vector>

#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
/// \brief Storage states the quadrature correction writes after a switching
/// event.
///
/// The corrector writes states only, and each component rebuilds its own
/// history. A three-phase component contributes one state per phase; the
/// coupling defaults reduce to the uncoupled case.
template <typename VarType> class MNAQuadratureCorrectionInterface {
public:
  typedef std::shared_ptr<MNAQuadratureCorrectionInterface<VarType>> Ptr;
  typedef std::vector<Ptr> List;

  /// Quantity the correction writes
  enum class StateKind { InductorCurrent, CapacitorVoltage };

  virtual ~MNAQuadratureCorrectionInterface() = default;

  /// Number of correctable states, one per phase
  virtual UInt mnaCorrectionStateCount() { return 1; }
  /// Quantity this component contributes
  virtual StateKind mnaCorrectionStateKind() = 0;
  /// Companion admittance as stamped, single frequency
  virtual VarType mnaCorrectionAdmittance(UInt index) = 0;
  /// Branch current for an inductor, terminal voltage for a capacitor
  virtual VarType mnaCorrectionState(UInt index) = 0;
  /// Called by the corrector only, after an event
  virtual void mnaCorrectionSetState(UInt index, VarType value) = 0;
  /// Inductance or capacitance
  virtual Real mnaCorrectionElementValue(UInt index) = 0;
  /// Companion admittance from the terminal voltage of state `col` to state
  /// `row`, as stamped
  virtual VarType mnaCorrectionCouplingAdmittance(UInt row, UInt col) {
    return row == col ? mnaCorrectionAdmittance(row) : VarType(0);
  }
  /// Entry (row, col) of the inductance or capacitance matrix
  virtual Real mnaCorrectionCouplingElement(UInt row, UInt col) {
    return row == col ? mnaCorrectionElementValue(row) : 0.0;
  }
  /// Matrix node indices of both terminals, negative meaning grounded
  virtual void mnaCorrectionNodes(UInt index, Int &node0, Int &node1) = 0;
  /// Recompute the right-hand-side contribution from the present states
  virtual void mnaCorrectionRebuildRightVector() = 0;
  /// Entry (row, col) of the admittance at the shift frequency, stamped in place
  /// of the companion admittance. (j w L)^-1 or j w C by default, zero for a
  /// current-controlled state.
  virtual VarType mnaCorrectionSteadyStateAdmittance(UInt row, UInt col,
                                                     Real omega) {
    if (mCorrectionCurrentControlled)
      return VarType(0);

    if constexpr (std::is_same<VarType, Complex>::value) {
      const UInt count = mnaCorrectionStateCount();
      MatrixComp element(count, count);
      for (UInt r = 0; r < count; ++r)
        for (UInt c = 0; c < count; ++c)
          element(r, c) = Complex(mnaCorrectionCouplingElement(r, c), 0.0);

      if (mnaCorrectionStateKind() == StateKind::CapacitorVoltage)
        return Complex(0.0, omega) * element(row, col);

      const MatrixComp admittance = (Complex(0.0, omega) * element).inverse();
      return admittance(row, col);
    } else {
      return VarType(0);
    }
  }
  /// False for a current-controlled state
  virtual Bool mnaCorrectionAllowed() { return !mCorrectionCurrentControlled; }
  /// Marks the state as held by a controller, such as a converter's filter
  /// current. It then enters the solve as a current source and is not
  /// corrected.
  void mnaCorrectionSetCurrentControlled(Bool controlled) {
    mCorrectionCurrentControlled = controlled;
  }

protected:
  Bool mCorrectionCurrentControlled = false;
};
} // namespace CPS
