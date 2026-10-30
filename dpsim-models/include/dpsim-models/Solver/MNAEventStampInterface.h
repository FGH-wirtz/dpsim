// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

#pragma once

#include <memory>
#include <vector>

#include <dpsim-models/Config.h>
#include <dpsim-models/Definitions.h>

namespace CPS {
/// \brief Stamp for the quadrature correction's network solve, for a component
/// without correctable states.
///
/// A component that injects a current computed from internal states, such as a
/// full order machine, stamps nothing and would look like an open circuit. It
/// adds the admittance of the inductance its current follows instead.
template <typename VarType> class MNAEventStampInterface {
public:
  typedef std::shared_ptr<MNAEventStampInterface<VarType>> Ptr;
  typedef std::vector<Ptr> List;

  virtual ~MNAEventStampInterface() = default;

  /// Adds the difference between the component's step stamp and its admittance
  /// at the shift frequency `omega`
  virtual void mnaApplyEventStamp(SparseMatrixRow &matrix, Real omega) = 0;
};
} // namespace CPS
