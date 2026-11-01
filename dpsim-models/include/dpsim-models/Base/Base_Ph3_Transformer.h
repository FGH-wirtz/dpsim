/* Copyright 2017-2021 Institute for Automation of Complex Power Systems,
 *                     EONERC, RWTH Aachen University
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at https://mozilla.org/MPL/2.0/.
 *********************************************************************************/

#pragma once

#include <dpsim-models/AttributeList.h>
#include <dpsim-models/Definitions.h>

#include <stdexcept>

namespace CPS {
namespace Base {
namespace Ph3 {
class Transformer {
protected:
  /// Nominal voltage of primary side
  Real mNominalVoltagePrimary;
  /// Nominal voltage of secondary side
  Real mNominalVoltageSecondary;
  /// Rated Apparent Power [VA]
  Real mRatedPower;
  /// Resistance [Ohm]
  Matrix mResistance;
  /// Inductance [H]
  Matrix mInductance;
  /// Winding matrix v0' = N v1 of the ideal transformer, empty: per-phase ratio
  Matrix mWinding;

public:
  /// Nominal voltage of the winding at the given terminal
  Real nominalVoltageAt(UInt terminal) const {
    return terminal == 0 ? mNominalVoltagePrimary : mNominalVoltageSecondary;
  }

  ///Transformer ratio
  const Attribute<Complex>::Ptr mRatio;

  explicit Transformer(CPS::AttributeList::Ptr attributeList)
      : mRatio(attributeList->create<Complex>("ratio")){};

  ///
  void setParameters(Real nomVoltagePrimary, Real nomVoltageSecondary,
                     Real ratedPower, Real ratioAbs, Real ratioPhase,
                     Matrix resistance, Matrix inductance) {
    mNominalVoltagePrimary = nomVoltagePrimary;
    mNominalVoltageSecondary = nomVoltageSecondary;
    mRatedPower = ratedPower;
    **mRatio = std::polar<Real>(ratioAbs, ratioPhase);
    mResistance = resistance;
    mInductance = inductance;
  }

  /// Yd vector group, clock 1, 5, 7 or 11: star on terminal 0 (high-voltage
  /// side), delta on terminal 1, zero sequence blocked. Call after
  /// setParameters().
  void setYdVectorGroup(Int clock) {
    // star winding of phase a across the delta terminals x - y
    Int x, y;
    if (clock == 1) {
      x = 0, y = 1;
    } else if (clock == 5) {
      x = 2, y = 0;
    } else if (clock == 7) {
      x = 1, y = 0;
    } else if (clock == 11) {
      x = 0, y = 2;
    } else {
      throw std::invalid_argument("Yd vector group needs clock 1, 5, 7 or 11.");
    }
    Real k = std::abs(**mRatio) / std::sqrt(3.);
    mWinding = Matrix::Zero(3, 3);
    for (Int p = 0; p < 3; ++p) {
      mWinding(p, (x + p) % 3) = k;
      mWinding(p, (y + p) % 3) = -k;
    }
    **mRatio = std::polar<Real>(std::abs(**mRatio), clock * PI / 6.);
  }
};
} // namespace Ph3
} // namespace Base
} // namespace CPS
