/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#pragma once

#include <vector>

#include <dpsim-models/DP/DP_Ph1_Switch.h>
#include <dpsim-models/SimNode.h>
#include <dpsim-models/SystemTopology.h>
#include <dpsim/Definitions.h>

namespace DPsim {

/// Time-domain Thevenin equivalent of a passive DP Ph1 network between node0
/// and node1, observed node voltages as further outputs. With subSteps m > 1
/// extracted at timeStep / m, outputs averaged over the step.
CPS::DP::Ph1::Switch::TheveninEquivalent
extractTheveninEquivalent(CPS::SystemTopology &system, Real timeStep,
                          const CPS::SimNode<Complex>::Ptr &node0,
                          const CPS::SimNode<Complex>::Ptr &node1,
                          const std::vector<CPS::SimNode<Complex>::Ptr> &observed,
                          UInt subSteps = 1);

/// The same with several ports (node0, node1), a null node meaning ground:
/// inputs (re, im) of the current into node1 per port, outputs (re, im) of
/// v0 - v1 per port, then of each observed node voltage. averageOutputs
/// false: outputs at the end of the step instead of averaged over the
/// sub-steps.
CPS::DP::Ph1::Switch::TheveninEquivalent extractTheveninEquivalent(
    CPS::SystemTopology &system, Real timeStep,
    const std::vector<std::pair<CPS::SimNode<Complex>::Ptr,
                                CPS::SimNode<Complex>::Ptr>> &ports,
    const std::vector<CPS::SimNode<Complex>::Ptr> &observed,
    UInt subSteps = 1, Bool averageOutputs = true);

} // namespace DPsim
