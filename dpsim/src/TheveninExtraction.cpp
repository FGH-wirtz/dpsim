/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <algorithm>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <stdexcept>
#include <tuple>

#include <dpsim-models/CompositePowerComp.h>
#include <dpsim-models/DP/DP_Ph1_Capacitor.h>
#include <dpsim-models/DP/DP_Ph1_Inductor.h>
#include <dpsim-models/DP/DP_Ph1_Transformer.h>
#include <dpsim/Simulation.h>
#include <dpsim/TheveninExtraction.h>

using namespace CPS;

namespace DPsim {

DP::Ph1::Switch::TheveninEquivalent
extractTheveninEquivalent(SystemTopology &system, Real timeStep,
                          const SimNode<Complex>::Ptr &node0,
                          const SimNode<Complex>::Ptr &node1,
                          const std::vector<SimNode<Complex>::Ptr> &observed,
                          UInt subSteps) {
  const UInt m = std::max<UInt>(subSteps, 1);
  const Real h = timeStep / m;
  Simulation sim("TheveninExtraction", Logger::Level::off);
  sim.setSystem(system);
  sim.setTimeStep(h);
  sim.setFinalTime(h);
  sim.setDomain(Domain::DP);
  sim.doStateSpaceExtraction(true);
  sim.start();

  // unit real and imaginary current into node1, out of node0
  auto &extractor = sim.getStateSpaceExtractor();
  const Matrix::Index n = extractor.getMnaInputMatrix().cols();
  Matrix injection = Matrix::Zero(n, 2);
  for (int c = 0; c < 2; ++c) {
    Matrix col = Matrix::Zero(n, 1);
    const Complex i = c == 0 ? Complex(1., 0.) : Complex(0., 1.);
    Math::addToVectorElement(col, node1->matrixNodeIndex(), i);
    Math::addToVectorElement(col, node0->matrixNodeIndex(), -i);
    injection.col(c) = col;
  }
  extractor.setInjection(injection);
  sim.step();
  sim.stop();

  // outputs: v0 - v1, then the observed node voltages, (re, im) each
  Matrix select = Matrix::Zero(2 + 2 * observed.size(), n);
  auto add = [&](Matrix::Index row, const SimNode<Complex>::Ptr &node,
                 Real sign) {
    select(row, node->matrixNodeIndex()) += sign;
    select(row + 1, node->matrixNodeIndex() + n / 2) += sign;
  };
  add(0, node0, 1.);
  add(0, node1, -1.);
  for (std::size_t i = 0; i < observed.size(); ++i)
    add(2 + 2 * i, observed[i], 1.);

  DP::Ph1::Switch::TheveninEquivalent eq;
  eq.A = extractor.getDiscreteStateMatrix();
  eq.B = extractor.getMnaInputMatrix() * extractor.getMnaInjectionSolution();
  eq.C = select * extractor.getMnaStateSolution();
  eq.D = select * extractor.getMnaInjectionSolution();
  eq.stateNames = extractor.getMetadata().stateNames;

  // sub-steps i = 0..m-1 with u held: y_i = C A^i x + (C S_i B + D) u,
  // S_i = sum_{j<i} A^j; after the step x = A^m x + S_m B u
  const Matrix::Index ns = eq.A.rows();
  Matrix last = Matrix::Identity(ns, ns), lastSum = Matrix::Zero(ns, ns);
  for (UInt i = 0; i + 1 < m; ++i) {
    lastSum += last;
    last = eq.A * last;
  }

  // storage elements at the end of the step, from the last sub-step:
  // v = K xMNA, i = Y v + h with h the state before that sub-step
  const Matrix mnaX = extractor.getMnaStateSolution() * last;
  const Matrix mnaU =
      extractor.getMnaStateSolution() * lastSum * eq.B +
      extractor.getMnaInjectionSolution();
  const Matrix hU = lastSum * eq.B;
  std::vector<Matrix> exRows, euRows;
  auto addElement = [&](auto &element, Complex y) {
    const String name = element.name();
    auto index = [&](const String &suffix) {
      auto it = std::find(eq.stateNames.begin(), eq.stateNames.end(),
                          name + suffix);
      if (it == eq.stateNames.end())
        throw std::runtime_error("extractTheveninEquivalent: no state " +
                                 name + suffix);
      return static_cast<Matrix::Index>(it - eq.stateNames.begin());
    };
    const Matrix::Index re = index("_re"), im = index("_im");
    Matrix k = Matrix::Zero(2, n);
    for (UInt t = 0; t < 2; ++t) {
      if (!element.terminalNotGrounded(t))
        continue;
      const Real sign = t == 1 ? 1. : -1.;
      k(0, element.matrixNodeIndex(t)) += sign;
      k(1, element.matrixNodeIndex(t) + n / 2) += sign;
    }
    Matrix yAug(2, 2);
    yAug << y.real(), -y.imag(), y.imag(), y.real();
    Matrix ex(4, ns), eu(4, 2);
    ex.topRows(2) = k * mnaX;
    eu.topRows(2) = k * mnaU;
    ex.bottomRows(2) = yAug * ex.topRows(2);
    eu.bottomRows(2) = yAug * eu.topRows(2);
    ex.row(2) += last.row(re);
    ex.row(3) += last.row(im);
    eu.row(2) += hU.row(re);
    eu.row(3) += hU.row(im);
    eq.elements.push_back({name, element.node(0)->name(),
                           element.node(1)->name(),
                           element.mnaCorrectionElementValue(0)});
    exRows.push_back(ex);
    euRows.push_back(eu);
  };
  // node graph for the phase shift of each element: two-terminal leaves
  // without shift, a transformer from its inner node to terminal 1 with
  // -arg(ratio)
  std::vector<std::tuple<String, String, Real>> edges;
  std::function<void(const MNAInterface::Ptr &)> collect =
      [&](const MNAInterface::Ptr &comp) {
        if (auto l = std::dynamic_pointer_cast<DP::Ph1::Inductor>(comp))
          addElement(*l, l->getMNAConductance());
        else if (auto c = std::dynamic_pointer_cast<DP::Ph1::Capacitor>(comp))
          addElement(*c, c->getMNAConductance());
        if (auto trafo = std::dynamic_pointer_cast<DP::Ph1::Transformer>(comp))
          edges.emplace_back(trafo->virtualNode(0)->name(),
                             trafo->node(1)->name(),
                             -std::arg(trafo->attributeTyped<Complex>("ratio")
                                           ->get()));
        if (auto composite =
                std::dynamic_pointer_cast<CompositePowerComp<Complex>>(comp)) {
          for (auto sub : composite->mnaSubComponents())
            collect(sub);
        } else if (auto leaf =
                       std::dynamic_pointer_cast<SimPowerComp<Complex>>(comp)) {
          if (leaf->terminalNumber() == 2 && leaf->terminalNotGrounded(0) &&
              leaf->terminalNotGrounded(1))
            edges.emplace_back(leaf->node(0)->name(), leaf->node(1)->name(),
                               0.);
        }
      };
  for (auto comp : system.mComponents)
    collect(std::dynamic_pointer_cast<MNAInterface>(comp));
  std::map<String, Real> shift{{node0->name(), 0.}};
  std::deque<String> queue{node0->name()};
  while (!queue.empty()) {
    const String at = queue.front();
    queue.pop_front();
    for (auto &[a, b, phi] : edges) {
      if (a != at && b != at)
        continue;
      const String next = a == at ? b : a;
      const Real value = shift[at] + (a == at ? phi : -phi);
      auto it = shift.find(next);
      if (it == shift.end()) {
        shift[next] = value;
        queue.push_back(next);
      } else if (std::abs(std::remainder(it->second - value, 2. * PI)) >
                 1e-9) {
        throw std::runtime_error("extractTheveninEquivalent: node " + next +
                                 " has two phase shifts.");
      }
    }
  }
  for (auto &element : eq.elements) {
    auto it = shift.find(element.node0);
    if (it == shift.end())
      it = shift.find(element.node1);
    if (it != shift.end())
      element.phaseShift = it->second;
  }
  eq.Ex = Matrix::Zero(4 * eq.elements.size(), ns);
  eq.Eu = Matrix::Zero(4 * eq.elements.size(), 2);
  for (std::size_t e = 0; e < eq.elements.size(); ++e) {
    eq.Ex.middleRows(4 * e, 4) = exRows[e];
    eq.Eu.middleRows(4 * e, 4) = euRows[e];
  }
  if (m == 1)
    return eq;

  Matrix power = Matrix::Identity(ns, ns), sum = Matrix::Zero(ns, ns);
  Matrix cBar = Matrix::Zero(eq.C.rows(), ns);
  Matrix dBar = Matrix::Zero(eq.D.rows(), eq.D.cols());
  for (UInt i = 0; i < m; ++i) {
    cBar += eq.C * power;
    dBar += eq.C * sum * eq.B + eq.D;
    sum += power;
    power = eq.A * power;
  }
  DP::Ph1::Switch::TheveninEquivalent agg;
  agg.A = power;
  agg.B = sum * eq.B;
  agg.C = cBar / m;
  agg.D = dBar / m;
  agg.stateNames = eq.stateNames;
  agg.elements = eq.elements;
  agg.Ex = eq.Ex;
  agg.Eu = eq.Eu;
  return agg;
}

} // namespace DPsim
