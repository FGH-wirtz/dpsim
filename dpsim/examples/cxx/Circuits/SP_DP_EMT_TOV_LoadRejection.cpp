/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <DPsim.h>
#include <dpsim/TheveninExtraction.h>

using namespace DPsim;
using namespace CPS;

// Load rejection by fault clearing in a cable-rich 110 kV grid (Testfall 1).
// 380 kV grid - coupling transformer - B1 (load, MSC, cable equivalent to B5
// with resistive load, cable K1 to B3) - breaker L1 - overhead line L1 - B2
// (load, fault). B3 - plant transformer 110/20 kV - B4 - two WECC WT4B
// parks (HVRT-capable and legacy).
// Three-phase fault at B2, L1 opens at current zero, the fault extinguishes,
// L1 recloses. All domains are initialized from the same power flow.
struct LineData {
  Real resistance;
  Real inductance;
  Real capacitance;
};

struct Params {
  Real frequency = 50;
  Real omega = 2. * PI * 50;
  Real timeStep = 50e-6;
  Real finalTime = 1.7;

  Real ehvVoltage = 380e3;
  Real hvVoltage = 110e3;
  Real mvVoltage = 20e3;

  // grid equivalent, merged into the coupling transformer (380 kV side)
  Real gridShortCircuitPower = 5e9;
  Real gridRX = 0.1;
  Real couplingPower = 200e6;
  Real couplingUk = 0.15;
  Real couplingXR = 40;
  // 380 kV side tap, sets B1 to about 1.04 pu before the fault
  Real couplingTap = 1.045;

  // per km: R [Ohm], X [Ohm], C [F]
  Real l1Length = 40;
  Real l1R = 0.12, l1X = 0.4, l1C = 9e-9;
  Real k1Length = 30;
  Real cableEqLength = 105;
  Real cableR = 0.03, cableX = 0.12, cableC = 0.25e-6;

  Real loadB1P = 50e6;
  Real loadB2P = 150e6, loadB2Q = 50e6;
  Real loadB5P = 30e6;
  Real mscQ = 50e6;

  Real plantTrafoPower = 120e6;
  // Yd5, 110 kV neutral open: positive-sequence ratio angle
  Real plantShift = 5. * PI / 6.;
  Real plantTrafoUk = 0.12;
  Real plantTrafoXR = 30;

  // two parks at B4: HVRT-capable (VDE-AR-N 4120) and legacy without HVRT
  Real wtActivePower = 70e6;
  Real wtLegacyActivePower = 30e6;
  Real wtReactivePower = 0;
  Real wtRatingFactor = 1.1;
  Bool wtProtection = true;
  Int wtMeasurement = 0;
  // 0 WT4B units, 1 none, 2 constant current at the power-flow injection
  Int wtMode = 0;
  // DP1: compensated breaker, Thevenin equivalents across the open breaker
  Bool compensated = false;
  // breaker current into the DP1 network, zero sequence via an equivalent
  Bool compSpaceVectorMode = false;
  Bool compLog = true;
  // a pole interrupts in the step its current zero is detected
  Bool compSameStep = true;
  Bool compSpaceVector = true;
  // extraction step of the equivalents, averaged to the time step
  Real compStep = 10e-6;
  // equivalents without the fault after the fault ends
  Bool compTopology = true;
  // states go to the DP1 network once all poles are open
  Bool compHandover = true;
  // Ph1: zero-sequence grid (for the Thevenin equivalent)
  Bool zeroSequence = false;
  // three-phase fault before the opening (false: L1 opens under load)
  Bool withFault = true;
  // DP3/EMT neutral grounding: 0 decoupled phases (Z0 = Z1), 1 solid,
  // 2 resonant (Petersen coil at the coupling transformer star point)
  Int grounding = 0;
  Bool quadratureCorrection = true;
  Bool zeroCrossingInterpolation = false;
  Real kpPLL = 0.25;
  Real kiPLL = 2;
  Real omegaCutoffPLL = 2. * PI * 50;
  // WT shunt (x zBase): damps the transformer LV snubber resonance;
  // compensated, so no steady-state effect
  Real wtShuntFactor = 1e3;

  Real faultResistance = 1;
  // faulted phases at B2: 3 (a, b, c), 1 (a to ground), 2 (b, c to ground),
  // 4 (b to c, no ground; Ph3 only)
  Int faultType = 3;
  // double earth fault: phase b to ground at B3 as well
  Bool secondFault = false;
  // negative-sequence reactive current, VDE-AR-N 4120 (0: off)
  Real kNegative = 0.;
  Real kNegativeLegacy = 0.;
  Real kNegativeTauMeas = 0.005;
  // EMT: sequence current into B4 for impedance scans (rms per phase; 0: off)
  Real probeCurrent = 0.;
  Real probeFrequency = 0.;
  Bool probeNegative = true;
  std::array<Bool, 3> faultMask() const {
    return faultType == 1   ? std::array<Bool, 3>{true, false, false}
           : faultType == 2 ? std::array<Bool, 3>{false, true, true}
                            : std::array<Bool, 3>{true, true, true};
  }
  Real breakerResistance = 0.01;
  Real switchOpen = 1e9;

  // after 0.5 s, when the WT ride-through protection is active
  Real faultOn = 0.60;
  Real breakerOpen = 0.75;
  // opening instant for SP and ideal breakers
  Real breakerOpenIdeal = 0.7594;
  Real faultOff = 0.95;
  Real reclose = 1.16;
  // second opening after the reclose, off if negative
  Real reopen = -1;

  Real couplingResistance() const {
    Real xGrid = ehvVoltage * ehvVoltage / gridShortCircuitPower /
                 std::sqrt(1 + gridRX * gridRX);
    Real xTrafo = couplingUk * ehvVoltage * ehvVoltage / couplingPower;
    return gridRX * xGrid + xTrafo / couplingXR;
  }
  Real couplingInductance() const {
    Real xGrid = ehvVoltage * ehvVoltage / gridShortCircuitPower /
                 std::sqrt(1 + gridRX * gridRX);
    Real xTrafo = couplingUk * ehvVoltage * ehvVoltage / couplingPower;
    return (xGrid + xTrafo) / omega;
  }
  Real plantTrafoResistance() const {
    return plantTrafoUk * hvVoltage * hvVoltage / plantTrafoPower /
           plantTrafoXR;
  }
  Real plantTrafoInductance() const {
    return plantTrafoUk * hvVoltage * hvVoltage / plantTrafoPower / omega;
  }
  LineData line(Real length, Real r, Real x, Real c) const {
    return {length * r, length * x / omega, length * c};
  }
  LineData l1() const { return line(l1Length, l1R, l1X, l1C); }
  LineData k1() const { return line(k1Length, cableR, cableX, cableC); }
  LineData cableEq() const {
    return line(cableEqLength, cableR, cableX, cableC);
  }
};

struct SP1 {
  using Node = SimNode<Complex>;
  using Slack = SP::Ph1::NetworkInjection;
  using Line = SP::Ph1::PiLine;
  using Trafo = SP::Ph1::Transformer;
  using Res = SP::Ph1::Resistor;
  using Ind = SP::Ph1::Inductor;
  using Cap = SP::Ph1::Capacitor;
  using Sw = SP::Ph1::Switch;
  using Isrc = void;
  using WT = SP::Ph1::WECC_WT4B;
  using Event = SwitchEvent;
  static constexpr PhaseType phase = PhaseType::Single;
  static constexpr Domain domain = Domain::SP;
  static constexpr bool singlePhase = true;
  static constexpr bool switchModes = false;
  static constexpr const char *tag = "SP";
  static Real par(Real x) { return x; }
};

struct DP1 {
  using Node = SimNode<Complex>;
  using Slack = DP::Ph1::NetworkInjection;
  using Line = DP::Ph1::PiLine;
  using Trafo = DP::Ph1::Transformer;
  using Res = DP::Ph1::Resistor;
  using Ind = DP::Ph1::Inductor;
  using Cap = DP::Ph1::Capacitor;
  using Sw = DP::Ph1::Switch;
  using Isrc = DP::Ph1::CurrentSource;
  using WT = DP::Ph1::WECC_WT4B;
  using Event = SwitchEvent;
  static constexpr PhaseType phase = PhaseType::Single;
  static constexpr Domain domain = Domain::DP;
  static constexpr bool singlePhase = true;
  static constexpr bool switchModes = true;
  static constexpr const char *tag = "DP1";
  static Real par(Real x) { return x; }
};

struct DP3 {
  using Node = SimNode<Complex>;
  using Slack = DP::Ph3::NetworkInjection;
  using Line = DP::Ph3::PiLine;
  using Trafo = DP::Ph3::Transformer;
  using Res = DP::Ph3::Resistor;
  using Ind = DP::Ph3::Inductor;
  using Cap = DP::Ph3::Capacitor;
  using Sw = DP::Ph3::Switch;
  using Isrc = DP::Ph3::CurrentSource;
  using WT = DP::Ph3::WECC_WT4B;
  using Event = SwitchEvent3Ph;
  static constexpr PhaseType phase = PhaseType::ABC;
  static constexpr Domain domain = Domain::DP;
  static constexpr bool singlePhase = false;
  static constexpr bool switchModes = true;
  static constexpr const char *tag = "DP3";
  static Matrix par(Real x) { return Math::singlePhaseParameterToThreePhase(x); }
};

struct EMT3 {
  using Node = SimNode<Real>;
  using Slack = EMT::Ph3::NetworkInjection;
  using Line = EMT::Ph3::PiLine;
  using Trafo = EMT::Ph3::Transformer;
  using Res = EMT::Ph3::Resistor;
  using Ind = EMT::Ph3::Inductor;
  using Cap = EMT::Ph3::Capacitor;
  using Sw = EMT::Ph3::Switch;
  using Isrc = EMT::Ph3::CurrentSource;
  using WT = EMT::Ph3::WECC_WT4B;
  using Event = SwitchEvent3Ph;
  static constexpr PhaseType phase = PhaseType::ABC;
  static constexpr Domain domain = Domain::EMT;
  static constexpr bool singlePhase = false;
  static constexpr bool switchModes = true;
  static constexpr const char *tag = "EMT";
  static Matrix par(Real x) { return Math::singlePhaseParameterToThreePhase(x); }
};

SystemTopology powerFlow(const Params &p, const String &simName) {
  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto mk = [](const String &n) {
    return SimNode<Complex>::make(n, PhaseType::Single);
  };
  auto n0 = mk("N0"), b1 = mk("B1"), b2 = mk("B2"), b3 = mk("B3"),
       b4 = mk("B4"), b5 = mk("B5");
  SystemComponentList comps;

  auto slack = SP::Ph1::NetworkInjection::make("Slack");
  slack->setParameters(p.ehvVoltage);
  slack->setBaseVoltage(p.ehvVoltage);
  slack->modifyPowerFlowBusType(PowerflowBusType::VD);
  slack->connect({n0});
  comps.push_back(slack);

  auto coupling = SP::Ph1::Transformer::make("Coupling");
  coupling->setParameters(p.ehvVoltage, p.hvVoltage,
                          p.couplingTap * p.ehvVoltage / p.hvVoltage, 0,
                          p.couplingResistance(), p.couplingInductance());
  coupling->setBaseVoltage(p.ehvVoltage);
  coupling->connect({n0, b1});
  comps.push_back(coupling);

  auto addLine = [&](const String &name, const LineData &d,
                     SimNode<Complex>::Ptr from, SimNode<Complex>::Ptr to) {
    auto line = SP::Ph1::PiLine::make(name);
    line->setParameters(d.resistance, d.inductance, d.capacitance);
    line->setBaseVoltage(p.hvVoltage);
    line->connect({from, to});
    comps.push_back(line);
  };
  addLine("L1", p.l1(), b1, b2);
  addLine("K1", p.k1(), b1, b3);
  addLine("CableEq", p.cableEq(), b1, b5);

  // constant-impedance loads and the MSC as shunt admittances at 110 kV
  auto addShunt = [&](const String &name, Real P, Real Q,
                      SimNode<Complex>::Ptr node) {
    auto shunt = SP::Ph1::Shunt::make(name);
    shunt->setParameters(P / (p.hvVoltage * p.hvVoltage),
                         -Q / (p.hvVoltage * p.hvVoltage));
    shunt->setBaseVoltage(p.hvVoltage);
    shunt->connect({node});
    comps.push_back(shunt);
  };
  addShunt("LoadB1", p.loadB1P, 0, b1);
  addShunt("MSC", 0, -p.mscQ, b1);
  addShunt("LoadB2", p.loadB2P, p.loadB2Q, b2);
  addShunt("LoadB5", p.loadB5P, 0, b5);

  auto plantTrafo = SP::Ph1::Transformer::make("PlantTrafo");
  plantTrafo->setParameters(p.hvVoltage, p.mvVoltage, p.hvVoltage / p.mvVoltage,
                            0, p.plantTrafoResistance(),
                            p.plantTrafoInductance());
  plantTrafo->setBaseVoltage(p.hvVoltage);
  plantTrafo->connect({b3, b4});
  comps.push_back(plantTrafo);

  auto wt = SP::Ph1::Load::make("WT");
  wt->setParameters(-(p.wtActivePower + p.wtLegacyActivePower),
                    -p.wtReactivePower, p.mvVoltage);
  wt->modifyPowerFlowBusType(PowerflowBusType::PQ);
  wt->connect({b4});
  comps.push_back(wt);

  auto system = SystemTopology(p.frequency,
                               SystemNodeList{n0, b1, b2, b3, b4, b5}, comps);

  auto logger = DataLogger::make(simNamePF);
  for (auto node : {b1, b2, b3, b4, b5})
    logger->logAttribute(node->name(), node->attribute("v"));

  Simulation sim(simNamePF);
  sim.setSystem(system);
  sim.setTimeStep(p.finalTime);
  sim.setFinalTime(2 * p.finalTime);
  sim.setDomain(Domain::SP);
  sim.setSolverType(Solver::Type::NRP);
  sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  sim.doInitFromNodesAndTerminals(false);
  sim.addLogger(logger);
  sim.run();
  return system;
}

template <typename T> struct Grid {
  typename T::Node::Ptr n0, b1, b1l, b2, b3, b4, b5;
  SystemComponentList comps;
  std::shared_ptr<typename T::Line> l1, k1;
  std::shared_ptr<typename T::Sw> fault, fault2;
};

// passive grid without breaker and WT units; Ph1: positive or zero sequence
template <typename T> Grid<T> buildGrid(const Params &p, Bool faultClosed) {
  auto mk = [](const String &n) { return T::Node::make(n, T::phase); };
  auto n0 = mk("N0"), b1 = mk("B1"), b1l = mk("B1L"), b2 = mk("B2"),
       b3 = mk("B3"), b4 = mk("B4"), b5 = mk("B5");
  auto gnd = T::Node::GND;
  Grid<T> g;
  auto &comps = g.comps;

  auto slack = T::Slack::make("Slack");
  if constexpr (T::singlePhase)
    slack->setParameters(Complex(p.ehvVoltage, 0));
  slack->connect({n0});
  comps.push_back(slack);

  // phase matrix from sequence values, Ph1 takes one of them;
  // own: zero sequence set by the element (Yd), also when decoupled
  auto seq = [&](Real x1, Real x0, Bool own = false) {
    if (p.grounding == 0 && !own)
      x0 = x1;
    if constexpr (T::singlePhase) {
      return p.zeroSequence ? x0 : x1;
    } else {
      Matrix m = Matrix::Constant(3, 3, (x0 - x1) / 3.);
      m.diagonal().setConstant((x0 + 2. * x1) / 3.);
      return m;
    }
  };
  // zero sequence: line R0/R1 2.5, X0/X1 3, C0/C1 0.6; cable R0/R1 4;
  // none through loads (Dyn) and the plant transformer star point
  const Real open0 = 1e4;
  auto addTrafo = [&](const String &trafoName, Real v1, Real v2, Real power,
                      Real ratio, Real r, Real l, Real r0, Real l0,
                      typename T::Node::Ptr hv, typename T::Node::Ptr lv,
                      Bool own = false) {
    auto trafo = T::Trafo::make(trafoName, trafoName, Logger::Level::off, true);
    trafo->setParameters(v1, v2, power, ratio, 0, seq(r, r0, own),
                         seq(l, l0, own));
    trafo->connect({hv, lv});
    comps.push_back(trafo);
    return trafo;
  };
  Real ratioC = p.couplingTap * p.ehvVoltage / p.hvVoltage;
  Real c0 = 0.6 * p.l1().capacitance + p.k1().capacitance +
            p.cableEq().capacitance;
  // Petersen coil, 5 % overcompensated, 5 % loss; 3 Zn referred to 380 kV
  Real ln = 1. / (3. * p.omega * p.omega * c0) / 1.05;
  Real rn = 0.05 * p.omega * ln;
  Real kn = p.grounding == 2 ? 3. * ratioC * ratioC : 0.;
  addTrafo("Coupling", p.ehvVoltage, p.hvVoltage, p.couplingPower, ratioC,
           p.couplingResistance(), p.couplingInductance(),
           p.couplingResistance() + kn * rn,
           p.couplingInductance() + kn * ln, n0, b1);
  // Yd5: 20 kV lags by 150 deg; Ph1 takes the shift as complex ratio
  auto plantTrafo = addTrafo(
      "PlantTrafo", p.hvVoltage, p.mvVoltage, p.plantTrafoPower,
      p.hvVoltage / p.mvVoltage, p.plantTrafoResistance(),
      p.plantTrafoInductance(), open0 * p.plantTrafoResistance(),
      open0 * p.plantTrafoInductance(), b3, b4, true);
  if constexpr (T::singlePhase)
    plantTrafo->setParameters(p.hvVoltage, p.mvVoltage, p.plantTrafoPower,
                              p.hvVoltage / p.mvVoltage, p.plantShift,
                              seq(p.plantTrafoResistance(),
                                  open0 * p.plantTrafoResistance(), true),
                              seq(p.plantTrafoInductance(),
                                  open0 * p.plantTrafoInductance(), true));
  else
    plantTrafo->setYdVectorGroup(5);

  auto addLine = [&](const String &lineName, const LineData &d, Bool cable,
                     typename T::Node::Ptr from, typename T::Node::Ptr to) {
    auto line = T::Line::make(lineName);
    line->setParameters(seq(d.resistance, (cable ? 4. : 2.5) * d.resistance),
                        seq(d.inductance, (cable ? 1. : 3.) * d.inductance),
                        seq(d.capacitance, (cable ? 1. : 0.6) * d.capacitance));
    line->connect({from, to});
    comps.push_back(line);
    return line;
  };
  auto l1 = addLine("L1", p.l1(), false, b1l, b2);
  auto k1 = addLine("K1", p.k1(), true, b1, b3);
  addLine("CableEq", p.cableEq(), true, b1, b5);

  // constant-impedance loads: parallel R (and L) to ground
  auto addLoad = [&](const String &loadName, Real P, Real Q,
                     typename T::Node::Ptr node) {
    Real v2 = p.hvVoltage * p.hvVoltage;
    auto res = T::Res::make(loadName + "_R");
    res->setParameters(seq(v2 / P, open0 * v2 / P));
    res->connect({node, gnd});
    comps.push_back(res);
    if (Q > 0) {
      auto ind = T::Ind::make(loadName + "_L");
      ind->setParameters(
        seq(v2 / (p.omega * Q), open0 * v2 / (p.omega * Q)));
      ind->connect({node, gnd});
      comps.push_back(ind);
    }
  };
  addLoad("LoadB1", p.loadB1P, 0, b1);
  addLoad("LoadB2", p.loadB2P, p.loadB2Q, b2);
  addLoad("LoadB5", p.loadB5P, 0, b5);

  auto msc = T::Cap::make("MSC");
  // star point grounded (solid) or isolated (resonant)
  Real cMsc = p.mscQ / (p.omega * p.hvVoltage * p.hvVoltage);
  msc->setParameters(seq(cMsc, p.grounding == 2 ? 1e-3 * cMsc : cMsc));
  msc->connect({b1, gnd});
  comps.push_back(msc);

  auto fault = T::Sw::make("Fault");
  if constexpr (T::singlePhase) {
    fault->setParameters(T::par(p.switchOpen), T::par(p.faultResistance),
                         faultClosed);
  } else {
    // unfaulted phases keep the open resistance when the switch closes
    Matrix closed = T::par(p.switchOpen);
    for (int k = 0; k < 3; ++k)
      if (p.faultMask()[k])
        closed(k, k) = p.faultResistance;
    if (p.faultType == 4) {
      // b to c through the fault resistance, all phases open to ground
      Matrix g = Matrix::Identity(3, 3) / p.switchOpen;
      g(1, 1) += 1. / p.faultResistance;
      g(2, 2) += 1. / p.faultResistance;
      g(1, 2) -= 1. / p.faultResistance;
      g(2, 1) -= 1. / p.faultResistance;
      closed = g.inverse();
    }
    fault->setParameters(T::par(p.switchOpen), closed, faultClosed);
  }
  fault->connect({gnd, b2});
  comps.push_back(fault);
  std::shared_ptr<typename T::Sw> fault2;
  if (p.secondFault) {
    fault2 = T::Sw::make("Fault2");
    if constexpr (T::singlePhase) {
      fault2->setParameters(T::par(p.switchOpen), T::par(p.faultResistance),
                            faultClosed);
    } else {
      Matrix closed = T::par(p.switchOpen);
      closed(1, 1) = p.faultResistance;
      fault2->setParameters(T::par(p.switchOpen), closed, faultClosed);
    }
    fault2->connect({gnd, b3});
    comps.push_back(fault2);
  }

  g.n0 = n0, g.b1 = b1, g.b1l = b1l, g.b2 = b2, g.b3 = b3, g.b4 = b4, g.b5 = b5;
  g.l1 = l1, g.k1 = k1, g.fault = fault, g.fault2 = fault2;
  return g;
}

template <typename T>
void run(const Params &p, const String &simName, Bool idealBreaker) {
  auto systemPF = powerFlow(p, simName);
  String name = simName + "_" + T::tag;
  Logger::setLogDir("logs/" + name);

  auto g = buildGrid<T>(p, false);
  auto &comps = g.comps;
  auto n0 = g.n0, b1 = g.b1, b1l = g.b1l, b2 = g.b2, b3 = g.b3, b4 = g.b4,
       b5 = g.b5;
  auto l1 = g.l1, k1 = g.k1;
  auto fault = g.fault, fault2 = g.fault2;
  auto gnd = T::Node::GND;

  auto addWT = [&](const String &wtName, Real P, Bool hvrt) {
    auto wt = T::WT::make(wtName, wtName);
    wt->setParameters(p.omega, p.mvVoltage, P, p.wtReactivePower,
                      p.wtRatingFactor * P);
    wt->setControllerParameters(p.kpPLL, p.kiPLL, p.omegaCutoffPLL);
    wt->setShuntResistanceFactor(p.wtShuntFactor);
    if constexpr (!std::is_same_v<T, SP1>)
      wt->setNegativeSequenceSupport(hvrt ? p.kNegative : p.kNegativeLegacy,
                                     0.005, p.kNegativeTauMeas);
    wt->setProtectionParameters(hvrt, 1, p.wtProtection);
    wt->connect({b4});
    comps.push_back(wt);
    return wt;
  };
  auto breaker = T::Sw::make("BreakerL1");
  std::shared_ptr<DP::Ph1::ZeroSequenceEquivalent> zeroNet;
  std::shared_ptr<typename T::WT> wt, wt2;
  if (p.wtMode == 0) {
    wt = addWT("WT", p.wtActivePower, true);
    wt->setProtectionMeasurement(p.wtMeasurement);
    wt2 = addWT("WTLegacy", p.wtLegacyActivePower, false);
    if constexpr (std::is_same_v<T, DP1>) {
      if (p.compensated) {
        // Thevenin equivalents across the open breaker, fault on or off;
        // comp=sv: zero sequence seen from the open breaker and fault
        auto thevenin = [&](Bool zero, Bool faultClosed) {
          Params q = p;
          q.zeroSequence = zero;
          auto aux = buildGrid<DP1>(q, faultClosed);
          // open breaker kept so the network is not split in two
          auto open = DP::Ph1::Switch::make("BreakerL1");
          open->setParameters(p.switchOpen, p.breakerResistance, false);
          open->connect({aux.b1, aux.b1l});
          aux.comps.push_back(open);
          auto sys = SystemTopology(p.frequency,
                                    SystemNodeList{aux.n0, aux.b1, aux.b1l,
                                                   aux.b2, aux.b3, aux.b4,
                                                   aux.b5},
                                    aux.comps);
          sys.initWithPowerflow(systemPF, Domain::DP);
          aux.b1l->setInitialVoltage(
              std::dynamic_pointer_cast<SimNode<Complex>>(
                  systemPF.node<TopologicalNode>("B1"))
                  ->singleVoltage());
          std::vector<std::pair<SimNode<Complex>::Ptr, SimNode<Complex>::Ptr>>
              ports = {{aux.b1, aux.b1l}};
          if (p.compSpaceVectorMode)
            ports.push_back({nullptr, aux.b2});
          if (p.compSpaceVectorMode && p.secondFault)
            ports.push_back({nullptr, aux.b3});
          return extractTheveninEquivalent(
              sys, p.timeStep, ports, {aux.b1, aux.b1l, aux.b2, aux.b3, aux.b4},
              static_cast<UInt>(
                  std::max(1., std::round(p.timeStep / p.compStep))));
        };
        const std::vector<DP::Ph1::Switch::ObservedNode> observed = {
            {"B1"}, {"B1L"}, {"B2"}, {"B3"}, {"B4", -p.plantShift, false}};
        if (p.compSpaceVectorMode) {
          // breaker (0) and fault (1) share one zero-sequence network
          zeroNet = DP::Ph1::ZeroSequenceEquivalent::make(
              "ZeroSequence", p.secondFault ? 3 : 2, thevenin(true, false),
              observed);
          breaker->setZeroSequenceNetwork(zeroNet, 0);
          breaker->setSameStepOpening(p.compSameStep);
          fault->setZeroSequenceNetwork(zeroNet, 1);
          fault->setPoleMask(p.faultMask());
          fault->setCurrentZeroOpening(false);
          if (fault2) {
            fault2->setZeroSequenceNetwork(zeroNet, 2);
            fault2->setPoleMask({false, true, false});
            fault2->setCurrentZeroOpening(false);
          }
        } else {
          breaker->setTheveninEquivalents(thevenin(false, p.withFault),
                                          thevenin(true, p.withFault),
                                          observed);
          breaker->setNetworkHandover(p.compHandover);
          // after the fault ends (set 1)
          if (p.withFault)
            breaker->addTheveninEquivalents(thevenin(false, false),
                                            thevenin(true, false));
        }
        for (auto &unit : {wt, wt2})
          unit->setBreakerCompensation(
              zeroNet ? zeroNet->template attributeTyped<MatrixComp>(
                            "compensation_v_B4")
                      : breaker->template attributeTyped<MatrixComp>(
                            "compensation_v_B4"),
              breaker->template attributeTyped<Bool>("compensation_active"),
              p.compSpaceVector,
              breaker->template attributeTyped<Bool>("compensation_handed_over"));
      }
    }
  } else if constexpr (!std::is_void_v<typename T::Isrc>) {
    if (p.wtMode >= 2) {
      // DP1 LL-RMS envelope; DP3 phase peak; EMT3 phase RMS
      Complex i = std::conj(
          Complex(p.wtActivePower + p.wtLegacyActivePower, p.wtReactivePower) /
          std::dynamic_pointer_cast<SimNode<Complex>>(
              systemPF.node<TopologicalNode>("B4"))
              ->singleVoltage());
      auto src = T::Isrc::make("WTSource");
      if constexpr (T::singlePhase)
        src->setParameters(i);
      else if constexpr (T::domain == Domain::DP)
        src->setParameters(
            Math::singlePhaseVariableToThreePhase(i * std::sqrt(2. / 3.)));
      else
        src->setParameters(
            Math::singlePhaseVariableToThreePhase(i / std::sqrt(3.)), 50.);
      // Ph1 injects into terminal 1, Ph3 into terminal 0
      if constexpr (T::singlePhase)
        src->connect({gnd, b4});
      else
        src->connect({b4, gnd});
      comps.push_back(src);
    }
  }

  breaker->setParameters(T::par(p.switchOpen), T::par(p.breakerResistance),
                         true);
  breaker->connect({b1, b1l});
  comps.push_back(breaker);

  // with compensation the DP1 network keeps the breaker pole closed
  Bool compensated = T::singlePhase && p.compensated;
  if constexpr (T::switchModes) {
    breaker->setSwitchingMode(idealBreaker
                                  ? T::Sw::SwitchingMode::Ideal
                                  : T::Sw::SwitchingMode::CurrentZero);
    fault->setSwitchingMode(T::Sw::SwitchingMode::Ideal);
    if (fault2)
      fault2->setSwitchingMode(T::Sw::SwitchingMode::Ideal);
  }
  if constexpr (std::is_same_v<T, DP1>)
    if (compensated) {
      breaker->setSwitchingMode(p.compSpaceVectorMode
                                    ? T::Sw::SwitchingMode::SpaceVector
                                    : T::Sw::SwitchingMode::Compensated);
      if (p.compSpaceVectorMode)
        fault->setSwitchingMode(T::Sw::SwitchingMode::SpaceVector);
      if (p.compSpaceVectorMode && fault2)
        fault2->setSwitchingMode(T::Sw::SwitchingMode::SpaceVector);
    }
  // a single-phase network has no unsymmetrical fault of its own
  if constexpr (T::singlePhase)
    if ((p.faultType != 3 || p.secondFault) &&
        !(compensated && p.compSpaceVectorMode))
      throw std::invalid_argument(
          "faultType=1ph|2ph and fault2=on need comp=sv in single-phase "
          "domains.");
  if constexpr (T::singlePhase)
    if (p.faultType == 4)
      throw std::invalid_argument(
          "faultType=2pp needs per-port 3x3 conditions, not in Ph1.");
  Real tOpen = (T::switchModes && !idealBreaker) ? p.breakerOpen
                                                 : p.breakerOpenIdeal;
  std::shared_ptr<EMT::Ph3::CurrentSource> probe;
  if constexpr (std::is_same_v<T, EMT3>)
    if (p.probeCurrent > 0. && p.probeFrequency > 0.) {
      probe = EMT::Ph3::CurrentSource::make("Probe");
      const Real s = p.probeNegative ? 1. : -1.;
      MatrixComp iRef(3, 1);
      for (int k = 0; k < 3; ++k)
        iRef(k, 0) = std::polar(p.probeCurrent, s * 2. * PI * k / 3.);
      probe->setParameters(iRef, p.probeFrequency);
      probe->connect({gnd, b4});
      comps.push_back(probe);
    }

  auto system = SystemTopology(
      p.frequency, SystemNodeList{n0, b1, b1l, b2, b3, b4, b5}, comps);
  system.initWithPowerflow(systemPF, T::domain);
  // B1L is not in the power flow: the closed breaker puts it at B1
  b1l->setInitialVoltage(
      std::dynamic_pointer_cast<SimNode<Complex>>(
          systemPF.node<TopologicalNode>("B1"))
          ->singleVoltage());
  // the power flow ignores the Yd5 shift
  b4->setInitialVoltage(std::dynamic_pointer_cast<SimNode<Complex>>(
                            systemPF.node<TopologicalNode>("B4"))
                            ->singleVoltage() *
                        std::polar(1.0, -p.plantShift));

  auto logger = DataLogger::make(name);
  for (auto node : {b1, b1l, b2, b3, b4, b5})
    logger->logAttribute(node->name(), node->attribute("v"));
  logger->logAttribute("iL1", l1->attribute("i_intf"));
  logger->logAttribute("iK1", k1->attribute("i_intf"));
  logger->logAttribute("iBrk", breaker->attribute("i_intf"));
  if (probe)
    logger->logAttribute("iProbe", probe->attribute("i_intf"));
  if (compensated && p.compLog) {
    logger->logAttribute("comp_D", breaker->attribute("compensation_current"));
    for (String n : {"B1", "B1L", "B2", "B3", "B4"})
      logger->logAttribute("cv" + n,
                           zeroNet ? zeroNet->attribute("compensation_v_" + n)
                                   : breaker->attribute("compensation_v_" + n));
    if (zeroNet)
      logger->logAttribute("comp_F", fault->attribute("compensation_current"));
  }
  if (wt) {
    logger->logAttribute("wt_p", wt->attribute("p"));
    logger->logAttribute("wt_q", wt->attribute("q"));
    logger->logAttribute("wt_u", wt->attribute("UFilteredPu"));
    logger->logAttribute("wt_id", wt->attribute("idRefPu"));
    logger->logAttribute("wt_iq", wt->attribute("iqRefPu"));
    if constexpr (!std::is_same_v<T, SP1>) {
      logger->logAttribute("wt_u2", wt->attribute("U2Pu"));
      logger->logAttribute("wt_i2", wt->attribute("I2Pu"));
    }
    logger->logAttribute("wt2_p", wt2->attribute("p"));
    logger->logAttribute("wt2_q", wt2->attribute("q"));
    logger->logAttribute("wt2_u", wt2->attribute("UFilteredPu"));
    logger->logAttribute("wt_uprot", wt->attribute("UProtPu"));
    logger->logAttribute("wt_i", wt->attribute("i_intf"));
    logger->logAttribute("wt2_i", wt2->attribute("i_intf"));
    logger->logAttribute("wt_trip", wt->attribute("tripped"));
    logger->logAttribute("wt2_trip", wt2->attribute("tripped"));
  }

  Simulation sim(name);
  sim.setSystem(system);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.setDomain(T::domain);
  sim.doQuadratureCorrection(p.quadratureCorrection);
  sim.doZeroCrossingInterpolation(p.zeroCrossingInterpolation);
  sim.addLogger(logger);
  if (p.withFault)
    sim.addEvent(T::Event::make(p.faultOn, fault, true));
  if (p.withFault && fault2)
    sim.addEvent(T::Event::make(p.faultOn, fault2, true));
  sim.addEvent(T::Event::make(tOpen, breaker, false));
  if (p.withFault)
    sim.addEvent(T::Event::make(p.faultOff, fault, false));
  if (p.withFault && fault2)
    sim.addEvent(T::Event::make(p.faultOff, fault2, false));
  if (compensated && !p.compSpaceVectorMode && p.compTopology && p.withFault &&
      p.wtMode == 0)
    sim.addEvent(AttributeEvent<Int>::make(
        p.faultOff, breaker->template attributeTyped<Int>("thevenin_index"),
        1));
  sim.addEvent(T::Event::make(p.reclose, breaker, true));
  if (p.reopen > 0)
    sim.addEvent(T::Event::make(p.reopen, breaker, false));
  sim.run();
}

// Options (-o KEY=VALUE):
// domains=SP,DP1,DP3,EMT; breaker=currentzero|ideal;
// tOpenIdeal=<s> (opening, SP and ideal breakers); tCmd=<s> (opening command);
// tFaultOff=<s> (fault end); tReopen=<s> (second opening after the reclose);
// tap=<coupling transformer ratio factor>; msc=<Mvar> (capacitor bank at B1);
// protection=off (no VRT trips); meas=v1|peak|half (HVRT unit measurand);
// qc=off|interp (no quadrature correction, or zero-crossing interpolation);
// wt=off|fixed (no WT units, or constant current at the power-flow value);
// fault=off (L1 opens under load); faultR=<ohm>; fault2=on (b to ground at B3);
// faultType=1ph|2ph|2pp (a-g, bc-g, b-c; Ph1 only with comp=sv, 2pp not);
// ground=solid|resonant (DP3/EMT neutral grounding);
// comp=on|sv (DP1 compensated breaker; sv: space vector);
// ctrl=v1v2|v1 (voltage to the WT control); compStep=<s> (extraction step);
// compTopo=off (keep the fault equivalents); handover=off (no state handover);
// compLog=off; sameStep=off (comp=sv: pole opens in the step after its zero);
// k2=<k>, k2Legacy=<k> (negative-sequence reactive current); k2Meas=<s> (u2
// low pass); probeA=<A>, probeF=<Hz>, probeSeq=neg|pos (EMT current into B4).
int main(int argc, char *argv[]) {
  Params p;
  CommandLineArgs args(argc, argv, "TOV_LoadRejection", p.timeStep,
                       p.finalTime);
  p.timeStep = args.timeStep;
  p.finalTime = args.duration;
  String simName = args.name;

  String domains = args.options.count("domains")
                       ? args.getOptionString("domains")
                       : "SP,DP1,DP3,EMT";
  Bool idealBreaker = args.options.count("breaker") &&
                      args.getOptionString("breaker") == "ideal";
  if (args.options.count("tOpenIdeal"))
    p.breakerOpenIdeal = args.getOptionReal("tOpenIdeal");
  if (args.options.count("tCmd"))
    p.breakerOpen = args.getOptionReal("tCmd");
  if (args.options.count("tFaultOff"))
    p.faultOff = args.getOptionReal("tFaultOff");
  if (args.options.count("tReopen"))
    p.reopen = args.getOptionReal("tReopen");
  if (args.options.count("compLog"))
    p.compLog = args.getOptionString("compLog") != "off";
  if (args.options.count("sameStep"))
    p.compSameStep = args.getOptionString("sameStep") != "off";
  if (args.options.count("k2"))
    p.kNegative = args.getOptionReal("k2");
  if (args.options.count("k2Legacy"))
    p.kNegativeLegacy = args.getOptionReal("k2Legacy");
  if (args.options.count("k2Meas"))
    p.kNegativeTauMeas = args.getOptionReal("k2Meas");
  if (args.options.count("probeF"))
    p.probeFrequency = args.getOptionReal("probeF");
  if (args.options.count("probeA"))
    p.probeCurrent = args.getOptionReal("probeA");
  if (args.options.count("probeSeq"))
    p.probeNegative = args.getOptionString("probeSeq") != "pos";
  if (args.options.count("faultType")) {
    String f = args.getOptionString("faultType");
    p.faultType = f == "1ph" ? 1 : f == "2ph" ? 2 : f == "2pp" ? 4 : 3;
  }
  if (args.options.count("tap"))
    p.couplingTap = args.getOptionReal("tap");
  if (args.options.count("msc"))
    p.mscQ = 1e6 * args.getOptionReal("msc");
  if (args.options.count("protection"))
    p.wtProtection = args.getOptionString("protection") != "off";
  if (args.options.count("meas")) {
    String m = args.getOptionString("meas");
    p.wtMeasurement = m == "peak" ? 1 : m == "half" ? 2 : 0;
  }
  if (args.options.count("wt")) {
    String m = args.getOptionString("wt");
    p.wtMode = m == "off" ? 1 : m == "fixed" ? 2 : 0;
    if (p.wtMode == 1)
      p.wtActivePower = p.wtLegacyActivePower = 0;
  }
  if (args.options.count("comp"))
  {
    p.compensated = args.getOptionString("comp") == "on" ||
                    args.getOptionString("comp") == "sv";
    p.compSpaceVectorMode = args.getOptionString("comp") == "sv";
  }
  if (args.options.count("fault"))
    p.withFault = args.getOptionString("fault") != "off";
  if (args.options.count("fault2"))
    p.secondFault = args.getOptionString("fault2") == "on";
  if (args.options.count("faultR"))
    p.faultResistance = args.getOptionReal("faultR");
  if (args.options.count("ground")) {
    String g = args.getOptionString("ground");
    p.grounding = g == "solid" ? 1 : g == "resonant" ? 2 : 0;
  }
  if (args.options.count("compTopo"))
    p.compTopology = args.getOptionString("compTopo") != "off";
  if (args.options.count("handover"))
    p.compHandover = args.getOptionString("handover") != "off";
  if (args.options.count("compStep"))
    p.compStep = args.getOptionReal("compStep");
  if (args.options.count("ctrl"))
    p.compSpaceVector = args.getOptionString("ctrl") != "v1";
  if (args.options.count("qc")) {
    p.quadratureCorrection = args.getOptionString("qc") != "off";
    p.zeroCrossingInterpolation = args.getOptionString("qc") == "interp";
  }

  auto selected = [&](const String &tag) {
    return ("," + domains + ",").find("," + tag + ",") != String::npos;
  };
  if (selected("SP"))
    run<SP1>(p, simName, idealBreaker);
  if (selected("DP1"))
    run<DP1>(p, simName, idealBreaker);
  if (selected("DP3"))
    run<DP3>(p, simName, idealBreaker);
  if (selected("EMT"))
    run<EMT3>(p, simName, idealBreaker);
  return 0;
}
