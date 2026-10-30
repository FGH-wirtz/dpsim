// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// WSCC 9-bus comparison of the switch models: EMT zero-current switching as the
// reference against the DP ideal switch, the exponential ZCS ramp and the
// current-zero switch with quadrature correction. Reconstructs the test case of
//
//   C. Wirtz et al., "A Switch Model for Emulation of Zero Current Switching in
//   Dynamic Phasor", IEEE PES ISGT Europe 2023, DOI
//   10.1109/ISGTEUROPE56780.2023.10408527
//
// Setup taken from section IV-A of that paper: WSCC 9-bus at 50 Hz, switching
// action at bus 4, evaluation at bus 4 and bus 6, switching duration 0.01 s and
// a 50 us time step. Generators are ideal voltage sources so the same source
// model is used in both domains and the comparison isolates the switch.

#include <fstream>
#include <iostream>
#include <list>

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

namespace {

struct Parameters {
  Real nominalFrequency = 50.0;
  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 0.1;
  Real exponentialSwitchingTime = 0.01;
  Real zeroCrossingTolerance = 1e-6;
  Real timeStep = 50e-6;
  Real openCommandTime = 0.2;
  Real finalTime = 0.26;
};

/// The published case switches the bus 4 end of the bus 4 - bus 6 line. The
/// ground fault is kept as a control: there the current has other paths.
enum class Scenario { LineSwitch, GroundFault };

String scenarioName(Scenario s) {
  return s == Scenario::LineSwitch ? "Line" : "Fault";
}

/// Name of the bus 4 - bus 6 line in the WSCC-09 CIM data.
const char *LINE_4_6 = "LINE64";

std::list<fs::path> cimFiles() {
  return DPsim::Utils::findFiles(
      {"WSCC-09_DI.xml", "WSCC-09_EQ.xml", "WSCC-09_SV.xml", "WSCC-09_TP.xml"},
      "build/_deps/cim-data-src/WSCC-09/WSCC-09", "CIMPATH");
}

SystemTopology runPowerFlow(const Parameters &p, const String &simName) {
  const String name = simName + "_PF";
  Logger::setLogDir("logs/" + name);

  CPS::CIM::Reader reader(name, Logger::Level::off, Logger::Level::off);
  SystemTopology system =
      reader.loadCIM(p.nominalFrequency, cimFiles(), Domain::SP,
                     PhaseType::Single, CPS::GeneratorType::PVNode);
  system.component<CPS::SP::Ph1::SynchronGenerator>("GEN1")
      ->modifyPowerFlowBusType(CPS::PowerflowBusType::VD);

  Simulation sim(name, Logger::Level::off);
  sim.setSystem(system);
  sim.setTimeStep(p.finalTime);
  sim.setFinalTime(2 * p.finalTime);
  sim.setDomain(Domain::SP);
  sim.setSolverType(Solver::Type::NRP);
  sim.setSolverAndComponentBehaviour(Solver::Behaviour::Initialization);
  sim.doInitFromNodesAndTerminals(true);
  sim.run();

  return system;
}

void runEMT(const Parameters &p, const SystemTopology &systemPF, Scenario sc,
            EMT::Ph3::Switch::SwitchingMode mode, const String &modeName) {
  const String simName = "WSCC9_ZCS_EMT_" + scenarioName(sc) + "_" + modeName;
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader(simName, Logger::Level::off, Logger::Level::off);
  SystemTopology sys =
      reader.loadCIM(p.nominalFrequency, cimFiles(), Domain::EMT,
                     PhaseType::ABC, CPS::GeneratorType::IdealVoltageSource);
  sys.initWithPowerflow(systemPF, Domain::EMT);

  auto sw = EMT::Ph3::Switch::make("Breaker", Logger::Level::off);
  sw->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance), true);
  sw->setSwitchingMode(mode);
  sw->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  sw->setExponentialSwitchingTime(p.exponentialSwitchingTime);
  if (sc == Scenario::GroundFault) {
    sw->connect({CPS::EMT::SimNode::GND, sys.node<CPS::EMT::SimNode>("BUS6")});
    sys.addComponent(sw);
  } else {
    // Break the bus 4 - bus 6 line at its bus 4 end and insert the switch, so
    // opening it interrupts the line current.
    auto mid = CPS::EMT::SimNode::make("BUS4_sw", PhaseType::ABC);
    for (auto comp : sys.mComponents) {
      if (comp->name() != LINE_4_6)
        continue;
      auto line = std::dynamic_pointer_cast<CPS::SimPowerComp<Real>>(comp);
      line->connect({sys.node<CPS::EMT::SimNode>("BUS6"), mid});
    }
    sw->connect({mid, sys.node<CPS::EMT::SimNode>("BUS4")});
    sys.addNode(mid);
    sys.addComponent(sw);
  }

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v4",
                       sys.node<CPS::EMT::SimNode>("BUS4")->attribute("v"));
  logger->logAttribute("v6",
                       sys.node<CPS::EMT::SimNode>("BUS6")->attribute("v"));
  logger->logAttribute("i_sw", sw->attribute("i_intf"));
  logger->logAttribute("v_sw", sw->attribute("v_intf"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.addEvent(SwitchEvent3Ph::make(p.openCommandTime, sw, false));
  sim.addLogger(logger);
  sim.run();
}

void runDP(const Parameters &p, const SystemTopology &systemPF, Scenario sc,
           DP::Ph1::Switch::SwitchingMode mode, const String &modeName) {
  const String simName = "WSCC9_ZCS_DP_" + scenarioName(sc) + "_" + modeName;
  Logger::setLogDir("logs/" + simName);

  CPS::CIM::Reader reader(simName, Logger::Level::off, Logger::Level::off);
  SystemTopology sys =
      reader.loadCIM(p.nominalFrequency, cimFiles(), Domain::DP,
                     PhaseType::Single, CPS::GeneratorType::IdealVoltageSource);
  sys.initWithPowerflow(systemPF, Domain::DP);

  auto sw = DP::Ph1::Switch::make("Breaker", Logger::Level::off);
  sw->setParameters(p.switchOpenResistance, p.switchClosedResistance, true);
  sw->setSwitchingMode(mode);
  sw->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  sw->setExponentialSwitchingTime(p.exponentialSwitchingTime);
  if (sc == Scenario::GroundFault) {
    sw->connect({CPS::DP::SimNode::GND, sys.node<CPS::DP::SimNode>("BUS6")});
    sys.addComponent(sw);
  } else {
    auto mid = CPS::DP::SimNode::make("BUS4_sw", PhaseType::Single);
    for (auto comp : sys.mComponents) {
      if (comp->name() != LINE_4_6)
        continue;
      auto line = std::dynamic_pointer_cast<CPS::SimPowerComp<Complex>>(comp);
      line->connect({sys.node<CPS::DP::SimNode>("BUS6"), mid});
    }
    sw->connect({mid, sys.node<CPS::DP::SimNode>("BUS4")});
    sys.addNode(mid);
    sys.addComponent(sw);
  }

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v4",
                       sys.node<CPS::DP::SimNode>("BUS4")->attribute("v"));
  logger->logAttribute("v6",
                       sys.node<CPS::DP::SimNode>("BUS6")->attribute("v"));
  logger->logAttribute("i_sw", sw->attribute("i_intf"));
  logger->logAttribute("v_sw", sw->attribute("v_intf"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(sys);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.addEvent(SwitchEvent::make(p.openCommandTime, sw, false));
  sim.addLogger(logger);
  sim.run();
}

} // namespace

int main(int argc, char *argv[]) {
  try {
    Parameters p;

    CommandLineArgs args(argc, argv);
    if (argc > 1 && args.timeStep > 0.0)
      p.timeStep = args.timeStep;

    std::cout << "WSCC 9-bus switch comparison, dt = " << p.timeStep << " s"
              << std::endl;

    const SystemTopology systemPF = runPowerFlow(p, "WSCC9_ZCS");

    for (auto sc : {Scenario::LineSwitch, Scenario::GroundFault}) {
      std::cout << "scenario " << scenarioName(sc) << std::endl;

      runEMT(p, systemPF, sc, EMT::Ph3::Switch::SwitchingMode::CurrentZero,
             "CurrentZero");
      runDP(p, systemPF, sc, DP::Ph1::Switch::SwitchingMode::Ideal, "Ideal");
      runDP(p, systemPF, sc,
            DP::Ph1::Switch::SwitchingMode::ExponentialZCSEmulation,
            "ExponentialZCS");
      runDP(p, systemPF, sc, DP::Ph1::Switch::SwitchingMode::CurrentZero,
            "CurrentZero");

      std::cout << "  done" << std::endl;
    }

    return 0;
  } catch (const std::exception &e) {
    std::cerr << "EXCEPTION: " << e.what() << std::endl;
    return 1;
  } catch (...) {
    std::cerr << "EXCEPTION: unknown" << std::endl;
    return 1;
  }
}
