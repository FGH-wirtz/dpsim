/* Author: Christoph Wirtz <christoph.wirtz@fgh-ma.de>
 * SPDX-FileCopyrightText: 2026 FGH e.V.
 * SPDX-License-Identifier: MPL-2.0
 */

#include <DPsim.h>

using namespace DPsim;
using namespace CPS;

// Slack (110 kV) - line - n2 - transformer 110/20 kV - n3 - WECC WT4B, with a
// three-phase resistive fault at n2. All domains are initialized from the same
// power flow, so the 3-phase DP and EMT networks run in the same peak scale.
struct Params {
  Real frequency = 50;
  Real omega = 2. * PI * 50;
  Real timeStep = 50e-6;
  Real finalTime = 1.5;

  Real gridVoltage = 110e3;
  Real lineResistance = 2.4;
  Real lineInductance = 0.0255;
  Real lineCapacitance = 180e-9;

  Real mvVoltage = 20e3;
  Real trafoPower = 50e6;
  Real trafoResistance = 1.21;
  Real trafoInductance = 0.0924;

  Real wtActivePower = 40e6;
  Real wtReactivePower = 0;
  Real wtRatedPower = 45e6;
  Real kpPLL = 0.25;
  Real kiPLL = 2;
  Real omegaCutoffPLL = 2. * PI * 50;
  // damps the transformer LV snubber resonance (~6.5 kHz) against the WT
  // frame tracker; Norton-compensated, so steady-state-neutral
  Real wtShuntFactor = 1e3;

  Real faultResistance = 3;
  Real switchOpen = 1e9;
  Real faultOn = 0.5;
  Real faultOff = 0.65;
};

SystemTopology powerFlow(const Params &p, const String &simName) {
  String simNamePF = simName + "_PF";
  Logger::setLogDir("logs/" + simNamePF);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

  auto extnet = SP::Ph1::NetworkInjection::make("Slack");
  extnet->setParameters(p.gridVoltage);
  extnet->setBaseVoltage(p.gridVoltage);
  extnet->modifyPowerFlowBusType(PowerflowBusType::VD);

  auto line = SP::Ph1::PiLine::make("Line");
  line->setParameters(p.lineResistance, p.lineInductance, p.lineCapacitance);
  line->setBaseVoltage(p.gridVoltage);

  auto trafo = SP::Ph1::Transformer::make("Trafo");
  trafo->setParameters(p.gridVoltage, p.mvVoltage, p.gridVoltage / p.mvVoltage,
                       0, p.trafoResistance, p.trafoInductance);
  trafo->setBaseVoltage(p.gridVoltage);

  auto load = SP::Ph1::Load::make("WT");
  load->setParameters(-p.wtActivePower, -p.wtReactivePower, p.mvVoltage);
  load->modifyPowerFlowBusType(PowerflowBusType::PQ);

  extnet->connect({n1});
  line->connect({n1, n2});
  trafo->connect({n2, n3});
  load->connect({n3});
  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                     SystemComponentList{extnet, line, trafo, load});

  auto logger = DataLogger::make(simNamePF);
  logger->logAttribute("v1", n1->attribute("v"));
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));

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

template <typename WT>
void configureWT(const Params &p, std::shared_ptr<WT> wt) {
  wt->setParameters(p.omega, p.mvVoltage, p.wtActivePower, p.wtReactivePower,
                    p.wtRatedPower);
  wt->setControllerParameters(p.kpPLL, p.kiPLL, p.omegaCutoffPLL);
  wt->setShuntResistanceFactor(p.wtShuntFactor);
}

template <typename WT>
void logWT(std::shared_ptr<DataLogger> logger, std::shared_ptr<WT> wt) {
  logger->logAttribute("wt_p", wt->attribute("p"));
  logger->logAttribute("wt_q", wt->attribute("q"));
  logger->logAttribute("wt_u", wt->attribute("UFilteredPu"));
  logger->logAttribute("wt_id", wt->attribute("idRefPu"));
  logger->logAttribute("wt_iq", wt->attribute("iqRefPu"));
}

void runSP1ph(const Params &p, const String &simName) {
  auto systemPF = powerFlow(p, simName);
  String name = simName + "_SP";
  Logger::setLogDir("logs/" + name);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

  auto extnet = SP::Ph1::NetworkInjection::make("Slack");
  extnet->setParameters(Complex(p.gridVoltage, 0));
  auto line = SP::Ph1::PiLine::make("Line");
  line->setParameters(p.lineResistance, p.lineInductance, p.lineCapacitance);
  auto trafo = SP::Ph1::Transformer::make("Trafo", "Trafo",
                                          Logger::Level::off, true);
  trafo->setParameters(p.gridVoltage, p.mvVoltage, p.trafoPower,
                       p.gridVoltage / p.mvVoltage, 0, p.trafoResistance,
                       p.trafoInductance);
  auto wt = SP::Ph1::WECC_WT4B::make("WT", "WT");
  configureWT(p, wt);
  auto fault = SP::Ph1::Switch::make("Fault");
  fault->setParameters(p.switchOpen, p.faultResistance);
  fault->open();

  extnet->connect({n1});
  line->connect({n1, n2});
  trafo->connect({n2, n3});
  wt->connect({n3});
  fault->connect({SimNode<Complex>::GND, n2});
  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                     SystemComponentList{extnet, line, trafo, wt, fault});
  system.initWithPowerflow(systemPF, Domain::SP);

  auto logger = DataLogger::make(name);
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("itrafo", trafo->attribute("i_intf"));
  logWT(logger, wt);

  Simulation sim(name);
  sim.setSystem(system);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.setDomain(Domain::SP);
  sim.addLogger(logger);
  sim.addEvent(SwitchEvent::make(p.faultOn, fault, true));
  sim.addEvent(SwitchEvent::make(p.faultOff, fault, false));
  sim.run();
}

void runDP1ph(const Params &p, const String &simName) {
  auto systemPF = powerFlow(p, simName);
  String name = simName + "_DP1";
  Logger::setLogDir("logs/" + name);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);

  auto extnet = DP::Ph1::NetworkInjection::make("Slack");
  extnet->setParameters(Complex(p.gridVoltage, 0));
  auto line = DP::Ph1::PiLine::make("Line");
  line->setParameters(p.lineResistance, p.lineInductance, p.lineCapacitance);
  auto trafo = DP::Ph1::Transformer::make("Trafo", "Trafo",
                                          Logger::Level::off, true);
  trafo->setParameters(p.gridVoltage, p.mvVoltage, p.trafoPower,
                       p.gridVoltage / p.mvVoltage, 0, p.trafoResistance,
                       p.trafoInductance);
  auto wt = DP::Ph1::WECC_WT4B::make("WT", "WT");
  configureWT(p, wt);
  auto fault = DP::Ph1::Switch::make("Fault");
  fault->setSwitchingMode(DP::Ph1::Switch::SwitchingMode::CurrentZero);
  fault->setParameters(p.switchOpen, p.faultResistance);
  fault->open();

  extnet->connect({n1});
  line->connect({n1, n2});
  trafo->connect({n2, n3});
  wt->connect({n3});
  fault->connect({SimNode<Complex>::GND, n2});
  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                     SystemComponentList{extnet, line, trafo, wt, fault});
  system.initWithPowerflow(systemPF, Domain::DP);

  auto logger = DataLogger::make(name);
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("itrafo", trafo->attribute("i_intf"));
  logWT(logger, wt);

  Simulation sim(name);
  sim.setSystem(system);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.setDomain(Domain::DP);
  sim.addLogger(logger);
  sim.addEvent(SwitchEvent::make(p.faultOn, fault, true));
  sim.addEvent(SwitchEvent::make(p.faultOff, fault, false));
  sim.run();
}

void runDP3ph(const Params &p, const String &simName) {
  auto systemPF = powerFlow(p, simName);
  String name = simName + "_DP3";
  Logger::setLogDir("logs/" + name);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::ABC);

  auto extnet = DP::Ph3::NetworkInjection::make("Slack");
  auto line = DP::Ph3::PiLine::make("Line");
  line->setParameters(
      Math::singlePhaseParameterToThreePhase(p.lineResistance),
      Math::singlePhaseParameterToThreePhase(p.lineInductance),
      Math::singlePhaseParameterToThreePhase(p.lineCapacitance));
  auto trafo = DP::Ph3::Transformer::make("Trafo", "Trafo",
                                          Logger::Level::off, true);
  trafo->setParameters(
      p.gridVoltage, p.mvVoltage, p.trafoPower, p.gridVoltage / p.mvVoltage, 0,
      Math::singlePhaseParameterToThreePhase(p.trafoResistance),
      Math::singlePhaseParameterToThreePhase(p.trafoInductance));
  auto wt = DP::Ph3::WECC_WT4B::make("WT", "WT");
  configureWT(p, wt);
  auto fault = DP::Ph3::Switch::make("Fault");
  fault->setSwitchingMode(DP::Ph3::Switch::SwitchingMode::CurrentZero);
  fault->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpen),
      Math::singlePhaseParameterToThreePhase(p.faultResistance), false);

  extnet->connect({n1});
  line->connect({n1, n2});
  trafo->connect({n2, n3});
  wt->connect({n3});
  fault->connect({SimNode<Complex>::GND, n2});
  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                     SystemComponentList{extnet, line, trafo, wt, fault});
  system.initWithPowerflow(systemPF, Domain::DP);

  auto logger = DataLogger::make(name);
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("itrafo", trafo->attribute("i_intf"));
  logWT(logger, wt);

  Simulation sim(name);
  sim.setSystem(system);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.setDomain(Domain::DP);
  sim.addLogger(logger);
  sim.addEvent(SwitchEvent3Ph::make(p.faultOn, fault, true));
  sim.addEvent(SwitchEvent3Ph::make(p.faultOff, fault, false));
  sim.run();
}

void runEMT3ph(const Params &p, const String &simName) {
  auto systemPF = powerFlow(p, simName);
  String name = simName + "_EMT";
  Logger::setLogDir("logs/" + name);

  auto n1 = SimNode<Real>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Real>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<Real>::make("n3", PhaseType::ABC);

  auto extnet = EMT::Ph3::NetworkInjection::make("Slack");
  auto line = EMT::Ph3::PiLine::make("Line");
  line->setParameters(
      Math::singlePhaseParameterToThreePhase(p.lineResistance),
      Math::singlePhaseParameterToThreePhase(p.lineInductance),
      Math::singlePhaseParameterToThreePhase(p.lineCapacitance));
  auto trafo = EMT::Ph3::Transformer::make("Trafo", "Trafo",
                                           Logger::Level::off, true);
  trafo->setParameters(
      p.gridVoltage, p.mvVoltage, p.trafoPower, p.gridVoltage / p.mvVoltage, 0,
      Math::singlePhaseParameterToThreePhase(p.trafoResistance),
      Math::singlePhaseParameterToThreePhase(p.trafoInductance));
  auto wt = EMT::Ph3::WECC_WT4B::make("WT", "WT");
  configureWT(p, wt);
  auto fault = EMT::Ph3::Switch::make("Fault");
  fault->setSwitchingMode(EMT::Ph3::Switch::SwitchingMode::CurrentZero);
  fault->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpen),
      Math::singlePhaseParameterToThreePhase(p.faultResistance), false);

  extnet->connect({n1});
  line->connect({n1, n2});
  trafo->connect({n2, n3});
  wt->connect({n3});
  fault->connect({SimNode<Real>::GND, n2});
  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                     SystemComponentList{extnet, line, trafo, wt, fault});
  system.initWithPowerflow(systemPF, Domain::EMT);

  auto logger = DataLogger::make(name);
  logger->logAttribute("v2", n2->attribute("v"));
  logger->logAttribute("v3", n3->attribute("v"));
  logger->logAttribute("itrafo", trafo->attribute("i_intf"));
  logWT(logger, wt);

  Simulation sim(name);
  sim.setSystem(system);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.setDomain(Domain::EMT);
  sim.addLogger(logger);
  sim.addEvent(SwitchEvent3Ph::make(p.faultOn, fault, true));
  sim.addEvent(SwitchEvent3Ph::make(p.faultOff, fault, false));
  sim.run();
}

int main(int argc, char *argv[]) {
  Params p;
  String simName = "WT4B_Trafo_Fault";
  CommandLineArgs args(argc, argv);
  if (argc > 1) {
    p.timeStep = args.timeStep;
    p.finalTime = args.duration;
    if (args.name != "dpsim")
      simName = args.name;
  }

  runSP1ph(p, simName);
  runDP1ph(p, simName);
  runDP3ph(p, simName);
  runEMT3ph(p, simName);
  return 0;
}
