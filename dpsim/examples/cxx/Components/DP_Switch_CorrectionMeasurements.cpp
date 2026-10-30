// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// Measurement harness for the quadrature correction. One topology in EMT::Ph1
// and DP::Ph1 with the same values: an EMT reference at a fine step, and EMT
// and DP over a sweep of steps, DP with the correction on and off. The EMT
// current zero is analytic, since the circuit is linear and in steady state.
// The analysis is external.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph1_Switch.h>
#include <dpsim-models/DP/DP_Ph3_Switch.h>
#include <dpsim-models/EMT/EMT_Ph1_Switch.h>
#include <dpsim-models/EMT/EMT_Ph3_Switch.h>

using namespace DPsim;
using namespace CPS;

namespace {

// Set by --interpolate as the last argument
Bool zeroCrossingInterpolation = false;

struct Parameters {
  Real frequency = 50.0;
  Real sourceVoltage = 20e3;

  Real sourceResistance = 1.0;
  Real lineInductance = 50e-3;
  Real loadResistance = 50.0;
  // Breaker stray capacitance on the line side. Its recovery transient, with a
  // period of about 16 us, is resolved only by the reference step.
  Real strayCapacitance = 125e-12;

  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 1e-3;
  Real zeroCrossingTolerance = 1e-6;

  Real commandTime = 0.1;
  Real finalTime = 0.14;

  Real referenceTimeStep = 2e-7;
  // Down to where the recovery transient is resolved, so the sweep spans
  // both regimes: unresolved at the coarse end, resolved at the fine end.
  std::vector<Real> sweep{200e-6, 100e-6, 50e-6, 20e-6,
                          10e-6,  5e-6,   2e-6,  1e-6};

  Real omega() const { return 2.0 * PI * frequency; }
};

/// First zero of the steady-state inductor current at or after the command.
/// i(t) = |V| / |Z| * cos(w t - atan(X / R)), so the zeros are where
/// w t - atan(X / R) = pi / 2 + k pi.
Real analyticCurrentZero(const Parameters &p) {
  const Real reactance = p.omega() * p.lineInductance;
  const Real resistance = p.sourceResistance + p.loadResistance;
  const Real phase = std::atan2(reactance, resistance);
  const Real target = p.omega() * p.commandTime - phase - 0.5 * PI;
  const Real k = std::ceil(target / PI);

  return (phase + 0.5 * PI + k * PI) / p.omega();
}

String formatTimeStep(Real timeStep) {
  std::ostringstream name;
  name << std::fixed << std::setprecision(0) << (timeStep * 1e9) << "ns";
  return name.str();
}

struct RunRecord {
  String name;
  String domain;
  Real timeStep = 0.0;
  Bool correction = false;
  Real eventTime = 0.0;
  Real openingTime = -1.0;
};

RunRecord runEMT(const Parameters &p, Real timeStep, const String &label) {
  const String simName = "DP_Switch_CorrectionMeasurements_EMT_" + label;
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Real>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Real>::make("n3", PhaseType::Single);
  auto n4 = SimNode<Real>::make("n4", PhaseType::Single);

  auto vs = EMT::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0), p.frequency);

  auto rSource = EMT::Ph1::Resistor::make("R_src", Logger::Level::off);
  rSource->setParameters(p.sourceResistance);

  auto lLine = EMT::Ph1::Inductor::make("L_line", Logger::Level::off);
  lLine->setParameters(p.lineInductance);

  auto breaker = EMT::Ph1::Switch::make("Breaker", Logger::Level::off);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  breaker->setSwitchingMode(EMT::Ph1::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  auto rLoad = EMT::Ph1::Resistor::make("R_load", Logger::Level::off);
  rLoad->setParameters(p.loadResistance);

  auto cStray = EMT::Ph1::Capacitor::make("C_stray", Logger::Level::off);
  cStray->setParameters(p.strayCapacitance);

  vs->connect({SimNode<Real>::GND, n1});
  rSource->connect({n1, n2});
  lLine->connect({n2, n3});
  breaker->connect({n3, n4});
  rLoad->connect({n4, SimNode<Real>::GND});
  cStray->connect({n3, SimNode<Real>::GND});

  auto system = SystemTopology(
      p.frequency, SystemNodeList{n1, n2, n3, n4},
      SystemComponentList{vs, rSource, lLine, breaker, rLoad, cStray});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_line", lLine->attribute("i_intf"));
  logger->logAttribute("v_line", lLine->attribute("v_intf"));
  logger->logAttribute("v_breaker", breaker->attribute("v_intf"));
  logger->logAttribute("v_load", n4->attribute("v"));
  logger->logAttribute("v_stray", n3->attribute("v"));
  logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(p.finalTime);
  sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
  sim.run();
  sim.logLUTimes();

  RunRecord record;
  record.name = simName;
  record.domain = "EMT";
  record.timeStep = timeStep;
  record.correction = false;
  record.eventTime = p.commandTime;
  record.openingTime = **breaker->mZeroCrossingTime;

  return record;
}

RunRecord runDP(const Parameters &p, Real timeStep, Bool correction,
                const String &label) {
  const String simName = "DP_Switch_CorrectionMeasurements_DP_" + label + "_" +
                         (correction ? "corrected" : "plain");
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
  auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto rSource = DP::Ph1::Resistor::make("R_src", Logger::Level::off);
  rSource->setParameters(p.sourceResistance);

  auto lLine = DP::Ph1::Inductor::make("L_line", Logger::Level::off);
  lLine->setParameters(p.lineInductance);

  auto breaker = DP::Ph1::Switch::make("Breaker", Logger::Level::info);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  breaker->setSwitchingMode(DP::Ph1::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  auto rLoad = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  rLoad->setParameters(p.loadResistance);

  auto cStray = DP::Ph1::Capacitor::make("C_stray", Logger::Level::off);
  cStray->setParameters(p.strayCapacitance);

  vs->connect({SimNode<Complex>::GND, n1});
  rSource->connect({n1, n2});
  lLine->connect({n2, n3});
  breaker->connect({n3, n4});
  rLoad->connect({n4, SimNode<Complex>::GND});
  cStray->connect({n3, SimNode<Complex>::GND});

  auto system = SystemTopology(
      p.frequency, SystemNodeList{n1, n2, n3, n4},
      SystemComponentList{vs, rSource, lLine, breaker, rLoad, cStray});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_line", lLine->attribute("i_intf"));
  logger->logAttribute("v_line", lLine->attribute("v_intf"));
  logger->logAttribute("v_breaker", breaker->attribute("v_intf"));
  logger->logAttribute("v_load", n4->attribute("v"));
  logger->logAttribute("i_instantaneous",
                       breaker->attribute("i_instantaneous"));
  logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));
  logger->logAttribute("v_stray", n3->attribute("v"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(p.finalTime);
  sim.doQuadratureCorrection(correction);
  sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
  sim.run();
  sim.logLUTimes();

  RunRecord record;
  record.name = simName;
  record.domain = "DP";
  record.timeStep = timeStep;
  record.correction = correction;
  record.eventTime = p.commandTime;
  record.openingTime =
      breaker->attributeTyped<Real>("zero_crossing_time")->get();

  return record;
}

// Two switches, one asking for a correction and one ramping its resistance for
// a thousand steps. Only the switch that reports an event starts the
// correction, so the ramp must not add a correction call on each of its steps.
void runMixedModeCost(const Parameters &p) {
  const String simName = "DP_Switch_CorrectionMeasurements_MixedModeCost";
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
  auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);
  auto n5 = SimNode<Complex>::make("n5", PhaseType::Single);
  auto n6 = SimNode<Complex>::make("n6", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto rSource = DP::Ph1::Resistor::make("R_src", Logger::Level::off);
  rSource->setParameters(p.sourceResistance);

  auto lLine = DP::Ph1::Inductor::make("L_line", Logger::Level::off);
  lLine->setParameters(p.lineInductance);

  auto breaker = DP::Ph1::Switch::make("Breaker", Logger::Level::info);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  breaker->setSwitchingMode(DP::Ph1::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  auto rLoad = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  rLoad->setParameters(p.loadResistance);

  // The ramping one, on a damped branch of its own so it does not interfere
  auto lRamp = DP::Ph1::Inductor::make("L_ramp", Logger::Level::off);
  lRamp->setParameters(p.lineInductance);

  auto rRamp = DP::Ph1::Resistor::make("R_ramp", Logger::Level::off);
  rRamp->setParameters(p.loadResistance);

  auto ramping = DP::Ph1::Switch::make("Ramping", Logger::Level::info);
  ramping->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  ramping->setSwitchingMode(
      DP::Ph1::Switch::SwitchingMode::ExponentialZCSEmulation);
  ramping->setExponentialSwitchingTime(0.01);

  vs->connect({SimNode<Complex>::GND, n1});
  rSource->connect({n1, n2});
  lLine->connect({n2, n3});
  breaker->connect({n3, n4});
  rLoad->connect({n4, SimNode<Complex>::GND});
  ramping->connect({n2, n5});
  lRamp->connect({n5, n6});
  rRamp->connect({n6, SimNode<Complex>::GND});

  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3, n4, n5, n6},
                     SystemComponentList{vs, rSource, lLine, breaker, rLoad,
                                         ramping, lRamp, rRamp});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_line", lLine->attribute("i_intf"));
  logger->logAttribute("i_ramp", lRamp->attribute("i_intf"));
  logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));
  logger->logAttribute("i_instantaneous",
                       breaker->attribute("i_instantaneous"));
  logger->logAttribute("R_ramping", ramping->attribute("effective_resistance"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(50e-6);
  sim.setFinalTime(p.finalTime);
  // The ramp starts first and is still running when the breaker interrupts
  sim.addEvent(SwitchEvent::make(0.09, ramping, false));
  sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
  sim.run();
  sim.logLUTimes();

  std::cout << "Mixed-mode cost run written to logs/" << simName << std::endl;
}

// One network, five treatments of the same opening event, for the method
// comparison table. The network is built here rather than shared with runDP()
// so that the measurement sweep above stays exactly as it was validated.
struct MethodConfig {
  String label;
  DP::Ph1::Switch::SwitchingMode mode;
  Bool correction;
  Bool dampedStep;
  Real exponentialTime;
};

void runMethodComparison(const Parameters &p, Real timeStep) {
  const std::vector<MethodConfig> configs{
      {"Ideal", DP::Ph1::Switch::SwitchingMode::Ideal, false, false, 0.0},
      {"CurrentZero", DP::Ph1::Switch::SwitchingMode::CurrentZero, false, false,
       0.0},
      {"ExponentialZCS",
       DP::Ph1::Switch::SwitchingMode::ExponentialZCSEmulation, false, false,
       10e-3},
      {"DampedStep", DP::Ph1::Switch::SwitchingMode::CurrentZero, false, true,
       0.0},
      {"QuadratureCorrection", DP::Ph1::Switch::SwitchingMode::CurrentZero,
       true, false, 0.0},
  };

  std::ofstream manifest("logs/method_comparison_manifest.csv");
  manifest << "run,method,time_step_s,correction,damped_step,"
              "exponential_time_s,opening_time_s,analytic_zero_s\n";
  manifest << std::scientific << std::setprecision(12);

  const Real analyticZero = analyticCurrentZero(p);

  for (const auto &config : configs) {
    const String simName = "DP_Switch_MethodComparison_" + config.label + "_" +
                           formatTimeStep(timeStep);
    Logger::setLogDir("logs/" + simName);

    auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
    auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
    auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
    auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);

    n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

    auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
    vs->setParameters(Complex(p.sourceVoltage, 0.0));

    auto rSource = DP::Ph1::Resistor::make("R_src", Logger::Level::off);
    rSource->setParameters(p.sourceResistance);

    auto lLine = DP::Ph1::Inductor::make("L_line", Logger::Level::off);
    lLine->setParameters(p.lineInductance);

    auto breaker = DP::Ph1::Switch::make("Breaker", Logger::Level::info);
    breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                           true);
    breaker->setSwitchingMode(config.mode);
    breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
    if (config.exponentialTime > 0.0)
      breaker->setExponentialSwitchingTime(config.exponentialTime);

    auto rLoad = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
    rLoad->setParameters(p.loadResistance);

    auto cStray = DP::Ph1::Capacitor::make("C_stray", Logger::Level::off);
    cStray->setParameters(p.strayCapacitance);

    vs->connect({SimNode<Complex>::GND, n1});
    rSource->connect({n1, n2});
    lLine->connect({n2, n3});
    breaker->connect({n3, n4});
    rLoad->connect({n4, SimNode<Complex>::GND});
    cStray->connect({n3, SimNode<Complex>::GND});

    auto system = SystemTopology(
        p.frequency, SystemNodeList{n1, n2, n3, n4},
        SystemComponentList{vs, rSource, lLine, breaker, rLoad, cStray});

    auto logger = DataLogger::make(simName);
    logger->logAttribute("i_line", lLine->attribute("i_intf"));
    logger->logAttribute("v_line", lLine->attribute("v_intf"));
    logger->logAttribute("v_breaker", breaker->attribute("v_intf"));
    logger->logAttribute("v_load", n4->attribute("v"));
    logger->logAttribute("i_instantaneous",
                         breaker->attribute("i_instantaneous"));
    logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));
    logger->logAttribute("v_stray", n3->attribute("v"));

    Simulation sim(simName, Logger::Level::info);
    sim.setSystem(system);
    sim.addLogger(logger);
    sim.setDomain(Domain::DP);
    sim.setSolverType(Solver::Type::MNA);
    sim.setTimeStep(timeStep);
    sim.setFinalTime(p.finalTime);
    sim.doSystemMatrixRecomputation(true);
    sim.doQuadratureCorrection(config.correction);
    sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
    sim.setDampedStep(config.dampedStep);
    sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
    sim.run();
    sim.logLUTimes();

    const Real openingTime =
        breaker->attributeTyped<Real>("zero_crossing_time")->get();

    manifest << simName << "," << config.label << "," << timeStep << ","
             << (config.correction ? 1 : 0) << ","
             << (config.dampedStep ? 1 : 0) << "," << config.exponentialTime
             << "," << openingTime << "," << analyticZero << "\n";

    std::cout << "Method comparison: " << config.label << " written to logs/"
              << simName << std::endl;
  }
}

// A second feeder R2 + L2 at the line-end bus, beside the breaker. It never
// carries the breaker current, so its envelope must come through the opening
// unchanged, checked against an EMT reference opened where DP opened.
struct FeederParameters {
  Real sourceInductance = 10e-3;
  Real busCapacitance = 1e-6;
  Real busResistance = 100.0;
  Real feederResistance = 200.0;
  Real feederInductance = 20e-3;
};

Real runSecondFeederDP(const Parameters &p, const FeederParameters &f,
                       Real timeStep, Bool correction) {
  const String simName = "DP_Switch_SecondFeeder_DP_" +
                         formatTimeStep(timeStep) + "_" +
                         (correction ? "corrected" : "plain");
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto nA = SimNode<Complex>::make("nA", PhaseType::Single);
  auto nB = SimNode<Complex>::make("nB", PhaseType::Single);
  auto nD = SimNode<Complex>::make("nD", PhaseType::Single);
  auto nE = SimNode<Complex>::make("nE", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));
  auto lSource = DP::Ph1::Inductor::make("L_s", Logger::Level::off);
  lSource->setParameters(f.sourceInductance);
  auto cBus = DP::Ph1::Capacitor::make("C_A", Logger::Level::off);
  cBus->setParameters(f.busCapacitance);
  auto rBus = DP::Ph1::Resistor::make("R_A", Logger::Level::off);
  rBus->setParameters(f.busResistance);
  auto lLine = DP::Ph1::Inductor::make("L_line", Logger::Level::off);
  lLine->setParameters(p.lineInductance);
  auto cStray = DP::Ph1::Capacitor::make("C_stray", Logger::Level::off);
  cStray->setParameters(p.strayCapacitance);
  auto breaker = DP::Ph1::Switch::make("Breaker", Logger::Level::info);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  breaker->setSwitchingMode(DP::Ph1::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  auto rLoad = DP::Ph1::Resistor::make("R_load", Logger::Level::off);
  rLoad->setParameters(p.loadResistance);
  auto rFeeder = DP::Ph1::Resistor::make("R_2", Logger::Level::off);
  rFeeder->setParameters(f.feederResistance);
  auto lFeeder = DP::Ph1::Inductor::make("L_2", Logger::Level::off);
  lFeeder->setParameters(f.feederInductance);

  vs->connect({SimNode<Complex>::GND, n1});
  lSource->connect({n1, nA});
  cBus->connect({nA, SimNode<Complex>::GND});
  rBus->connect({nA, SimNode<Complex>::GND});
  lLine->connect({nA, nB});
  cStray->connect({nB, SimNode<Complex>::GND});
  breaker->connect({nB, nD});
  rLoad->connect({nD, SimNode<Complex>::GND});
  rFeeder->connect({nB, nE});
  lFeeder->connect({nE, SimNode<Complex>::GND});

  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, nA, nB, nD, nE},
                     SystemComponentList{vs, lSource, cBus, rBus, lLine, cStray,
                                         breaker, rLoad, rFeeder, lFeeder});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_line", lLine->attribute("i_intf"));
  logger->logAttribute("i_feeder", lFeeder->attribute("i_intf"));
  logger->logAttribute("i_source", lSource->attribute("i_intf"));
  logger->logAttribute("v_breaker", breaker->attribute("v_intf"));
  logger->logAttribute("v_bus", nB->attribute("v"));
  logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(p.finalTime);
  sim.doQuadratureCorrection(correction);
  sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
  sim.run();

  return breaker->attributeTyped<Real>("zero_crossing_time")->get();
}

void runSecondFeederEMT(const Parameters &p, const FeederParameters &f,
                        Real timeStep, Real openingTime) {
  const String simName =
      "DP_Switch_SecondFeeder_EMT_" + formatTimeStep(timeStep);
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Real>::make("n1", PhaseType::Single);
  auto nA = SimNode<Real>::make("nA", PhaseType::Single);
  auto nB = SimNode<Real>::make("nB", PhaseType::Single);
  auto nD = SimNode<Real>::make("nD", PhaseType::Single);
  auto nE = SimNode<Real>::make("nE", PhaseType::Single);

  auto vs = EMT::Ph1::VoltageSource::make("vs", Logger::Level::off);
  vs->setParameters(Complex(p.sourceVoltage, 0.0), p.frequency);
  auto lSource = EMT::Ph1::Inductor::make("L_s", Logger::Level::off);
  lSource->setParameters(f.sourceInductance);
  auto cBus = EMT::Ph1::Capacitor::make("C_A", Logger::Level::off);
  cBus->setParameters(f.busCapacitance);
  auto rBus = EMT::Ph1::Resistor::make("R_A", Logger::Level::off);
  rBus->setParameters(f.busResistance);
  auto lLine = EMT::Ph1::Inductor::make("L_line", Logger::Level::off);
  lLine->setParameters(p.lineInductance);
  auto cStray = EMT::Ph1::Capacitor::make("C_stray", Logger::Level::off);
  cStray->setParameters(p.strayCapacitance);
  auto breaker = EMT::Ph1::Switch::make("Breaker", Logger::Level::off);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  breaker->setSwitchingMode(EMT::Ph1::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  auto rLoad = EMT::Ph1::Resistor::make("R_load", Logger::Level::off);
  rLoad->setParameters(p.loadResistance);
  auto rFeeder = EMT::Ph1::Resistor::make("R_2", Logger::Level::off);
  rFeeder->setParameters(f.feederResistance);
  auto lFeeder = EMT::Ph1::Inductor::make("L_2", Logger::Level::off);
  lFeeder->setParameters(f.feederInductance);

  vs->connect({SimNode<Real>::GND, n1});
  lSource->connect({n1, nA});
  cBus->connect({nA, SimNode<Real>::GND});
  rBus->connect({nA, SimNode<Real>::GND});
  lLine->connect({nA, nB});
  cStray->connect({nB, SimNode<Real>::GND});
  breaker->connect({nB, nD});
  rLoad->connect({nD, SimNode<Real>::GND});
  rFeeder->connect({nB, nE});
  lFeeder->connect({nE, SimNode<Real>::GND});

  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, nA, nB, nD, nE},
                     SystemComponentList{vs, lSource, cBus, rBus, lLine, cStray,
                                         breaker, rLoad, rFeeder, lFeeder});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_line", lLine->attribute("i_intf"));
  logger->logAttribute("i_feeder", lFeeder->attribute("i_intf"));
  logger->logAttribute("i_source", lSource->attribute("i_intf"));
  logger->logAttribute("v_breaker", breaker->attribute("v_intf"));
  logger->logAttribute("v_bus", nB->attribute("v"));
  logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(p.finalTime);
  sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
  sim.run();
}

void runSecondFeeder(const Parameters &p) {
  const FeederParameters f;
  const Real timeStep = 50e-6;

  const Real openingTime = runSecondFeederDP(p, f, timeStep, true);
  runSecondFeederDP(p, f, timeStep, false);
  runSecondFeederEMT(p, f, p.referenceTimeStep, openingTime);

  std::cout << std::scientific << std::setprecision(12)
            << "Second feeder: DP opened at " << openingTime
            << " s, EMT reference detects its own current zero" << std::endl;
}

// Further validation networks for the quadrature correction, built from one
// netlist in both domains. Node "src" is driven by the source, "gnd" is ground.
struct NetElement {
  String name;
  char kind;
  String node0;
  String node1;
  Real value;
};

template <typename VarType> struct Ph1Models;
template <> struct Ph1Models<Complex> {
  using Resistor = DP::Ph1::Resistor;
  using Inductor = DP::Ph1::Inductor;
  using Capacitor = DP::Ph1::Capacitor;
  using Switch = DP::Ph1::Switch;
  using Source = DP::Ph1::VoltageSource;
};
template <> struct Ph1Models<Real> {
  using Resistor = EMT::Ph1::Resistor;
  using Inductor = EMT::Ph1::Inductor;
  using Capacitor = EMT::Ph1::Capacitor;
  using Switch = EMT::Ph1::Switch;
  using Source = EMT::Ph1::VoltageSource;
};

/// Returns the instant the breaker opened: found by the DP switch, given to EMT
template <typename VarType>
Real runNetlist(const Parameters &p, const String &caseName,
                const std::vector<NetElement> &netlist, Real timeStep,
                Bool correction, Real openingTime) {
  using M = Ph1Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  const String simName =
      "DP_Switch_" + caseName + (dp ? "_DP_" : "_EMT_") +
      formatTimeStep(timeStep) +
      (dp ? (correction ? "_corrected" : "_plain") : String());
  Logger::setLogDir("logs/" + simName);

  std::map<String, typename SimNode<VarType>::Ptr> nodes;
  SystemNodeList nodeList;
  auto node = [&](const String &name) {
    if (name == "gnd")
      return SimNode<VarType>::GND;
    auto found = nodes.find(name);
    if (found != nodes.end())
      return found->second;
    auto created = SimNode<VarType>::make(name, PhaseType::Single);
    nodes[name] = created;
    nodeList.push_back(created);
    return created;
  };

  auto source = M::Source::make("vs", Logger::Level::off);
  if constexpr (std::is_same<VarType, Complex>::value) {
    source->setParameters(Complex(p.sourceVoltage, 0.0));
    node("src")->setInitialVoltage(Complex(p.sourceVoltage, 0.0));
  } else {
    source->setParameters(Complex(p.sourceVoltage, 0.0), p.frequency);
  }
  source->connect({SimNode<VarType>::GND, node("src")});

  SystemComponentList components{source};
  std::shared_ptr<typename M::Switch> breaker;
  auto logger = DataLogger::make(simName);

  for (const auto &element : netlist) {
    switch (element.kind) {
    case 'R': {
      auto r = M::Resistor::make(element.name, Logger::Level::off);
      r->setParameters(element.value);
      r->connect({node(element.node0), node(element.node1)});
      components.push_back(r);
      break;
    }
    case 'L': {
      auto l = M::Inductor::make(element.name, Logger::Level::off);
      l->setParameters(element.value);
      l->connect({node(element.node0), node(element.node1)});
      logger->logAttribute("i_" + element.name, l->attribute("i_intf"));
      components.push_back(l);
      break;
    }
    case 'C': {
      auto c = M::Capacitor::make(element.name, Logger::Level::off);
      c->setParameters(element.value);
      c->connect({node(element.node0), node(element.node1)});
      components.push_back(c);
      break;
    }
    case 'B': {
      breaker = M::Switch::make("Breaker",
                                dp ? Logger::Level::info : Logger::Level::off);
      breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                             true);
      breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
      breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
      breaker->connect({node(element.node0), node(element.node1)});
      // Logged so the scheduler keeps the switch's post-step, which is where
      // the current zero is detected
      logger->logAttribute("i_breaker", breaker->attribute("i_intf"));
      logger->logAttribute("pole_closed", breaker->attribute("pole_closed"));
      components.push_back(breaker);
      break;
    }
    default:
      throw std::invalid_argument("Unknown element kind in " + caseName);
    }
  }

  for (const auto &entry : nodes)
    logger->logAttribute("v_" + entry.first, entry.second->attribute("v"));

  auto system = SystemTopology(p.frequency, nodeList, components);

  Simulation sim(simName, dp ? Logger::Level::info : Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(dp ? Domain::DP : Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(p.finalTime);
  if (dp) {
    sim.doQuadratureCorrection(correction);
    sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  }
  sim.addEvent(SwitchEvent::make(p.commandTime, breaker, false));
  sim.run();

  return breaker->template attributeTyped<Real>("zero_crossing_time")->get();
}

void runNetlistCase(const Parameters &p, const String &caseName,
                    const std::vector<NetElement> &netlist) {
  const Real timeStep = 50e-6;
  const Real openingTime =
      runNetlist<Complex>(p, caseName, netlist, timeStep, true, 0.0);
  runNetlist<Complex>(p, caseName, netlist, timeStep, false, 0.0);
  runNetlist<Real>(p, caseName, netlist, p.referenceTimeStep, false,
                   openingTime);

  std::cout << std::scientific << std::setprecision(12) << caseName
            << ": DP opened at " << openingTime
            << " s, EMT reference detects its own current zero" << std::endl;
}

const std::vector<NetElement> lineEnd{
    {"L_s", 'L', "src", "A", 10e-3},   {"C_A", 'C', "A", "gnd", 1e-6},
    {"R_A", 'R', "A", "gnd", 100.0},   {"L_line", 'L', "A", "B", 50e-3},
    {"C_B", 'C', "B", "gnd", 125e-12}, {"Breaker", 'B', "B", "D", 0.0},
    {"R_D", 'R', "D", "gnd", 50.0}};

const std::vector<NetElement> parallelLines{
    {"L_s", 'L', "src", "A", 10e-3}, {"C_A", 'C', "A", "gnd", 1e-6},
    {"R_A", 'R', "A", "gnd", 100.0}, {"L_line", 'L', "A", "B", 50e-3},
    {"L_par", 'L', "A", "B", 30e-3}, {"C_B", 'C', "B", "gnd", 125e-12},
    {"Breaker", 'B', "B", "D", 0.0}, {"R_D", 'R', "D", "gnd", 50.0}};

const std::vector<NetElement> meshTie{
    {"L_s", 'L', "src", "A", 10e-3},   {"C_A", 'C', "A", "gnd", 1e-6},
    {"R_A", 'R', "A", "gnd", 100.0},   {"L_line", 'L', "A", "B", 50e-3},
    {"C_B", 'C', "B", "gnd", 1e-9},    {"R_B", 'R', "B", "gnd", 80.0},
    {"L_2", 'L', "A", "C", 30e-3},     {"C_C", 'C', "C", "gnd", 1e-9},
    {"R_C", 'R', "C", "gnd", 60.0},    {"L_t", 'L', "B", "X", 20e-3},
    {"C_X", 'C', "X", "gnd", 100e-12}, {"Breaker", 'B', "X", "C", 0.0}};

// Capacitor bank energization: an Ideal breaker closes at a fixed instant onto
// an uncharged bank behind an inrush reactor. The source phase is chosen so the
// switch voltage is at a zero (synchronous closing) or at a peak at that instant.
struct ClosingCase {
  String name;
  Real busCapacitance;
  Real reactorInductance;
};

template <typename VarType>
void runClosing(const Parameters &p, const ClosingCase &c, Real timeStep,
                Real phase, const String &tag, Bool correction = false) {
  using M = Ph1Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  const Real closingTime = 0.1;
  const String simName =
      "DP_Switch_" + c.name + "_" + tag + (dp ? "_DP_" : "_EMT_") +
      formatTimeStep(timeStep) +
      (dp ? (correction ? "_corrected" : "_plain") : String());
  Logger::setLogDir("logs/" + simName);

  auto src = SimNode<VarType>::make("src", PhaseType::Single);
  auto a = SimNode<VarType>::make("A", PhaseType::Single);
  auto b = SimNode<VarType>::make("b", PhaseType::Single);
  auto n = SimNode<VarType>::make("c", PhaseType::Single);
  const Complex sourceVoltage = std::polar(p.sourceVoltage, phase);

  auto vs = M::Source::make("vs", Logger::Level::off);
  if constexpr (std::is_same<VarType, Complex>::value) {
    vs->setParameters(sourceVoltage);
    src->setInitialVoltage(sourceVoltage);
  } else {
    vs->setParameters(sourceVoltage, p.frequency);
  }
  vs->connect({SimNode<VarType>::GND, src});
  auto lSource = M::Inductor::make("L_s", Logger::Level::off);
  lSource->setParameters(10e-3);
  lSource->connect({src, a});
  auto cBus = M::Capacitor::make("C_A", Logger::Level::off);
  cBus->setParameters(c.busCapacitance);
  cBus->connect({a, SimNode<VarType>::GND});
  auto rBus = M::Resistor::make("R_A", Logger::Level::off);
  rBus->setParameters(100.0);
  rBus->connect({a, SimNode<VarType>::GND});
  auto breaker = M::Switch::make("Breaker", Logger::Level::off);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         false);
  if constexpr (std::is_same<VarType, Complex>::value)
    breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
  breaker->connect({a, b});
  auto lReactor = M::Inductor::make("L_c", Logger::Level::off);
  lReactor->setParameters(c.reactorInductance);
  lReactor->connect({b, n});
  auto cBank = M::Capacitor::make("C_1", Logger::Level::off);
  cBank->setParameters(10e-6);
  cBank->connect({n, SimNode<VarType>::GND});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_L_c", lReactor->attribute("i_intf"));
  logger->logAttribute("v_A", a->attribute("v"));
  logger->logAttribute("v_c", n->attribute("v"));

  auto system = SystemTopology(
      p.frequency, SystemNodeList{src, a, b, n},
      SystemComponentList{vs, lSource, cBus, rBus, breaker, lReactor, cBank});

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(dp ? Domain::DP : Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(closingTime + 0.01);
  if (dp) {
    sim.doQuadratureCorrection(correction);
    sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  }
  sim.addEvent(SwitchEvent::make(closingTime - 1e-10, breaker, true));
  sim.run();
}

void runCapacitorClosing(const Parameters &p) {
  const std::vector<ClosingCase> cases{{"CapSingle", 1e-6, 0.5e-3},
                                       {"CapBackToBack", 10e-6, 50e-6}};
  for (const auto &c : cases) {
    // Bus voltage of the open network relative to the source
    const Complex busImpedance =
        1.0 / (1.0 / 100.0 + Complex(0.0, p.omega() * c.busCapacitance));
    const Complex transfer =
        busImpedance / (busImpedance + Complex(0.0, p.omega() * 10e-3));
    // The closing instant 0.1 s is a whole number of periods
    for (const auto &[tag, phase] : std::vector<std::pair<String, Real>>{
             {"zero", 0.5 * PI - std::arg(transfer)},
             {"a30", PI / 3.0 - std::arg(transfer)},
             {"a60", PI / 6.0 - std::arg(transfer)},
             {"peak", -std::arg(transfer)}}) {
      runClosing<Real>(p, c, p.referenceTimeStep, phase, tag);
      for (const Real timeStep : {50e-6, 20e-6, 5e-6, 1e-6}) {
        runClosing<Real>(p, c, timeStep, phase, tag);
        runClosing<Complex>(p, c, timeStep, phase, tag, false);
        runClosing<Complex>(p, c, timeStep, phase, tag, true);
      }
    }
  }
}

// The coupled three-phase case of DP_Switch_GroupCorrection with an EMT
// reference, both opening each pole at its own current zero. A non-zero
// strayCapacitance adds it per phase on the breaker's line side.
template <typename VarType> struct Ph3Models;
template <> struct Ph3Models<Complex> {
  using Resistor = DP::Ph3::Resistor;
  using Inductor = DP::Ph3::Inductor;
  using Capacitor = DP::Ph3::Capacitor;
  using Switch = DP::Ph3::Switch;
  using Source = DP::Ph3::VoltageSource;
};
template <> struct Ph3Models<Real> {
  using Resistor = EMT::Ph3::Resistor;
  using Inductor = EMT::Ph3::Inductor;
  using Capacitor = EMT::Ph3::Capacitor;
  using Switch = EMT::Ph3::Switch;
  using Source = EMT::Ph3::VoltageSource;
};

template <typename VarType>
void runCoupledPhasesRun(const Parameters &p, Real timeStep, Bool correction,
                         Real strayCapacitance) {
  using M = Ph3Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  const String simName =
      String("DP_Switch_CoupledPhases") +
      (strayCapacitance > 0.0 ? "Stray" : "") + (dp ? "_DP_" : "_EMT_") +
      formatTimeStep(timeStep) +
      (dp ? (correction ? "_corrected" : "_plain") : String());
  Logger::setLogDir("logs/" + simName);

  Matrix inductance = Matrix::Zero(3, 3);
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col)
      inductance(row, col) = row == col ? 50e-3 : 15e-3;

  auto n1 = SimNode<VarType>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<VarType>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<VarType>::make("n3", PhaseType::ABC);
  const MatrixComp phaseVoltage =
      Math::singlePhaseVariableToThreePhase(Complex(p.sourceVoltage, 0.0));

  auto vs = M::Source::make("vs", Logger::Level::off);
  if constexpr (std::is_same<VarType, Complex>::value) {
    n1->setInitialVoltage(phaseVoltage);
    vs->setParameters(phaseVoltage);
  } else {
    // The EMT source scales its reference from RMS line to peak phase
    vs->setParameters(phaseVoltage / RMS3PH_TO_PEAK1PH, p.frequency);
  }
  vs->connect({SimNode<VarType>::GND, n1});

  auto coupled = M::Inductor::make("L_coupled", Logger::Level::off);
  coupled->setParameters(inductance);
  coupled->connect({n1, n2});

  auto breaker =
      M::Switch::make("Breaker", dp ? Logger::Level::info : Logger::Level::off);
  breaker->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance), true);
  breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  breaker->connect({n2, n3});

  auto load = M::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(Math::singlePhaseParameterToThreePhase(50.0));
  load->connect({n3, SimNode<VarType>::GND});

  SystemComponentList components{vs, coupled, breaker, load};
  if (strayCapacitance > 0.0) {
    auto stray = M::Capacitor::make("C_stray", Logger::Level::off);
    stray->setParameters(
        Math::singlePhaseParameterToThreePhase(strayCapacitance));
    stray->connect({n2, SimNode<VarType>::GND});
    components.push_back(stray);
  }

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_coupled", coupled->attribute("i_intf"));
  logger->logAttribute("v_n2", n2->attribute("v"));
  logger->logAttribute("v_n3", n3->attribute("v"));
  // Logged so the scheduler keeps the switch's post-step, which is where the
  // current zeros are detected
  logger->logAttribute("pole_a", breaker->attribute("pole_closed_a"));
  logger->logAttribute("pole_b", breaker->attribute("pole_closed_b"));
  logger->logAttribute("pole_c", breaker->attribute("pole_closed_c"));

  auto system =
      SystemTopology(p.frequency, SystemNodeList{n1, n2, n3}, components);

  Simulation sim(simName, dp ? Logger::Level::info : Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(dp ? Domain::DP : Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(p.finalTime);
  if (dp) {
    sim.doQuadratureCorrection(correction);
    sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  }
  sim.addEvent(SwitchEvent3Ph::make(p.commandTime, breaker, false));
  sim.run();

  std::cout
      << std::scientific << std::setprecision(9) << simName
      << ": poles opened at "
      << breaker->template attributeTyped<Real>("zero_crossing_time_a")->get()
      << ", "
      << breaker->template attributeTyped<Real>("zero_crossing_time_b")->get()
      << ", "
      << breaker->template attributeTyped<Real>("zero_crossing_time_c")->get()
      << " s" << std::endl;
}

void runCoupledPhases(const Parameters &p) {
  for (const Real stray : {0.0, p.strayCapacitance}) {
    runCoupledPhasesRun<Real>(p, 1e-6, false, stray);
    for (const Real timeStep : {50e-6, 20e-6}) {
      runCoupledPhasesRun<Complex>(p, timeStep, true, stray);
      runCoupledPhasesRun<Complex>(p, timeStep, false, stray);
    }
  }
}

// The back-to-back bank of runCapacitorClosing per phase, closed by a DP::Ph3
// breaker whose three poles close together, so pole a closes at its voltage
// zero and poles b and c at 60 degrees from theirs.
template <typename SwitchType> class PoleCloseEvent : public Event {
public:
  PoleCloseEvent(Real t, const std::shared_ptr<SwitchType> &sw, UInt phase)
      : Event(t), mSwitch(sw), mPhase(phase) {}

  void execute() override { mSwitch->closePole(mPhase); }

private:
  std::shared_ptr<SwitchType> mSwitch;
  UInt mPhase;
};

template <typename VarType>
void runClosingPh3Run(const Parameters &p, Real timeStep, Bool correction,
                      Real poleGridStep = 0.0) {
  using M = Ph3Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  const Bool perPole = poleGridStep > 0.0;
  const Real closingTime = 0.1;
  const Real busCapacitance = 10e-6;
  const String simName =
      String(perPole ? "DP_Switch_CapBackToBackPowPh3_" +
                           formatTimeStep(poleGridStep)
                     : "DP_Switch_CapBackToBackPh3") +
      (dp ? "_DP_" : "_EMT_") + formatTimeStep(timeStep) +
      (dp ? (correction ? "_corrected" : "_plain") : String());
  Logger::setLogDir("logs/" + simName);

  const Complex busImpedance =
      1.0 / (1.0 / 100.0 + Complex(0.0, p.omega() * busCapacitance));
  const Complex transfer =
      busImpedance / (busImpedance + Complex(0.0, p.omega() * 10e-3));
  const MatrixComp phaseVoltage = Math::singlePhaseVariableToThreePhase(
      std::polar(p.sourceVoltage, 0.5 * PI - std::arg(transfer)));

  auto src = SimNode<VarType>::make("src", PhaseType::ABC);
  auto a = SimNode<VarType>::make("A", PhaseType::ABC);
  auto b = SimNode<VarType>::make("b", PhaseType::ABC);
  auto n = SimNode<VarType>::make("c", PhaseType::ABC);

  auto vs = M::Source::make("vs", Logger::Level::off);
  if constexpr (std::is_same<VarType, Complex>::value) {
    src->setInitialVoltage(phaseVoltage);
    vs->setParameters(phaseVoltage);
  } else {
    // The EMT source scales its reference from RMS line to peak phase
    vs->setParameters(phaseVoltage / RMS3PH_TO_PEAK1PH, p.frequency);
  }
  vs->connect({SimNode<VarType>::GND, src});
  auto lSource = M::Inductor::make("L_s", Logger::Level::off);
  lSource->setParameters(Math::singlePhaseParameterToThreePhase(10e-3));
  lSource->connect({src, a});
  auto cBus = M::Capacitor::make("C_A", Logger::Level::off);
  cBus->setParameters(Math::singlePhaseParameterToThreePhase(busCapacitance));
  cBus->connect({a, SimNode<VarType>::GND});
  auto rBus = M::Resistor::make("R_A", Logger::Level::off);
  rBus->setParameters(Math::singlePhaseParameterToThreePhase(100.0));
  rBus->connect({a, SimNode<VarType>::GND});
  auto breaker = M::Switch::make("Breaker", Logger::Level::off);
  breaker->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance), false);
  if (dp || perPole)
    breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
  breaker->connect({a, b});
  auto lReactor = M::Inductor::make("L_c", Logger::Level::off);
  lReactor->setParameters(Math::singlePhaseParameterToThreePhase(50e-6));
  lReactor->connect({b, n});
  auto cBank = M::Capacitor::make("C_1", Logger::Level::off);
  cBank->setParameters(Math::singlePhaseParameterToThreePhase(10e-6));
  cBank->connect({n, SimNode<VarType>::GND});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_L_c", lReactor->attribute("i_intf"));
  logger->logAttribute("v_A", a->attribute("v"));
  logger->logAttribute("v_c", n->attribute("v"));

  auto system = SystemTopology(
      p.frequency, SystemNodeList{src, a, b, n},
      SystemComponentList{vs, lSource, cBus, rBus, breaker, lReactor, cBank});

  Simulation sim(simName, Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(dp ? Domain::DP : Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(closingTime + 0.01);
  if (dp) {
    sim.doQuadratureCorrection(correction);
    sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  }
  if (perPole) {
    // Point-on-wave closing: each pole at its own voltage zero, in the order
    // a, c, b, 60 degrees apart, rounded up to the step grid
    const std::array<UInt, 3> order{{0, 2, 1}};
    for (UInt k = 0; k < 3; ++k) {
      const Real zero = closingTime + k / (6.0 * p.frequency);
      const Real onGrid = std::ceil(zero / poleGridStep - 1e-6) * poleGridStep;
      sim.addEvent(std::make_shared<PoleCloseEvent<typename M::Switch>>(
          onGrid - 1e-10, breaker, order[k]));
    }
  } else {
    sim.addEvent(SwitchEvent3Ph::make(closingTime - 1e-10, breaker, true));
  }
  sim.run();
}

void runClosingPowPh3(const Parameters &p) {
  for (const Real timeStep : {50e-6, 20e-6, 5e-6}) {
    runClosingPh3Run<Real>(p, p.referenceTimeStep, false, timeStep);
    runClosingPh3Run<Real>(p, timeStep, false, timeStep);
    runClosingPh3Run<Complex>(p, timeStep, false, timeStep);
    runClosingPh3Run<Complex>(p, timeStep, true, timeStep);
  }
}

void runClosingPh3(const Parameters &p) {
  runClosingPh3Run<Real>(p, p.referenceTimeStep, false);
  for (const Real timeStep : {50e-6, 20e-6, 5e-6}) {
    runClosingPh3Run<Real>(p, timeStep, false);
    runClosingPh3Run<Complex>(p, timeStep, false);
    runClosingPh3Run<Complex>(p, timeStep, true);
  }
}

void writeManifest(const std::vector<RunRecord> &records, Real analyticZero) {
  const String path = "logs/correction_measurements_manifest.csv";

  std::ofstream csv(path, std::ios::out | std::ios::trunc);

  if (!csv.is_open())
    throw std::runtime_error("Could not open manifest: " + path);

  csv << std::scientific << std::setprecision(12);
  csv << "run,domain,time_step_s,correction,event_time_s,opening_time_s,"
      << "analytic_zero_s\n";

  for (const auto &r : records)
    csv << r.name << "," << r.domain << "," << r.timeStep << ","
        << static_cast<Int>(r.correction) << "," << r.eventTime << ","
        << r.openingTime << "," << analyticZero << "\n";

  std::cout << "Manifest written to " << path << std::endl;
}

} // namespace

int main(int argc, char *argv[]) {
  if (argc > 1 && String(argv[argc - 1]) == "--interpolate") {
    zeroCrossingInterpolation = true;
    --argc;
  }
  const Parameters p;

  if (argc > 1 && String(argv[1]) == "--second-feeder") {
    runSecondFeeder(p);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--line-end") {
    runNetlistCase(p, "LineEnd", lineEnd);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--parallel-lines") {
    runNetlistCase(p, "ParallelLines", parallelLines);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--coupled-phases") {
    runCoupledPhases(p);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--cap-close") {
    runCapacitorClosing(p);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--cap-close-ph3") {
    runClosingPh3(p);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--cap-close-pow-ph3") {
    runClosingPowPh3(p);
    return EXIT_SUCCESS;
  }
  if (argc > 1 && String(argv[1]) == "--mesh-tie") {
    runNetlistCase(p, "MeshTie", meshTie);
    return EXIT_SUCCESS;
  }

  const Real analyticZero = analyticCurrentZero(p);

  std::cout << std::scientific << std::setprecision(9)
            << "Analytic steady-state current zero after the command: "
            << analyticZero << " s" << std::endl;

  std::vector<RunRecord> records;

  records.push_back(runEMT(p, p.referenceTimeStep, "reference"));
  records.push_back(runDP(p, p.referenceTimeStep, true, "reference"));
  records.push_back(runDP(p, p.referenceTimeStep, false, "reference"));

  for (const auto timeStep : p.sweep) {
    const String label = formatTimeStep(timeStep);
    records.push_back(runEMT(p, timeStep, label));
    records.push_back(runDP(p, timeStep, true, label));
    records.push_back(runDP(p, timeStep, false, label));
  }

  writeManifest(records, analyticZero);

  // Method comparison for the paper's comparison table, at a step of 50 us,
  // which does not resolve the recovery transient.
  runMethodComparison(p, 50e-6);

  for (const auto &r : records) {
    if (r.domain != "DP" || !r.correction)
      continue;

    std::cout << "dt " << r.timeStep << " s: DP found its current zero at "
              << r.openingTime << " s, "
              << (r.openingTime - analyticZero) / r.timeStep
              << " time steps from the analytic one" << std::endl;
  }

  runMixedModeCost(p);

  return EXIT_SUCCESS;
}
