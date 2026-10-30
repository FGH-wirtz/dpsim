// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// Three poles of a DP::Ph3 breaker clearing at their own current zeros over
// one cycle. Each pole reports its own event, and each correction changes only
// the phase that lost its path.

#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph3_Switch.h>

using namespace DPsim;
using namespace CPS;

namespace {

struct Parameters {
  Real frequency = 50.0;
  Real sourceVoltage = 20e3;
  Real sourceResistance = 1.0;
  Real lineInductance = 50e-3;
  Real loadResistance = 50.0;
  Real strayCapacitance = 125e-12;

  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 1e-3;
  Real zeroCrossingTolerance = 1e-6;

  Real timeStep = 50e-6;
  Real commandTime = 0.1;
  Real finalTime = 0.14;
};

} // namespace

int main(int argc, char *argv[]) {
  Parameters p;

  const String simName = "DP_Switch_Ph3PerPoleClearing";
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::ABC);
  auto n4 = SimNode<Complex>::make("n4", PhaseType::ABC);

  const Complex phaseVoltage(p.sourceVoltage, 0.0);
  n1->setInitialVoltage(Math::singlePhaseVariableToThreePhase(phaseVoltage));

  auto vs = DP::Ph3::VoltageSource::make("vs", Logger::Level::off);
  // No source frequency: DP::Ph3::VoltageSource takes srcFreq relative to the
  // shift frequency, so 50 Hz here would run the network at 100 Hz.
  vs->setParameters(Math::singlePhaseVariableToThreePhase(phaseVoltage));

  auto rSource = DP::Ph3::Resistor::make("R_src", Logger::Level::off);
  rSource->setParameters(
      Math::singlePhaseParameterToThreePhase(p.sourceResistance));

  auto lLine = DP::Ph3::Inductor::make("L_line", Logger::Level::off);
  lLine->setParameters(
      Math::singlePhaseParameterToThreePhase(p.lineInductance));

  auto breaker = DP::Ph3::Switch::make("Breaker", Logger::Level::info);
  breaker->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance), true);
  breaker->setSwitchingMode(DP::Ph3::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  auto rLoad = DP::Ph3::Resistor::make("R_load", Logger::Level::off);
  rLoad->setParameters(
      Math::singlePhaseParameterToThreePhase(p.loadResistance));

  auto cStray = DP::Ph3::Capacitor::make("C_stray", Logger::Level::off);
  cStray->setParameters(
      Math::singlePhaseParameterToThreePhase(p.strayCapacitance));

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
  logger->logAttribute("pole_a", breaker->attribute("pole_closed_a"));
  logger->logAttribute("pole_b", breaker->attribute("pole_closed_b"));
  logger->logAttribute("pole_c", breaker->attribute("pole_closed_c"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.doSystemMatrixRecomputation(true);
  sim.doQuadratureCorrection(true);
  sim.addEvent(SwitchEvent3Ph::make(p.commandTime, breaker, false));

  const auto poles =
      std::array<String, 3>{"pole_closed_a", "pole_closed_b", "pole_closed_c"};
  const auto names = std::array<String, 3>{"A", "B", "C"};

  std::array<Real, 3> openingTime{{-1.0, -1.0, -1.0}};
  std::array<Bool, 3> wasClosed{{true, true, true}};

  sim.start();

  while (sim.time() < p.finalTime + DOUBLE_EPSILON) {
    sim.next();

    for (UInt phase = 0; phase < 3; ++phase) {
      const Bool closed = breaker->attributeTyped<Bool>(poles[phase])->get();
      if (wasClosed[phase] && !closed) {
        openingTime[phase] = sim.time();
        wasClosed[phase] = false;
      }
    }
  }

  sim.stop();
  sim.logLUTimes();

  std::ofstream manifest("logs/ph3_per_pole_manifest.csv");
  manifest << "run,phase,opening_time_s,command_time_s,time_step_s\n";
  manifest << std::scientific << std::setprecision(12);

  UInt cleared = 0;
  std::cout << "Three-pole clearing, command at " << p.commandTime << " s"
            << std::endl;
  for (UInt phase = 0; phase < 3; ++phase) {
    manifest << simName << "," << names[phase] << "," << openingTime[phase]
             << "," << p.commandTime << "," << p.timeStep << "\n";

    if (openingTime[phase] > 0.0) {
      ++cleared;
      std::cout << "  pole " << names[phase] << " cleared at "
                << std::scientific << std::setprecision(9) << openingTime[phase]
                << " s, " << (openingTime[phase] - p.commandTime) * 1e3
                << " ms after the command" << std::endl;
    } else {
      std::cout << "  pole " << names[phase] << " did not clear" << std::endl;
    }
  }

  if (cleared != 3) {
    std::cout << "Three-pole clearing: FAIL, " << cleared << " of 3 poles"
              << std::endl;
    return 1;
  }

  std::cout << "Three-pole clearing: all three poles cleared, statistics in "
               "logs/"
            << simName << std::endl;
  return 0;
}
