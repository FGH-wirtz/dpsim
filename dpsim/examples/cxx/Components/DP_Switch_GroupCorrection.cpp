// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// Runs the quadrature correction on groups of inductors that share the
// interrupted current:
//   - mixed group: a stray inductance in series with a large one, whose
//     currents have to stay equal through the event,
//   - blocked group: two inductors on an interrupted stub, both going to zero,
//   - coupled phases: a mutually coupled three-phase inductor with one pole
//     opening at its own current zero, the healthy phases keeping their
//     instantaneous values.

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph1_Switch.h>
#include <dpsim-models/DP/DP_Ph3_Switch.h>

using namespace DPsim;
using namespace CPS;

namespace {

// Set by --interpolate as the last argument
Bool zeroCrossingInterpolation = false;

struct Parameters {
  Real frequency = 50.0;
  Real sourceVoltage = 20e3;
  Real timeStep = 50e-6;
  Real eventTime = 0.2;
  Real finalTime = 0.25;
  Real zeroCrossingTolerance = 1e-6;
  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 1e-3;

  Real lineResistance = 1.0;
  Real lineInductance = 0.1;
  Real strayInductance = 1e-6;
  Real loadResistance = 50.0;
  Real branchInductance = 0.5;

  Real stubResistance = 20.0;
  Real stubInductanceFirst = 1e-3;
  Real stubInductanceSecond = 2e-3;

  // Samples after the command in which the outcome is judged. The current zero
  // follows the command within a cycle, so this has to span one.
  Real judgeWindow = 0.03;

  // Coupled three-phase case. The mutual term is what the correction has to
  // account for; at 0.3 of the self inductance it is large enough that ignoring
  // it shows up plainly.
  Real selfInductance = 50e-3;
  Real mutualInductance = 15e-3;
  Real coupledLoadResistance = 50.0;
  // Steps after the opening over which the healthy phases are compared, short
  // of the next pole's zero
  std::size_t coupledCompareSteps = 20;
};

std::shared_ptr<DP::Ph1::Switch> makeBreaker(const Parameters &p) {
  auto breaker = DP::Ph1::Switch::make("Breaker", Logger::Level::info);
  breaker->setParameters(p.switchOpenResistance, p.switchClosedResistance,
                         true);
  breaker->setSwitchingMode(DP::Ph1::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  return breaker;
}

Complex envelope(const SimPowerComp<Complex>::Ptr &comp, const String &name) {
  return comp->attributeTyped<MatrixComp>(name)->get()(0, 0);
}

/// Largest value of |a - b| / reference over the samples, which is the relative
/// violation of the current law between two branches in series.
Real worstRatio(const std::vector<Complex> &a, const std::vector<Complex> &b,
                Real reference) {
  Real worst = 0.0;
  for (std::size_t idx = 0; idx < a.size() && idx < b.size(); ++idx)
    worst = std::max(worst, std::abs(a[idx] - b[idx]) / reference);
  return worst;
}

Real worstMagnitude(const std::vector<Complex> &samples) {
  Real worst = 0.0;
  for (const auto &sample : samples)
    worst = std::max(worst, std::abs(sample));
  return worst;
}

// The stray inductance is in series with the line, so the two currents have to
// stay equal through the event. Driving the stray current to zero shows up here as a
// violation of order one.
Bool runMixedGroup(const Parameters &p) {
  const String simName = "DP_Switch_GroupCorrection_MixedGroup";
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
  auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);
  auto n5 = SimNode<Complex>::make("n5", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::info);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto rLine = DP::Ph1::Resistor::make("R_line", Logger::Level::info);
  rLine->setParameters(p.lineResistance);

  auto lLine = DP::Ph1::Inductor::make("L_line", Logger::Level::info);
  lLine->setParameters(p.lineInductance);

  auto lStray = DP::Ph1::Inductor::make("L_stray", Logger::Level::info);
  lStray->setParameters(p.strayInductance);

  auto rLoad = DP::Ph1::Resistor::make("R_load", Logger::Level::info);
  rLoad->setParameters(p.loadResistance);

  auto breaker = makeBreaker(p);

  auto lBranch = DP::Ph1::Inductor::make("L_branch", Logger::Level::info);
  lBranch->setParameters(p.branchInductance);

  vs->connect({SimNode<Complex>::GND, n1});
  rLine->connect({n1, n2});
  lLine->connect({n2, n3});
  lStray->connect({n3, n4});
  rLoad->connect({n4, SimNode<Complex>::GND});
  breaker->connect({n4, n5});
  lBranch->connect({n5, SimNode<Complex>::GND});

  auto system = SystemTopology(
      p.frequency, SystemNodeList{n1, n2, n3, n4, n5},
      SystemComponentList{vs, rLine, lLine, lStray, rLoad, breaker, lBranch});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_line", lLine->attribute("i_intf"));
  logger->logAttribute("i_stray", lStray->attribute("i_intf"));
  logger->logAttribute("i_branch", lBranch->attribute("i_intf"));
  logger->logAttribute("v_breaker", breaker->attribute("v_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.addEvent(SwitchEvent::make(p.eventTime, breaker, false));

  std::vector<Complex> lineSamples;
  std::vector<Complex> straySamples;
  std::vector<Complex> branchSamples;
  Real preEventLine = 0.0;

  sim.start();

  while (sim.time() < p.finalTime + DOUBLE_EPSILON) {
    sim.next();

    if (sim.time() < p.eventTime)
      preEventLine = std::abs(envelope(lLine, "i_intf"));

    if (sim.time() >= p.eventTime &&
        sim.time() <= p.eventTime + p.judgeWindow) {
      lineSamples.push_back(envelope(lLine, "i_intf"));
      straySamples.push_back(envelope(lStray, "i_intf"));
      branchSamples.push_back(envelope(lBranch, "i_intf"));
    }
  }

  sim.stop();

  const Real preEventBranch = std::abs(branchSamples.front());
  const Real violation = worstRatio(straySamples, lineSamples, preEventLine);
  const Real branchResidual =
      std::abs(branchSamples.back()) / std::max(preEventBranch, 1e-12);

  const Bool passed = violation < 1e-2 && branchResidual < 1e-2;

  std::cout << "Mixed group: " << (passed ? "PASS" : "FAIL") << std::scientific
            << std::setprecision(6) << " (current law violation " << violation
            << " of a line current of " << preEventLine
            << " A, interrupted branch left at " << branchResidual
            << " of its pre-event current)" << std::endl;

  return passed;
}

// The stub carries real current and then loses its only path, so both of its
// inductors and every combination of them are blocked.
Bool runBlockedGroup(const Parameters &p) {
  const String simName = "DP_Switch_GroupCorrection_BlockedGroup";
  Logger::setLogDir("logs/" + simName);

  auto n1 = SimNode<Complex>::make("n1", PhaseType::Single);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::Single);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::Single);
  auto n4 = SimNode<Complex>::make("n4", PhaseType::Single);
  auto n5 = SimNode<Complex>::make("n5", PhaseType::Single);

  n1->setInitialVoltage(Complex(p.sourceVoltage, 0.0));

  auto vs = DP::Ph1::VoltageSource::make("vs", Logger::Level::info);
  vs->setParameters(Complex(p.sourceVoltage, 0.0));

  auto rLine = DP::Ph1::Resistor::make("R_line", Logger::Level::info);
  rLine->setParameters(p.lineResistance);

  auto rLoad = DP::Ph1::Resistor::make("R_load", Logger::Level::info);
  rLoad->setParameters(p.loadResistance);

  auto breaker = makeBreaker(p);

  auto lFirst = DP::Ph1::Inductor::make("L_stub_1", Logger::Level::info);
  lFirst->setParameters(p.stubInductanceFirst);

  auto lSecond = DP::Ph1::Inductor::make("L_stub_2", Logger::Level::info);
  lSecond->setParameters(p.stubInductanceSecond);

  auto rStub = DP::Ph1::Resistor::make("R_stub", Logger::Level::info);
  rStub->setParameters(p.stubResistance);

  vs->connect({SimNode<Complex>::GND, n1});
  rLine->connect({n1, n2});
  rLoad->connect({n2, SimNode<Complex>::GND});
  breaker->connect({n2, n3});
  lFirst->connect({n3, n4});
  lSecond->connect({n4, n5});
  rStub->connect({n5, SimNode<Complex>::GND});

  auto system = SystemTopology(
      p.frequency, SystemNodeList{n1, n2, n3, n4, n5},
      SystemComponentList{vs, rLine, rLoad, breaker, lFirst, lSecond, rStub});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_stub_1", lFirst->attribute("i_intf"));
  logger->logAttribute("i_stub_2", lSecond->attribute("i_intf"));
  logger->logAttribute("v_breaker", breaker->attribute("v_intf"));

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(p.finalTime);
  sim.addEvent(SwitchEvent::make(p.eventTime, breaker, false));

  std::vector<Complex> firstSamples;
  std::vector<Complex> secondSamples;
  Real preEventStub = 0.0;

  sim.start();

  while (sim.time() < p.finalTime + DOUBLE_EPSILON) {
    sim.next();

    if (sim.time() < p.eventTime)
      preEventStub = std::abs(envelope(lFirst, "i_intf"));

    if (sim.time() >= p.eventTime + p.judgeWindow) {
      firstSamples.push_back(envelope(lFirst, "i_intf"));
      secondSamples.push_back(envelope(lSecond, "i_intf"));
    }
  }

  sim.stop();

  const Real reference = std::max(preEventStub, 1e-12);
  const Real firstResidual = worstMagnitude(firstSamples) / reference;
  const Real secondResidual = worstMagnitude(secondSamples) / reference;

  const Bool passed = firstResidual < 1e-2 && secondResidual < 1e-2;

  std::cout << "Blocked group: " << (passed ? "PASS" : "FAIL")
            << std::scientific << std::setprecision(6)
            << " (pre-event stub current " << preEventStub << " A, left at "
            << firstResidual << " and " << secondResidual << " of it)"
            << std::endl;

  return passed;
}

// One pole of a three-phase breaker opens on a mutually coupled inductor, at its
// own current zero. The run is made with and without the correction.
struct CoupledRun {
  Int openedPhase = -1;
  Real openedTime = -1.0;
  MatrixComp beforeJump = MatrixComp::Zero(3, 1);
  MatrixComp afterJump = MatrixComp::Zero(3, 1);
  /// Instantaneous phase currents Re{I e^{j w t}} over the steps after the pole
  /// opened
  std::vector<std::array<Real, 3>> after;
};

CoupledRun simulateCoupledPhases(const Parameters &p, Bool correction) {
  const String simName = String("DP_Switch_GroupCorrection_CoupledPhases") +
                         (correction ? "" : "_plain");
  Logger::setLogDir("logs/" + simName);

  Matrix inductance = Matrix::Zero(3, 3);
  for (UInt row = 0; row < 3; ++row)
    for (UInt col = 0; col < 3; ++col)
      inductance(row, col) = row == col ? p.selfInductance : p.mutualInductance;

  auto n1 = SimNode<Complex>::make("n1", PhaseType::ABC);
  auto n2 = SimNode<Complex>::make("n2", PhaseType::ABC);
  auto n3 = SimNode<Complex>::make("n3", PhaseType::ABC);

  const Complex phaseVoltage(p.sourceVoltage, 0.0);
  n1->setInitialVoltage(Math::singlePhaseVariableToThreePhase(phaseVoltage));

  auto vs = DP::Ph3::VoltageSource::make("vs", Logger::Level::off);
  // srcFreq is relative to the shift frequency and defaults to zero. Passing
  // the system frequency would drive the envelope at 50 Hz on top of the
  // carrier and run this case at 100 Hz.
  vs->setParameters(Math::singlePhaseVariableToThreePhase(phaseVoltage));

  auto coupled = DP::Ph3::Inductor::make("L_coupled", Logger::Level::off);
  coupled->setParameters(inductance);

  auto breaker = DP::Ph3::Switch::make("Breaker", Logger::Level::info);
  breaker->setParameters(
      Math::singlePhaseParameterToThreePhase(p.switchOpenResistance),
      Math::singlePhaseParameterToThreePhase(p.switchClosedResistance), true);
  breaker->setSwitchingMode(DP::Ph3::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);

  auto load = DP::Ph3::Resistor::make("R_load", Logger::Level::off);
  load->setParameters(
      Math::singlePhaseParameterToThreePhase(p.coupledLoadResistance));

  vs->connect({SimNode<Complex>::GND, n1});
  coupled->connect({n1, n2});
  breaker->connect({n2, n3});
  load->connect({n3, SimNode<Complex>::GND});

  auto system = SystemTopology(p.frequency, SystemNodeList{n1, n2, n3},
                               SystemComponentList{vs, coupled, breaker, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("i_coupled", coupled->attribute("i_intf"));
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
  sim.doQuadratureCorrection(correction);
  sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  sim.addEvent(SwitchEvent3Ph::make(p.eventTime, breaker, false));

  const auto poles =
      std::array<String, 3>{"pole_closed_a", "pole_closed_b", "pole_closed_c"};
  const Real omega = 2.0 * PI * p.frequency;

  CoupledRun run;
  Bool awaitingCorrection = false;

  sim.start();

  while (sim.time() < p.finalTime + DOUBLE_EPSILON) {
    sim.next();

    const MatrixComp current =
        coupled->attributeTyped<MatrixComp>("i_intf")->get();

    // The pole turns in the post-step, and the correction runs in the step
    // after it, once the switch reports its parameter change.
    if (awaitingCorrection) {
      run.afterJump = current;
      awaitingCorrection = false;
    }

    if (run.openedPhase >= 0) {
      if (run.after.size() < p.coupledCompareSteps) {
        // After next() the time already points at the coming step, and the
        // envelope has to be turned with the instant it belongs to
        const Real instant = sim.time() - p.timeStep;
        std::array<Real, 3> instantaneous;
        for (UInt phase = 0; phase < 3; ++phase)
          instantaneous[phase] =
              (current(phase, 0) * std::polar(1.0, omega * instant)).real();
        run.after.push_back(instantaneous);
      }
      continue;
    }

    for (UInt phase = 0; phase < 3; ++phase) {
      if (breaker->attributeTyped<Bool>(poles[phase])->get())
        continue;

      run.openedPhase = static_cast<Int>(phase);
      run.openedTime = sim.time();
      run.beforeJump = current;
      awaitingCorrection = true;
      break;
    }
  }

  sim.stop();

  return run;
}

// The healthy phases are compared with the run without correction, which
// matches an EMT reference to about 1e-4 of the amplitude
// (DP_Switch_CorrectionMeasurements --coupled-phases).
Bool runCoupledPhases(const Parameters &p) {
  const CoupledRun corrected = simulateCoupledPhases(p, true);
  const CoupledRun plain = simulateCoupledPhases(p, false);

  if (corrected.openedPhase < 0 || plain.openedPhase < 0) {
    std::cout << "Coupled phases: FAIL (no pole opened)" << std::endl;
    return false;
  }

  const Int opened = corrected.openedPhase;
  const Real amplitude = std::abs(corrected.beforeJump(opened, 0));
  const Real residual =
      std::abs(corrected.afterJump(opened, 0)) / std::max(amplitude, 1e-12);

  Real deviation = 0.0;
  const std::size_t steps =
      std::min(corrected.after.size(), plain.after.size());
  for (std::size_t k = 0; k < steps; ++k)
    for (UInt phase = 0; phase < 3; ++phase)
      if (static_cast<Int>(phase) != opened)
        deviation = std::max(deviation, std::abs(corrected.after[k][phase] -
                                                 plain.after[k][phase]));

  const Real relativeDeviation = deviation / std::max(amplitude, 1e-12);
  const Bool passed = plain.openedPhase == opened && steps > 0 &&
                      relativeDeviation < 1e-3 && residual < 0.05;

  std::cout << "Coupled phases: " << (passed ? "PASS" : "FAIL")
            << std::scientific << std::setprecision(6) << " (phase " << opened
            << " opened at " << corrected.openedTime
            << " s, healthy instantaneous currents within " << deviation
            << " A of the run without correction, " << relativeDeviation
            << " of the amplitude, interrupted current left at " << residual
            << " of its pre-event value)" << std::endl;

  return passed;
}

} // namespace

int main(int argc, char *argv[]) {
  if (argc > 1 && String(argv[argc - 1]) == "--interpolate") {
    zeroCrossingInterpolation = true;
    --argc;
  }
  Parameters p;

  UInt passed = 0;

  if (runMixedGroup(p))
    ++passed;

  if (runBlockedGroup(p))
    ++passed;

  if (runCoupledPhases(p))
    ++passed;

  std::cout << "Group correction: " << passed << "/3 passed." << std::endl;

  return passed == 3 ? EXIT_SUCCESS : EXIT_FAILURE;
}
