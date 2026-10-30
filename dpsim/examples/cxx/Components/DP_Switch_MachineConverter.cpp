// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// The quadrature correction next to machines and converters, against EMT, on
// the second feeder case in three phases. Each breaker pole opens at its own
// current zero, in DP and in EMT.
//
//   --trstab     the source is a classical machine (TrStab)
//   --converter  a stiff source, and a grid-following converter at bus A
//   --syngendq   the source is a full order machine, which the correction
//                sees only through its stamp
//
// Each case writes an EMT run at 1 us and DP runs at 50 us with and without
// the correction. The analysis is external.

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <type_traits>

#include <DPsim.h>
#include <dpsim-models/DP/DP_Ph3_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/DP/DP_Ph3_Switch.h>
#include <dpsim-models/DP/DP_Ph3_SynchronGeneratorTrStab.h>
#include <dpsim-models/EMT/EMT_Ph3_AvVoltageSourceInverterDQ.h>
#include <dpsim-models/EMT/EMT_Ph3_Switch.h>

using namespace DPsim;
using namespace CPS;

namespace {

// Set by --interpolate as the last argument
Bool zeroCrossingInterpolation = false;

struct Parameters {
  Real frequency = 50.0;
  Real omega() const { return 2.0 * PI * frequency; }

  // Phase emf, peak
  Real emf = 20e3;
  Real transientInductance = 10e-3;
  Real nominalPower = 15e6;
  Real inertia = 0.5;

  Real shuntCapacitance = 1e-6;
  Real shuntResistance = 100.0;
  Real lineInductance = 50e-3;
  Real strayCapacitance = 125e-12;
  Real loadResistance = 50.0;
  Real feederResistance = 200.0;
  Real feederInductance = 20e-3;

  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 1e-3;
  Real zeroCrossingTolerance = 1e-6;

  // Converter case: the SGIB values of the DPsim examples, scaled from 1.5 kV
  // and 100 kW to 20 kV and 4 MW. Impedances scale with the base impedance,
  // the power and PLL gains with the voltage. The filter capacitance is 2 uF.
  Real sourceInductance = 10e-3;
  Real converterActivePower = 4e6;
  Real converterReactivePower = 1e6;
  Real impedanceScale = (20e3 * 20e3 / 4e6) / (1500. * 1500. / 100e3);
  Real voltageScale = 1500. / 20e3;
  Real filterInductance = 0.002 * impedanceScale;
  Real filterCapacitance = 2e-6;
  Real filterResistance = 0.1 * impedanceScale;
  Real couplingResistance = 0.1 * impedanceScale;
  Real kpPll = 0.25 * voltageScale;
  Real kiPll = 0.2 * voltageScale;
  Real kpPower = 0.001 * voltageScale;
  Real kiPower = 0.008 * voltageScale;
  Real kpCurrent = 0.3 * impedanceScale;
  Real kiCurrent = 1.0 * impedanceScale;

  // The converter's power loop settles slowly, so its case switches later
  Real converterCommandTime = 0.3;

  Real commandTime = 0.1;
  Real finalTime = 0.14;
  Real referenceTimeStep = 1e-6;
  Real timeStep = 50e-6;
};

/// Balanced steady state of the closed network, phase a, peak phasors
struct OperatingPoint {
  Complex busA, busB, busD;
  Complex power;
};

OperatingPoint operatingPoint(const Parameters &p) {
  const Real w = p.omega();
  const Complex j(0.0, 1.0);
  const Complex feeder = p.feederResistance + j * w * p.feederInductance;
  const Complex admittanceB =
      j * w * p.strayCapacitance + 1.0 / p.loadResistance + 1.0 / feeder;
  const Complex line = j * w * p.lineInductance;
  const Complex admittanceA = j * w * p.shuntCapacitance +
                              1.0 / p.shuntResistance +
                              1.0 / (line + 1.0 / admittanceB);
  const Complex reactance = j * w * p.transientInductance;

  OperatingPoint op;
  op.busA = p.emf / (1.0 + reactance * admittanceA);
  op.busB = op.busA / (1.0 + line * admittanceB);
  op.busD = op.busB;
  const Complex current = (Complex(p.emf, 0.0) - op.busA) / reactance;
  op.power = 1.5 * op.busA * std::conj(current);
  return op;
}

template <typename VarType> struct Models;
template <> struct Models<Complex> {
  using Resistor = DP::Ph3::Resistor;
  using Inductor = DP::Ph3::Inductor;
  using Capacitor = DP::Ph3::Capacitor;
  using Switch = DP::Ph3::Switch;
  using Generator = DP::Ph3::SynchronGeneratorTrStab;
  using Source = DP::Ph3::VoltageSource;
  using Converter = DP::Ph3::AvVoltageSourceInverterDQ;
  using FullOrderGenerator = DP::Ph3::SynchronGeneratorDQTrapez;
};
template <> struct Models<Real> {
  using Resistor = EMT::Ph3::Resistor;
  using Inductor = EMT::Ph3::Inductor;
  using Capacitor = EMT::Ph3::Capacitor;
  using Switch = EMT::Ph3::Switch;
  using Generator = EMT::Ph3::SynchronGeneratorTrStab;
  using Source = EMT::Ph3::VoltageSource;
  using Converter = EMT::Ph3::AvVoltageSourceInverterDQ;
  using FullOrderGenerator = EMT::Ph3::SynchronGeneratorDQTrapez;
};

template <typename VarType>
void runTrStab(const Parameters &p, Real timeStep, Bool correction) {
  using M = Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  std::ostringstream stepName;
  stepName << std::fixed << std::setprecision(0) << timeStep * 1e9 << "ns";
  const String simName =
      String("DP_Switch_MachineTrStab") + (dp ? "_DP_" : "_EMT_") +
      stepName.str() + (dp ? (correction ? "_corrected" : "_plain") : String());
  Logger::setLogDir("logs/" + simName);

  const OperatingPoint op = operatingPoint(p);

  auto busA = SimNode<VarType>::make("A", PhaseType::ABC);
  auto busB = SimNode<VarType>::make("B", PhaseType::ABC);
  auto busD = SimNode<VarType>::make("D", PhaseType::ABC);
  auto busE = SimNode<VarType>::make("E", PhaseType::ABC);
  busA->setInitialVoltage(op.busA);
  busB->setInitialVoltage(op.busB);
  busD->setInitialVoltage(op.busD);

  auto phase = [](Real value) {
    return Math::singlePhaseParameterToThreePhase(value);
  };

  auto generator = M::Generator::make("G", Logger::Level::off);
  // In per unit, because the SI setter takes H as kinetic energy (it does not
  // divide by the nominal power). The base impedance is nomVolt^2 / nomPower.
  const Real transientReactance =
      p.omega() * p.transientInductance * p.nominalPower / (p.emf * p.emf);
  generator->setStandardParametersPU(p.nominalPower, p.emf, p.frequency,
                                     transientReactance, p.inertia);
  generator->setInitialValues(op.power, op.power.real());
  generator->connect({busA});

  auto shuntC = M::Capacitor::make("C_A", Logger::Level::off);
  shuntC->setParameters(phase(p.shuntCapacitance));
  shuntC->connect({SimNode<VarType>::GND, busA});
  auto shuntR = M::Resistor::make("R_A", Logger::Level::off);
  shuntR->setParameters(phase(p.shuntResistance));
  shuntR->connect({SimNode<VarType>::GND, busA});

  auto line = M::Inductor::make("L_line", Logger::Level::off);
  line->setParameters(phase(p.lineInductance));
  line->connect({busA, busB});

  auto stray = M::Capacitor::make("C_B", Logger::Level::off);
  stray->setParameters(phase(p.strayCapacitance));
  stray->connect({SimNode<VarType>::GND, busB});

  auto feederR = M::Resistor::make("R_2", Logger::Level::off);
  feederR->setParameters(phase(p.feederResistance));
  feederR->connect({busB, busE});
  auto feederL = M::Inductor::make("L_2", Logger::Level::off);
  feederL->setParameters(phase(p.feederInductance));
  feederL->connect({busE, SimNode<VarType>::GND});

  auto breaker =
      M::Switch::make("Breaker", dp ? Logger::Level::info : Logger::Level::off);
  breaker->setParameters(phase(p.switchOpenResistance),
                         phase(p.switchClosedResistance), true);
  breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  breaker->connect({busB, busD});

  auto load = M::Resistor::make("R_D", Logger::Level::off);
  load->setParameters(phase(p.loadResistance));
  load->connect({busD, SimNode<VarType>::GND});

  auto system =
      SystemTopology(p.frequency, SystemNodeList{busA, busB, busD, busE},
                     SystemComponentList{generator, shuntC, shuntR, line, stray,
                                         feederR, feederL, breaker, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_A", busA->attribute("v"));
  logger->logAttribute("v_B", busB->attribute("v"));
  logger->logAttribute("i_line", line->attribute("i_intf"));
  logger->logAttribute("i_feeder", feederL->attribute("i_intf"));
  logger->logAttribute("i_gen", generator->attribute("i_intf"));
  logger->logAttribute("w_r", generator->attribute("w_r"));
  logger->logAttribute("delta_r", generator->attribute("delta_r"));
  logger->logAttribute("P_elec", generator->attribute("P_elec"));
  // Logged so the scheduler keeps the switch's post-step, which is where the
  // current zeros are detected
  logger->logAttribute("pole_a", breaker->attribute("pole_closed_a"));
  logger->logAttribute("pole_b", breaker->attribute("pole_closed_b"));
  logger->logAttribute("pole_c", breaker->attribute("pole_closed_c"));

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

/// An inductor the quadrature correction leaves alone while still solving the
/// network with it, to see whether correcting the supply path disturbs the
/// converter's control
class UncorrectedInductor : public DP::Ph3::Inductor {
public:
  using DP::Ph3::Inductor::Inductor;
  Bool mnaCorrectionAllowed() override { return false; }
};

/// A converter whose filter inductor the correction treats like any other,
/// to see what declaring it a current source changes
class UndeclaredConverter : public DP::Ph3::AvVoltageSourceInverterDQ {
public:
  using DP::Ph3::AvVoltageSourceInverterDQ::AvVoltageSourceInverterDQ;
  void mnaParentInitialize(Real omega, Real timeStep,
                           Attribute<Matrix>::Ptr leftVector) override {
    DP::Ph3::AvVoltageSourceInverterDQ::mnaParentInitialize(omega, timeStep,
                                                            leftVector);
    mSubInductorF->mnaCorrectionSetCurrentControlled(false);
  }
};

enum class ConverterRun {
  Plain,
  Corrected,
  SourceUncorrected,
  ConverterUndeclared
};

/// Balanced steady state of the converter case, phase a, peak phasors. The
/// converter injects a fixed power, so bus A is found by fixed-point
/// iteration.
struct ConverterOperatingPoint {
  Complex busA, busB;
  /// Power generated by the converter, three phase
  Complex power;
  /// Initial states of the power controller
  Real phiD, phiQ, gammaD, gammaQ;
};

ConverterOperatingPoint converterOperatingPoint(const Parameters &p) {
  const Real w = p.omega();
  const Complex j(0.0, 1.0);
  const Complex feeder = p.feederResistance + j * w * p.feederInductance;
  const Complex admittanceB =
      j * w * p.strayCapacitance + 1.0 / p.loadResistance + 1.0 / feeder;
  const Complex line = j * w * p.lineInductance;
  const Complex admittanceA = j * w * p.shuntCapacitance +
                              1.0 / p.shuntResistance +
                              1.0 / (line + 1.0 / admittanceB);
  const Complex source = j * w * p.sourceInductance;

  ConverterOperatingPoint op;
  op.power = Complex(p.converterActivePower, p.converterReactivePower);
  op.busA = p.emf;
  for (int iteration = 0; iteration < 100; ++iteration) {
    const Complex injection = std::conj(2.0 / 3.0 * op.power / op.busA);
    op.busA = (Complex(p.emf, 0.0) / source + injection) /
              (1.0 / source + admittanceA);
  }
  op.busB = op.busA / (1.0 + line * admittanceB);

  // The converter's own initialization, repeated: current into the
  // component, then back through Rc, Cf, Lf and Rf to the source voltage
  const Complex current = -std::conj(2.0 / 3.0 * op.power / op.busA);
  const Complex vc = op.busA - current * p.couplingResistance;
  const Complex icf = vc * j * w * p.filterCapacitance;
  const Complex vf = vc - (current - icf) * j * w * p.filterInductance;
  const Complex vs = vf - (current - icf) * p.filterResistance;

  // In the power invariant dq frame of the PLL, aligned with bus A, a balanced
  // peak phasor X becomes sqrt(3/2) X e^{-j theta}. In steady state the power
  // integrators carry the current reference and the current integrators the
  // source voltage.
  const Complex rotation = std::polar(std::sqrt(1.5), -std::arg(op.busA));
  const Complex generated = -current * rotation;
  const Complex sourceDq = vs * rotation;
  op.phiD = generated.real() / p.kiPower;
  op.phiQ = generated.imag() / p.kiPower;
  op.gammaD = sourceDq.real() / p.kiCurrent;
  op.gammaQ = sourceDq.imag() / p.kiCurrent;
  return op;
}

template <typename VarType>
void runConverter(const Parameters &p, Real timeStep, ConverterRun mode) {
  using M = Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  const Bool correction = mode != ConverterRun::Plain;
  const Real commandTime = p.converterCommandTime;
  const Real finalTime = p.converterCommandTime + 0.04;
  std::ostringstream stepName;
  stepName << std::fixed << std::setprecision(0) << timeStep * 1e9 << "ns";
  const String simName =
      String("DP_Switch_Converter") + (dp ? "_DP_" : "_EMT_") + stepName.str() +
      (dp ? (mode == ConverterRun::Plain               ? "_plain"
             : mode == ConverterRun::Corrected         ? "_corrected"
             : mode == ConverterRun::SourceUncorrected ? "_sourcekept"
                                                       : "_undeclared")
          : String());
  Logger::setLogDir("logs/" + simName);

  const ConverterOperatingPoint op = converterOperatingPoint(p);
  // Both domains initialize their nodes from RMS line values
  const Real scale = 1.0 / RMS3PH_TO_PEAK1PH;

  auto src = SimNode<VarType>::make("S", PhaseType::ABC);
  auto busA = SimNode<VarType>::make("A", PhaseType::ABC);
  auto busB = SimNode<VarType>::make("B", PhaseType::ABC);
  auto busD = SimNode<VarType>::make("D", PhaseType::ABC);
  auto busE = SimNode<VarType>::make("E", PhaseType::ABC);
  src->setInitialVoltage(Complex(p.emf, 0.0) * scale);
  busA->setInitialVoltage(op.busA * scale);
  busB->setInitialVoltage(op.busB * scale);
  busD->setInitialVoltage(op.busB * scale);

  auto phase = [](Real value) {
    return Math::singlePhaseParameterToThreePhase(value);
  };

  const MatrixComp sourceVoltage =
      Math::singlePhaseVariableToThreePhase(Complex(p.emf, 0.0));
  auto source = M::Source::make("vs", Logger::Level::off);
  if constexpr (std::is_same<VarType, Complex>::value)
    source->setParameters(sourceVoltage);
  else
    source->setParameters(sourceVoltage / RMS3PH_TO_PEAK1PH, p.frequency);
  source->connect({SimNode<VarType>::GND, src});

  std::shared_ptr<typename M::Inductor> sourceL;
  if constexpr (std::is_same<VarType, Complex>::value) {
    if (mode == ConverterRun::SourceUncorrected)
      sourceL =
          std::make_shared<UncorrectedInductor>("L_s", Logger::Level::off);
  }
  if (!sourceL)
    sourceL = M::Inductor::make("L_s", Logger::Level::off);
  sourceL->setParameters(phase(p.sourceInductance));
  sourceL->connect({src, busA});

  std::shared_ptr<typename M::Converter> converter;
  if constexpr (std::is_same<VarType, Complex>::value) {
    if (mode == ConverterRun::ConverterUndeclared)
      converter =
          std::make_shared<UndeclaredConverter>("pv", Logger::Level::off);
    else
      converter = M::Converter::make("pv", Logger::Level::off);
  } else
    converter = M::Converter::make("pv", "pv", Logger::Level::off, false);
  converter->setParameters(p.omega(), p.emf, p.converterActivePower,
                           p.converterReactivePower);
  converter->setControllerParameters(p.kpPll, p.kiPll, p.kpPower, p.kiPower,
                                     p.kpCurrent, p.kiCurrent, p.omega());
  converter->setFilterParameters(p.filterInductance, p.filterCapacitance,
                                 p.filterResistance, p.couplingResistance);
  converter->setInitialStateValues(op.power.real(), op.power.imag(), op.phiD,
                                   op.phiQ, op.gammaD, op.gammaQ);
  converter->connect({busA});

  auto shuntC = M::Capacitor::make("C_A", Logger::Level::off);
  shuntC->setParameters(phase(p.shuntCapacitance));
  shuntC->connect({SimNode<VarType>::GND, busA});
  auto shuntR = M::Resistor::make("R_A", Logger::Level::off);
  shuntR->setParameters(phase(p.shuntResistance));
  shuntR->connect({SimNode<VarType>::GND, busA});

  auto line = M::Inductor::make("L_line", Logger::Level::off);
  line->setParameters(phase(p.lineInductance));
  line->connect({busA, busB});

  auto stray = M::Capacitor::make("C_B", Logger::Level::off);
  stray->setParameters(phase(p.strayCapacitance));
  stray->connect({SimNode<VarType>::GND, busB});

  auto feederR = M::Resistor::make("R_2", Logger::Level::off);
  feederR->setParameters(phase(p.feederResistance));
  feederR->connect({busB, busE});
  auto feederL = M::Inductor::make("L_2", Logger::Level::off);
  feederL->setParameters(phase(p.feederInductance));
  feederL->connect({busE, SimNode<VarType>::GND});

  auto breaker =
      M::Switch::make("Breaker", dp ? Logger::Level::info : Logger::Level::off);
  breaker->setParameters(phase(p.switchOpenResistance),
                         phase(p.switchClosedResistance), true);
  breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  breaker->connect({busB, busD});

  auto load = M::Resistor::make("R_D", Logger::Level::off);
  load->setParameters(phase(p.loadResistance));
  load->connect({busD, SimNode<VarType>::GND});

  auto system = SystemTopology(
      p.frequency, SystemNodeList{src, busA, busB, busD, busE},
      SystemComponentList{source, sourceL, converter, shuntC, shuntR, line,
                          stray, feederR, feederL, breaker, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_A", busA->attribute("v"));
  logger->logAttribute("v_B", busB->attribute("v"));
  logger->logAttribute("i_line", line->attribute("i_intf"));
  logger->logAttribute("i_source", sourceL->attribute("i_intf"));
  logger->logAttribute("i_conv", converter->attribute("i_intf"));
  logger->logAttribute("Vc_d", converter->attribute("Vc_d"));
  logger->logAttribute("Vc_q", converter->attribute("Vc_q"));
  logger->logAttribute("Irc_d", converter->attribute("Irc_d"));
  logger->logAttribute("Irc_q", converter->attribute("Irc_q"));
  logger->logAttribute("pole_a", breaker->attribute("pole_closed_a"));
  logger->logAttribute("pole_b", breaker->attribute("pole_closed_b"));
  logger->logAttribute("pole_c", breaker->attribute("pole_closed_c"));

  Simulation sim(simName, dp ? Logger::Level::info : Logger::Level::off);
  sim.setSystem(system);
  sim.addLogger(logger);
  sim.setDomain(dp ? Domain::DP : Domain::EMT);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(timeStep);
  sim.setFinalTime(finalTime);
  if (dp) {
    sim.doQuadratureCorrection(correction);
    sim.doZeroCrossingInterpolation(zeroCrossingInterpolation);
  }
  sim.addEvent(SwitchEvent3Ph::make(commandTime, breaker, false));
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

/// The full order machine case. The machine parameters are the per unit set of
/// DP_EMT_SynGenDq7odTrapez_ThreePhFault on a 50 MVA base; the machine sees
/// bus A at 20 kV peak and supplies whatever the network draws there.
template <typename VarType>
void runFullOrderMachine(const Parameters &p, Real timeStep, Bool correction) {
  using M = Models<VarType>;
  const Bool dp = std::is_same<VarType, Complex>::value;
  std::ostringstream stepName;
  stepName << std::fixed << std::setprecision(0) << timeStep * 1e9 << "ns";
  const String simName =
      String("DP_Switch_MachineDQ") + (dp ? "_DP_" : "_EMT_") + stepName.str() +
      (dp ? (correction ? "_corrected" : "_plain") : String());
  Logger::setLogDir("logs/" + simName);

  const Real w = p.omega();
  const Complex j(0.0, 1.0);
  const Complex feeder = p.feederResistance + j * w * p.feederInductance;
  const Complex admittanceB =
      j * w * p.strayCapacitance + 1.0 / p.loadResistance + 1.0 / feeder;
  const Complex line = j * w * p.lineInductance;
  const Complex admittanceA = j * w * p.shuntCapacitance +
                              1.0 / p.shuntResistance +
                              1.0 / (line + 1.0 / admittanceB);
  const Complex voltageA(p.emf, 0.0);
  const Complex voltageB = voltageA / (1.0 + line * admittanceB);
  const Complex power = 1.5 * voltageA * std::conj(voltageA * admittanceA);
  const Real scale = 1.0 / RMS3PH_TO_PEAK1PH;

  auto busA = SimNode<VarType>::make("A", PhaseType::ABC);
  auto busB = SimNode<VarType>::make("B", PhaseType::ABC);
  auto busD = SimNode<VarType>::make("D", PhaseType::ABC);
  auto busE = SimNode<VarType>::make("E", PhaseType::ABC);
  busA->setInitialVoltage(voltageA * scale);
  busB->setInitialVoltage(voltageB * scale);
  busD->setInitialVoltage(voltageB * scale);

  auto phase = [](Real value) {
    return Math::singlePhaseParameterToThreePhase(value);
  };

  auto generator = M::FullOrderGenerator::make("G", Logger::Level::off);
  generator->setParametersFundamentalPerUnit(
      50e6, p.emf / RMS3PH_TO_PEAK1PH, p.frequency, 2, 1300, 0.003, 0.15,
      1.6599, 1.61, 0.0006, 0.1648, 0.0284, 0.1713, 0.0062, 0.7252, 0.0237,
      0.125, 3.7, power.real(), power.imag(), p.emf, 0.0, power.real());
  generator->connect({busA});

  auto shuntC = M::Capacitor::make("C_A", Logger::Level::off);
  shuntC->setParameters(phase(p.shuntCapacitance));
  shuntC->connect({SimNode<VarType>::GND, busA});
  auto shuntR = M::Resistor::make("R_A", Logger::Level::off);
  shuntR->setParameters(phase(p.shuntResistance));
  shuntR->connect({SimNode<VarType>::GND, busA});

  auto lineL = M::Inductor::make("L_line", Logger::Level::off);
  lineL->setParameters(phase(p.lineInductance));
  lineL->connect({busA, busB});

  auto stray = M::Capacitor::make("C_B", Logger::Level::off);
  stray->setParameters(phase(p.strayCapacitance));
  stray->connect({SimNode<VarType>::GND, busB});

  auto feederR = M::Resistor::make("R_2", Logger::Level::off);
  feederR->setParameters(phase(p.feederResistance));
  feederR->connect({busB, busE});
  auto feederL = M::Inductor::make("L_2", Logger::Level::off);
  feederL->setParameters(phase(p.feederInductance));
  feederL->connect({busE, SimNode<VarType>::GND});

  auto breaker =
      M::Switch::make("Breaker", dp ? Logger::Level::info : Logger::Level::off);
  breaker->setParameters(phase(p.switchOpenResistance),
                         phase(p.switchClosedResistance), true);
  breaker->setSwitchingMode(M::Switch::SwitchingMode::CurrentZero);
  breaker->setZeroCrossingTolerance(p.zeroCrossingTolerance);
  breaker->connect({busB, busD});

  auto load = M::Resistor::make("R_D", Logger::Level::off);
  load->setParameters(phase(p.loadResistance));
  load->connect({busD, SimNode<VarType>::GND});

  auto system = SystemTopology(
      p.frequency, SystemNodeList{busA, busB, busD, busE},
      SystemComponentList{generator, shuntC, shuntR, lineL, stray, feederR,
                          feederL, breaker, load});

  auto logger = DataLogger::make(simName);
  logger->logAttribute("v_A", busA->attribute("v"));
  logger->logAttribute("v_B", busB->attribute("v"));
  logger->logAttribute("i_line", lineL->attribute("i_intf"));
  logger->logAttribute("i_feeder", feederL->attribute("i_intf"));
  logger->logAttribute("i_gen", generator->attribute("i_intf"));
  logger->logAttribute("w_r", generator->attribute("w_r"));
  logger->logAttribute("pole_a", breaker->attribute("pole_closed_a"));
  logger->logAttribute("pole_b", breaker->attribute("pole_closed_b"));
  logger->logAttribute("pole_c", breaker->attribute("pole_closed_c"));

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

} // namespace

int main(int argc, char *argv[]) {
  if (argc > 1 && String(argv[argc - 1]) == "--interpolate") {
    zeroCrossingInterpolation = true;
    --argc;
  }
  const Parameters p;

  if (argc > 1 && String(argv[1]) == "--trstab") {
    const OperatingPoint op = operatingPoint(p);
    std::cout << std::scientific << std::setprecision(6)
              << "Operating point: |V_A| " << std::abs(op.busA) << " V, P "
              << op.power.real() << " W, Q " << op.power.imag() << " var"
              << std::endl;
    runTrStab<Real>(p, p.referenceTimeStep, false);
    runTrStab<Complex>(p, p.timeStep, true);
    runTrStab<Complex>(p, p.timeStep, false);
    return EXIT_SUCCESS;
  }

  if (argc > 1 && String(argv[1]) == "--converter") {
    const ConverterOperatingPoint op = converterOperatingPoint(p);
    std::cout << std::scientific << std::setprecision(6)
              << "Operating point: |V_A| " << std::abs(op.busA)
              << " V, converter P " << op.power.real() << " W, Q "
              << op.power.imag() << " var" << std::endl;
    runConverter<Real>(p, p.referenceTimeStep, ConverterRun::Plain);
    runConverter<Complex>(p, p.timeStep, ConverterRun::Corrected);
    runConverter<Complex>(p, p.timeStep, ConverterRun::SourceUncorrected);
    runConverter<Complex>(p, p.timeStep, ConverterRun::ConverterUndeclared);
    runConverter<Complex>(p, p.timeStep, ConverterRun::Plain);
    return EXIT_SUCCESS;
  }

  if (argc > 1 && String(argv[1]) == "--syngendq") {
    runFullOrderMachine<Real>(p, p.referenceTimeStep, false);
    runFullOrderMachine<Complex>(p, p.timeStep, true);
    runFullOrderMachine<Complex>(p, p.timeStep, false);
    return EXIT_SUCCESS;
  }

  std::cout << "Usage: DP_Switch_MachineConverter --trstab | --converter | "
               "--syngendq"
            << std::endl;
  return EXIT_FAILURE;
}
