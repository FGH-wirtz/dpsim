// SPDX-FileCopyrightText: 2026 Institute for Automation of Complex Power Systems, EONERC, RWTH Aachen University
// SPDX-License-Identifier: MPL-2.0

// Runs the quadrature correction on published grids, to measure its cost at
// scale and which states it changes. A breaker in series with an R-L branch,
// sized from the bus voltage, is attached to a spread of buses and opened one
// after another. IEEE118 is read from the Matpower file case118.m, so every
// bus starts from the solved power flow.
//
// Run from the repository root, or set CIMPATH; it also has to name the
// directory that holds case118.m.

#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <list>
#include <map>
#include <sstream>
#include <string>
#include <vector>

#include <DPsim.h>
#include <dpsim-models/CIM/Reader.h>
#include <dpsim-models/DP/DP_Ph1_Switch.h>

using namespace DPsim;
using namespace CPS;

namespace {

struct Grid {
  String name;
  Real frequency;
  String directory;
  std::vector<String> files;
  Bool matpower = false;
};

const std::vector<Grid> GRIDS{
    {"WSCC9",
     60.0,
     "build/_deps/cim-data-src/WSCC-09/WSCC-09",
     {"WSCC-09_DI.xml", "WSCC-09_EQ.xml", "WSCC-09_SV.xml", "WSCC-09_TP.xml"}},
    {"IEEE39",
     60.0,
     "build/_deps/cim-data-src/IEEE-39",
     {"Rootnet_FULL_NE_29J10h_DI.xml", "Rootnet_FULL_NE_29J10h_EQ.xml",
      "Rootnet_FULL_NE_29J10h_SV.xml", "Rootnet_FULL_NE_29J10h_TP.xml"}},
    {"CIGRE_MV",
     50.0,
     "build/_deps/cim-data-src/CIGRE_MV/NEPLAN/"
     "CIGRE_MV_no_tapchanger_With_LoadFlow_Results",
     {"Rootnet_FULL_NE_06J16h_DI.xml", "Rootnet_FULL_NE_06J16h_EQ.xml",
      "Rootnet_FULL_NE_06J16h_SV.xml", "Rootnet_FULL_NE_06J16h_TP.xml"}},
    {"case145",
     60.0,
     "build/_deps/cim-data-src/Matpower_cases",
     {"case145.xml"}},
    {"case300",
     60.0,
     "build/_deps/cim-data-src/Matpower_cases",
     {"case300.xml"}},
    {"IEEE118", 60.0, "", {"case118.m"}, true},
};

/// Rows of one numeric table of a Matpower case file, such as "mpc.bus"
std::vector<std::vector<Real>> readMatpowerTable(const fs::path &file,
                                                 const String &table) {
  std::ifstream in(file);
  if (!in)
    throw std::runtime_error("cannot open " + file.string());

  std::vector<std::vector<Real>> rows;
  String line;
  Bool inside = false;
  while (std::getline(in, line)) {
    const auto comment = line.find('%');
    if (comment != String::npos)
      line.erase(comment);
    if (!inside) {
      if (line.find(table + " = [") != String::npos)
        inside = true;
      continue;
    }
    if (line.find("];") != String::npos)
      break;
    for (auto &c : line)
      if (c == ';' || c == '\t')
        c = ' ';
    std::istringstream fields(line);
    std::vector<Real> row;
    Real value;
    while (fields >> value)
      row.push_back(value);
    if (!row.empty())
      rows.push_back(row);
  }
  if (rows.empty())
    throw std::runtime_error("no " + table + " table in " + file.string());
  return rows;
}

/// The Matpower case as a DP::Ph1 system. Voltages are line-to-line RMS
/// values and powers three-phase values, as the CIM reader uses them.
SystemTopology loadMatpower(const fs::path &file, Real frequency) {
  const Real baseMVA = 100.0;
  const Real omega = 2.0 * PI * frequency;
  const Real degree = PI / 180.0;

  const auto buses = readMatpowerTable(file, "mpc.bus");
  const auto gens = readMatpowerTable(file, "mpc.gen");
  const auto branches = readMatpowerTable(file, "mpc.branch");

  SystemTopology sys(frequency);
  std::map<Int, std::shared_ptr<CPS::DP::SimNode>> node;
  std::map<Int, Real> baseKV;

  for (const auto &b : buses) {
    const Int id = static_cast<Int>(b[0]);
    if (static_cast<Int>(b[1]) == 4)
      continue;
    baseKV[id] = b[9];
    auto n =
        CPS::DP::SimNode::make("BUS" + std::to_string(id), PhaseType::Single);
    n->setInitialVoltage(std::polar(b[7] * b[9] * 1e3, b[8] * degree));
    node[id] = n;
    sys.addNode(n);

    const Real voltage = b[7] * b[9] * 1e3;
    if (b[2] != 0.0 || b[3] != 0.0) {
      auto load = DP::Ph1::RXLoad::make("Load" + std::to_string(id),
                                        Logger::Level::off);
      load->setParameters(b[2] * 1e6, b[3] * 1e6, voltage);
      load->connect({n});
      sys.addComponent(load);
    }
    // Bus shunts are given in MW and MVAr consumed at 1 pu
    const Real admittanceBase = 1e6 / std::pow(b[9] * 1e3, 2);
    if (b[4] != 0.0) {
      auto r = DP::Ph1::Resistor::make("ShuntG" + std::to_string(id),
                                       Logger::Level::off);
      r->setParameters(1.0 / (b[4] * admittanceBase));
      r->connect({CPS::DP::SimNode::GND, n});
      sys.addComponent(r);
    }
    if (b[5] > 0.0) {
      auto c = DP::Ph1::Capacitor::make("ShuntC" + std::to_string(id),
                                        Logger::Level::off);
      c->setParameters(b[5] * admittanceBase / omega);
      c->connect({CPS::DP::SimNode::GND, n});
      sys.addComponent(c);
    } else if (b[5] < 0.0) {
      auto l = DP::Ph1::Inductor::make("ShuntL" + std::to_string(id),
                                       Logger::Level::off);
      l->setParameters(1.0 / (-b[5] * admittanceBase * omega));
      l->connect({CPS::DP::SimNode::GND, n});
      sys.addComponent(l);
    }
  }

  // One ideal source per generator bus, at the solved voltage
  std::map<Int, Bool> sourced;
  for (const auto &g : gens) {
    const Int id = static_cast<Int>(g[0]);
    if (g[7] <= 0.0 || !node.count(id) || sourced[id])
      continue;
    sourced[id] = true;
    auto source = DP::Ph1::VoltageSource::make("Gen" + std::to_string(id),
                                               Logger::Level::off);
    source->setParameters(node[id]->initialSingleVoltage());
    source->connect({CPS::DP::SimNode::GND, node[id]});
    sys.addComponent(source);
  }

  UInt index = 0;
  for (const auto &br : branches) {
    const Int from = static_cast<Int>(br[0]);
    const Int to = static_cast<Int>(br[1]);
    if (br[10] <= 0.0 || !node.count(from) || !node.count(to))
      continue;
    const String name = "Br" + std::to_string(index++) + "_" +
                        std::to_string(from) + "_" + std::to_string(to);
    const Real tap = br[8];
    const Real kvFrom = baseKV[from], kvTo = baseKV[to];

    if (tap == 0.0 && br[9] == 0.0) {
      const Real zBase = std::pow(kvFrom * 1e3, 2) / (baseMVA * 1e6);
      auto line = DP::Ph1::PiLine::make(name, Logger::Level::off);
      line->setParameters(br[2] * zBase, br[3] * zBase / omega,
                          br[4] / zBase / omega, 0.0);
      line->connect({node[from], node[to]});
      sys.addComponent(line);
      continue;
    }

    // Matpower puts the ideal tap at the from bus and the series impedance on
    // the to side. DPsim refers the impedance to terminal 0 and wants the
    // ratio of terminal 0 to terminal 1 at or above one, so pick the
    // orientation that gives that.
    const Real t = (tap == 0.0) ? 1.0 : tap;
    const Real shift = br[9] * degree;
    const Real ratioFrom = t * kvFrom / kvTo;
    auto transformer = DP::Ph1::Transformer::make(name, Logger::Level::off);
    if (ratioFrom >= 1.0) {
      const Real zBase = std::pow(kvFrom * 1e3, 2) / (baseMVA * 1e6);
      transformer->setParameters(kvFrom * 1e3, kvTo * 1e3, ratioFrom, shift,
                                 t * t * br[2] * zBase,
                                 t * t * br[3] * zBase / omega);
      transformer->connect({node[from], node[to]});
    } else {
      const Real zBase = std::pow(kvTo * 1e3, 2) / (baseMVA * 1e6);
      transformer->setParameters(kvTo * 1e3, kvFrom * 1e3, 1.0 / ratioFrom,
                                 -shift, br[2] * zBase, br[3] * zBase / omega);
      transformer->connect({node[to], node[from]});
    }
    sys.addComponent(transformer);
  }
  return sys;
}

struct Parameters {
  Real timeStep = 50e-6;
  Real firstCommand = 0.1;
  Real commandSpacing = 0.02;
  Real settle = 0.05;
  UInt breakerCount = 5;

  Real switchOpenResistance = 1e9;
  Real switchClosedResistance = 1e-3;
  Real zeroCrossingTolerance = 1e-6;

  /// Branch sizing: a current this large at 60 degrees lagging, from the bus's
  /// own initial voltage, so no grid-specific values are needed
  Real branchCurrent = 50.0;
  Real branchAngleDegrees = 60.0;
};

void runGrid(const Grid &grid, const Parameters &p) {
  const String simName = "DP_Switch_GridScaleProbe_" + grid.name;
  Logger::setLogDir("logs/" + simName);

  std::list<fs::path> files;
  for (const auto &f : grid.files)
    files.splice(files.end(),
                 DPsim::Utils::findFiles({f}, grid.directory, "CIMPATH"));

  SystemTopology sys(grid.frequency);
  if (grid.matpower) {
    if (files.empty())
      throw std::runtime_error("case file not found");
    sys = loadMatpower(files.front(), grid.frequency);
  } else {
    CPS::CIM::Reader reader(simName, Logger::Level::off, Logger::Level::off);
    sys = reader.loadCIM(grid.frequency, files, Domain::DP, PhaseType::Single,
                         CPS::GeneratorType::IdealVoltageSource);
  }

  const auto nodes = sys.mNodes;

  std::cout << grid.name << ": " << nodes.size() << " nodes, "
            << sys.mComponents.size() << " components" << std::endl;

  if (nodes.empty())
    throw std::runtime_error("no nodes");

  const Real omega = 2.0 * PI * grid.frequency;
  const Real angle = p.branchAngleDegrees * PI / 180.0;

  auto logger = DataLogger::make(simName);

  // Only buses the CIM data gives an initial voltage for can be sized from, so
  // the spread is taken over those rather than over the whole node list.
  std::vector<std::shared_ptr<CPS::DP::SimNode>> usable;
  for (const auto &raw : nodes) {
    auto node = std::dynamic_pointer_cast<CPS::DP::SimNode>(raw);
    if (node && std::abs(node->initialSingleVoltage()) >= 1.0)
      usable.push_back(node);
  }

  std::cout << "  " << usable.size() << " of " << nodes.size()
            << " buses carry an initial voltage" << std::endl;

  if (usable.empty())
    throw std::runtime_error("no bus carries an initial voltage");

  std::vector<std::shared_ptr<DP::Ph1::Switch>> breakers;

  UInt attached = 0;
  const UInt stride =
      std::max<UInt>(1, static_cast<UInt>(usable.size() / p.breakerCount));

  for (UInt idx = 0; attached < p.breakerCount && idx < usable.size();
       idx += stride) {
    auto node = usable[idx];

    const Real voltage = std::abs(node->initialSingleVoltage());
    const Real impedance = voltage / p.branchCurrent;
    const String tag = std::to_string(attached);

    auto sw = DP::Ph1::Switch::make("Breaker_" + tag, Logger::Level::info);
    sw->setParameters(p.switchOpenResistance, p.switchClosedResistance, true);
    sw->setSwitchingMode(DP::Ph1::Switch::SwitchingMode::CurrentZero);
    sw->setZeroCrossingTolerance(p.zeroCrossingTolerance);

    auto mid = CPS::DP::SimNode::make("probe_mid_" + tag, PhaseType::Single);
    auto tail = CPS::DP::SimNode::make("probe_tail_" + tag, PhaseType::Single);

    auto resistor =
        DP::Ph1::Resistor::make("Probe_R_" + tag, Logger::Level::off);
    resistor->setParameters(impedance * std::cos(angle));

    auto inductor =
        DP::Ph1::Inductor::make("Probe_L_" + tag, Logger::Level::off);
    inductor->setParameters(impedance * std::sin(angle) / omega);

    sw->connect({node, mid});
    resistor->connect({mid, tail});
    inductor->connect({tail, CPS::DP::SimNode::GND});

    sys.addNode(mid);
    sys.addNode(tail);
    sys.addComponent(sw);
    sys.addComponent(resistor);
    sys.addComponent(inductor);

    logger->logAttribute("i_probe_" + tag, inductor->attribute("i_intf"));
    logger->logAttribute("i_inst_" + tag, sw->attribute("i_instantaneous"));
    logger->logAttribute("pole_" + tag, sw->attribute("pole_closed"));
    breakers.push_back(sw);

    std::cout << "  breaker " << attached << " at node '" << node->name()
              << "', |V| = " << std::scientific << std::setprecision(3)
              << voltage << " V, |Z| = " << impedance << " Ohm" << std::endl;

    ++attached;
  }

  if (attached == 0)
    throw std::runtime_error("no usable bus found");

  const Real finalTime =
      p.firstCommand + attached * p.commandSpacing + p.settle;

  Simulation sim(simName, Logger::Level::info);
  sim.setSystem(sys);
  sim.addLogger(logger);
  sim.setDomain(Domain::DP);
  sim.setSolverType(Solver::Type::MNA);
  sim.setTimeStep(p.timeStep);
  sim.setFinalTime(finalTime);

  for (UInt idx = 0; idx < breakers.size(); ++idx)
    sim.addEvent(SwitchEvent::make(p.firstCommand + idx * p.commandSpacing,
                                   breakers[idx], false));

  sim.run();
  sim.logLUTimes();

  std::cout << "  " << attached << " breakers, final time " << finalTime
            << " s, statistics in logs/" << simName << std::endl;
}

} // namespace

int main(int argc, char *argv[]) {
  const Parameters p;

  std::vector<String> requested;
  for (int idx = 1; idx < argc; ++idx)
    requested.push_back(argv[idx]);

  for (const auto &grid : GRIDS) {
    if (!requested.empty() && std::find(requested.begin(), requested.end(),
                                        grid.name) == requested.end())
      continue;

    try {
      runGrid(grid, p);
    } catch (const std::exception &error) {
      std::cout << grid.name << ": skipped, " << error.what() << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
