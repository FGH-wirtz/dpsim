---
title: "Switch and Load Implementation"
linkTitle: "Switches and Loads"
date: 2026-07-31
description: >
  How the switch and load models are arranged in code, and the traps in configuring them.
weight: 21
---

The models are derived under [switches]({{< ref "/docs/Concepts/Models/switches.md" >}}) and
[loads]({{< ref "/docs/Concepts/Models/loads.md" >}}). This page covers only their arrangement in
code. Availability per domain is in
[model availability]({{< ref "/docs/Reference/model-availability.md" >}}).

## Switches

`Switch` implements `Base::Ph1::Switch` (or `Base::Ph3::Switch`) and stamps one admittance chosen by
`mIsClosed`, using `MNAStampUtils::stampAdmittance` so the grounded-terminal cases are handled
centrally. `SeriesSwitch` folds a series resistance into the same branch.

### Switching modes

`EMT::Ph3::Switch`, `DP::Ph3::Switch` and `DP::Ph1::Switch` carry a `SwitchingMode` enum — `Ideal`,
`CurrentZero`, `ExponentialZCSEmulation` — set through `setSwitchingMode()`, with
`setZeroCrossingTolerance()` and `setExponentialSwitchingTime()` for the two non-ideal modes. The
behaviour behind them is derived under [switches]({{< ref "/docs/Concepts/Models/switches.md" >}}).

The distinction that runs through the implementation is **commanded state versus physical state**.
`mIsClosed` is what the command asked for; `pole_closed_a/b/c` (`pole_closed` in `DP::Ph1`) is what
the poles are actually doing, and `mnaIsClosed()` reports the commanded state only in `Ideal` mode
and the pole state otherwise. `Base::Ph{1,3}::Switch::closeSwitch()` / `openSwitch()` are virtual for
exactly this reason: the override records the command and lets the post-step decide when the poles
follow. `Base::Ph1::Switch::close()` / `open()` are non-virtual delegates to them, so `SwitchEvent`
and the Python bindings need no change.

The non-ideal modes return `supportsPrecomputedSystemMatrices() == false` and drive the matrix
through `MNAVariableCompInterface::hasParameterChanged()` instead — per pole in `CurrentZero`, per
resistance value in `ExponentialZCSEmulation`, which means one refactorisation per step of the ramp.
`effective_resistance_*` is the value actually stamped and is the attribute to log when debugging a
transition; `exponential_progress`, `exponential_transition_active` and the start/end times are the
rest of the diagnostics.

{{% alert title="Watch out: the stamped resistance is one step ahead" color="warning" %}}
`updateExponentialTransition()` evaluates the ramp at `time + mTimeStep`, because the resistance it
writes is the one the *next* solve uses. `mTimeStep` is captured in `mnaCompInitialize`, so a switch
that never went through MNA initialisation would ramp against a zero step size.
{{% /alert %}}

In DP, `reconstructInstantaneousCurrent()` restores the carrier removed by the DP formulation. Note
the scaling asymmetry between the two DP variants: `DP::Ph3` envelopes are phase-peak and are built
from power flow with `RMS3PH_TO_PEAK1PH`, while `DP::Ph1` keeps the power-flow scaling, so the
reconstructed `i_instantaneous` of `DP::Ph1` is in envelope scaling. The zero-crossing instant does
not depend on it, which is why the detection is shared.

### varResSwitch

`varResSwitch` also implements `MNAVariableCompInterface`, which is what allows it to change
the system matrix during a run. Its `hasParameterChanged` is called each step and drives the
transition:

- Opening multiplies the resistance by `mDeltaResOpen` each step until it passes the target open
  value, then clamps to it and reports the transition finished.
- Closing uses `mDeltaResClosed`, which is `0`, so the first step takes the resistance to zero, the
  clamp catches it and sets the closed value. Closing is therefore immediate by construction, not by
  a separate code path.

{{% alert title="Watch out: setInitParameters is mandatory" color="warning" %}}
`setInitParameters(timestep)` must be called before the simulation, because the growth factor is
derived from the step size as `0.5 * timestep / 0.001 + 1`. It also captures the configured
resistances as the transition targets, since the live attributes are overwritten during the ramp. If
it is not called, `mDeltaResOpen` keeps its default of `1.5`, which is the value for a 1 ms step and
wrong for any other.
{{% /alert %}}

Its `initializeFromNodesAndTerminals` carries a comment saying it is not used.

New models should prefer `ExponentialZCSEmulation` on the plain `Switch`, which covers the same
ground with a parameterised duration and without rewriting its own resistance attributes.

## Quadrature correction

In DP, a switching event leaves the quadrature part of the pole current or
voltage in the companion history terms of the inductors and capacitors, and
that leftover appears as a sustained alternating error. `MNAQuadratureCorrector`
(`dpsim/include/dpsim/MNAQuadratureCorrector.h`) removes it once per event.

### What components supply

`MNAQuadratureCorrectionInterface` (`dpsim-models/include/dpsim-models/Solver/`)
is deliberately small: which quantity the component holds, the stamped companion
admittance, read/write access to the state, the element value, the terminal node
indices, and a hook to rebuild the component's own right-hand-side contribution.
**The corrector writes states only.** Each component rebuilds its own history, so
no component ever reaches into another component's history term.

Implemented by the `DP::Ph1` and `DP::Ph3` inductors and capacitors. A
three-phase component contributes one state per phase, and coupling between those
states is reported through `mnaCorrectionCouplingAdmittance(row, col)` and
`mnaCorrectionCouplingElement(row, col)`. Both default to the uncoupled case ---
the existing scalar on the diagonal, zero off it --- so a component with a
diagonal `L` or `C` implements neither, and the diagonal path costs nothing.
Components that do not implement the interface are invisible to the corrector,
which is why EMT results are unchanged.

{{% alert title="Watch out: collection happens after component initialization" color="warning" %}}
Correctable states are collected in `initializeQuadratureCorrector()`, not in
`identifyTopologyObjects()`. Composite components such as `PiLine` and `RXLoad`
only create their sub-components during `initializeFromNodesAndTerminals()`, so at
topology-identification time their sub-component lists are still empty and a
series inductor inside a `PiLine` would never be found. The collection is
recursive through `mnaSubComponents()` for the same reason.
{{% /alert %}}

{{% alert title="Watch out: a composite's right vector is an aggregate" color="warning" %}}
The solver's `mRightVectorStamps` holds the *parent* composite's right vector,
which `CompositePowerComp::mnaCompApplyRightSideVectorStamp` rebuilds by summing
its sub-components. Updating a sub-component's own vector is therefore invisible
until the parent re-aggregates, so `applyQuadratureCorrection()` re-runs the
right-side stamp of every top-level component before re-solving. The stamps are
pure functions of state and time, so re-running them changes nothing else.
{{% /alert %}}

### Where it runs

`MnaSolverDirect::applyQuadratureCorrection()` is called from both solve paths,
guarded by `anySwitchNeedsCorrection()`. Two things have to hold: the switch opts
into the correction through `MNASwitchInterface::requestsQuadratureCorrection()`,
which defaults to false and is true for the DP switches in `CurrentZero` only,
and it reports having just turned a pole through `takeSwitchingEvent()`.

The event is reported by the switch rather than deduced by the solver, because
`mnaIsClosed()` on a three-phase switch is `allPolesClosed()` and stops changing
after the first pole, so the solver cannot see the later ones. `setPoleClosed()`
sets the record where the pole actually turns, which makes it independent of the
solve path and of `hasParameterChanged()` having already consumed the change.
Reading the record clears it, so `anySwitchNeedsCorrection()` asks every switch
exactly once per solve and must itself be called exactly once -- a switch left
holding a record would otherwise trigger a correction on an unrelated later
step.

The recomputation path additionally requires `mVariableComponentChanged`, since
the correction has to run against the post-event matrix; a pole change is
exactly what makes the switch report a parameter change, so the two coincide.

{{% alert title="In practice only the recomputation path corrects" color="warning" %}}
Both DP switches return `supportsPrecomputedSystemMatrices() == false` in
`CurrentZero`, which is the only mode that opts into the correction, so a network
containing one is forced onto the recomputation path. The call in the
precomputed-matrix path is there for a switch class that might opt in while
supporting the bitset, and is not otherwise exercised.
{{% /alert %}}

The gate is deliberately independent of the bitset itself, so it also works when
there are more switches than bits. When a correction occurs the right-hand side
is re-assembled and the step is solved again, so the correction lands in the same
time step rather than one step later.

### Opening at a current zero

At a current zero nothing physical is inconsistent. Write an envelope at the
event instant as `X e^{jwt} = x + j y`. For a linear network the in-phase
part `x` and the quadrature part `y` obey the same circuit separately, driven by
the cosine and the sine of the source, and only `x` is the physical result. At
the current zero `x_sw = 0` but `y_sw = |I_sw|`. The companion model of an
inductor stores the envelope in its history term, so after the opening that term
still drives the full `|I_sw| / sqrt(1 + b^2)` into the network, with
`b = w dt / 2`, at any time step. In EMT the history term holds the instantaneous
current, which is zero there. On the measurement network that is the whole DP
error, several hundred percent from `50 us` down to `1 us`.

The switch reports each opened pole through
`MNASwitchInterface::takeSwitchedPoles()`: its terminal nodes, its closed and
open conductance, and the current it carried on the last closed step. The
corrector then solves the post-event network once at the shift frequency for
that current injected across the opened pole,

```math
Y_{50}\,\Delta v = u\, I_{sw}, \qquad \Delta I_k = Y_{50,k}\,\Delta v_k,
```

and adds the quadrature part of `dI_k` to every inductor state. The carrier is
taken at the step the pole opened on, `c_k = e^{j w t_k}`, the step whose states
are corrected:

```math
I_k \leftarrow I_k + j\,\mathrm{Im}\{\Delta I_k c_k\}\, \overline{c_k}.
```

DPsim opens on the first step after the sign change, in EMT and DP alike, so
`t_k` lies up to one step after the exact current zero `t0`. The pole then keeps
the in-phase current `Re{I_sw c_k}` it carries at `t_k`, which the opening chops
as in EMT. Taking the carrier at `t0` instead, `c0 = j conj(I_sw) / |I_sw|`,
would remove the whole interrupted current, but it changes the states of `t_k`
by `Im{dI_k c0} sin(w (t_k - t0))` in their physical part. That shift grows
with the step and with the share of the pole current a parallel or coupled
state takes: in a mutually coupled three-phase inductor it moved the healthy
phases by 1.07 A, with `c_k` by 0.011 A, and in a 110 kV load rejection case
with a 1 ms step it put DP 3ph behind EMT. With `c_k` the corrected DP run
chops the same residual current as EMT and is as accurate as EMT at the same
step; it removes only the envelope's quadrature part. The in-phase part of
`dI_k` is a real change of current, which the transient after the event brings
about by itself.

`Y_50` is the step matrix with every correctable element's companion admittance
replaced by `mnaCorrectionSteadyStateAdmittance()`, which defaults to the inverse
of `j w L` for an inductor and to `j w C` for a capacitor. Everything that is not
correctable keeps its stamp. It is factorised with Eigen's `SparseLU`, once per
event.

There is no threshold: the 50 Hz network decides which states carried the
interrupted current and by how much. With two parallel lines it splits the
current in inverse proportion to their inductances, and a feeder at the
line-end bus that never carried it is left alone.

Machines and converters take part through two hooks.

- A state held by a controller on the time scale of the event is declared with
  `mnaCorrectionSetCurrentControlled(true)`. It then enters the steady-state
  solve as a current source and is not corrected. `AvVoltageSourceInverterDQ`
  (`DP::Ph1` and `DP::Ph3`) sets it on its filter inductor while its control is
  on. A machine modelled as an emf behind an inductance, such as
  `SynchronGeneratorTrStab`, needs nothing: its inductor is corrected like any
  other.
- A component without correctable states that injects a current computed from
  its own states, such as `DP::Ph3::SynchronGeneratorDQ`, stamps nothing and
  would look like an open circuit to the solve. It implements
  `MNAEventStampInterface::mnaApplyEventStamp()` and adds its stator resistance
  in series with the mean subtransient inductance at the shift frequency. The
  solver collects such components and hands them to the corrector with
  `setEventStamps()`.

`DP_Switch_MachineConverter` (`dpsim/examples/cxx/Components/`) compares these
cases with EMT::Ph3 on a three-phase feeder whose breaker opens each pole at its
own current zero: `--trstab`, `--converter` and `--syngendq`.

### Zero-crossing interpolation (optional)

`Simulation::doZeroCrossingInterpolation()` (off by default) treats an opening
as EMT programs treat a switching instant between two steps. The corrector keeps
the interface voltage and current of every correctable component for the last
two steps (`storeStep()`, before every step). When poles open, it
interpolates these states linearly to the earliest interpolated current zero
`t0` of the opening poles (`SwitchedPole::zeroTime`), takes each pole current
there and corrects the quadrature part with the carrier of each pole's own zero,
`c0 = j conj(I_sw) / |I_sw|`, so `Re{I_sw c0} = 0` and no current is chopped.
The post-event network is then stepped once from `t0` with the step's own
factorisation, and the states are interpolated back to the step the poles
opened on. How the interrupted current divides between inductors and
capacitors follows from that step, without a threshold. Controller, machine and
converter internal states are not rewound. Events with a closing pole use the
normal correction. The cost is one additional solve per opening event and a copy
of the storage states per step. The TOV example takes `-o qc=interp`, the
switch examples `--interpolate` as their last argument.

EMT has no envelope, so an EMT event is not corrected. The statistics
report the factorisations, one per event, as `quadrature
solves` and the written states as `quadrature corrections`.

### Closing: the same correction

Closing is the dual case. At a zero of the pole voltage `x_sw = 0` but
`y_sw = |V_sw|`, so the quadrature circuit sees the full pole voltage shorted
and the capacitor that holds it stores the full envelope `|V_sw|` in its
history term. With `dG` the change of pole
conductance, opening and closing are one equation,

```math
Y_{50}\,\Delta v = -u\,\Delta G\,(v_1 - v_0),
```

which for an opening is the injection of `I_sw` above. A `CurrentZero` switch
reports a closing pole through `takeSwitchedPoles()` with `closed = true` and
the pole voltage of the previous step. The voltage is read from the terminal
nodes, not from `v_intf`, because the switch's post-step is scheduled out when
nothing reads its attributes. The corrector then differs from the opening in
two points:

- The capacitor voltages take the quadrature part of a closing pole, the
  inductor currents only that of an opening pole. The closing error sits in
  the capacitor history term, so correcting only the inductors leaves the
  ring, and the inductor part of a closing adds nothing.
- The carrier is taken at the commanded instant, at which the pole closes.
  The correction
  holds for any closing angle; at the voltage peak it changes nothing, because
  there the quadrature part is zero.

On an opening the capacitors are left alone. Correcting them there as well was
measured on every opening case and gained nothing: the validation errors move
in the third digit, the recovery error at `5 us` rises from 0.021 % to 0.062 %,
and the WSCC9 fault case changes by 93 % of the pole voltage.

`DP_Switch_CorrectionMeasurements --cap-close` closes a `DP::Ph1` breaker onto
an uncharged 10 uF bank behind a reactor, alone (`CapSingle`) and next to an
energised bank (`CapBackToBack`), at 0, 30, 60 and 90 degrees of the pole
voltage, against EMT at `0.2 us`. Uncorrected, the back-to-back bank closed at
the voltage zero is off by 1092 A at `50 us` against 61 A for EMT at the same
step; corrected it is 61 A. At every angle and step the corrected error is
within 0.2 % of EMT's own error at the same step, except in the back-to-back
case at `50 us` and 30, 60 or 90 degrees, where it is 3.5 % below it; at 90
degrees the uncorrected run is below it by as much.
`--cap-close-ph3` closes the back-to-back case with a `DP::Ph3` breaker whose
three poles close together, pole a at its voltage zero and poles b and c at 60
degrees from theirs; each pole is reported and corrected in one solve.
`--cap-close-pow-ph3` closes the same case point-on-wave: `closePole(phase)` on
`DP::Ph3::Switch` and `EMT::Ph3::Switch` (`CurrentZero` mode only) closes one
pole, and each pole is commanded at its own voltage zero, a, c, b, 1/300 s apart,
rounded up to the step grid. For every step the EMT `0.2 us` reference closes at
the same rounded instants. Each pole is a separate event with its own solve.

`EMT::Ph1::Switch` has the same `CurrentZero` mode (`setSwitchingMode()`): after
`open()` the pole keeps conducting, the post-step detects the sign change of the
branch current, and the pole is open from the next step on (`pole_closed`,
`zero_crossing_time`). The single-phase EMT references in
`DP_Switch_CorrectionMeasurements` use it with the DP opening command, so EMT and
DP detect their own zero and both open on the step after it. EMT then interrupts
the current left at that step (0.115 A at `1 us`, 5.76 A at `50 us` in the
measurement network).

A closing that forms a loop of capacitors without an inductance is a different
problem, since then `x` itself jumps. Ideal-mode switches keep their behaviour;
only `CurrentZero` reports its poles.

### Test cases

`DP_Switch_GroupCorrection` (`dpsim/examples/cxx/Components/`) runs the
correction on groups of inductors that share the interrupted current: a stray
inductance in series with a large one, an interrupted two-inductor stub, and a
mutually coupled three-phase inductor with one pole opening at its current
zero. It asserts the outcome the network requires --- the series pair keeps its
current law, the interrupted stub goes to zero --- since the corrector is
solver-internal and the mechanism itself is only visible in the log. The coupled
case runs with and without the correction and requires the healthy phases'
instantaneous currents to agree within 1e-3 of the amplitude, since the run
without it matches an EMT reference there
(`DP_Switch_CorrectionMeasurements --coupled-phases`); the envelope's flux
linkage is no criterion, its quadrature part is not observable.

`DP_Switch_Ph3PerPoleClearing` opens a `DP::Ph3` breaker whose poles clear at
their own current zeros over one cycle. Each pole reports its own event, and
each event corrects only the phase that lost its path.

### Damped step, for comparison

`Simulation::setDampedStep(true)` takes the step after a switching event with
backward Euler instead of the trapezoidal rule, which is how Sana et al. treat a
discontinuity. It is there as a baseline to measure the correction against, not
as a default, and it needs system matrix recomputation.

`Solver::prepareStep()` runs between steps, after the events and before the
components' pre step tasks, so a rule change still reaches the history terms of
the step that follows. When a variable component changed during the previous
step, `MnaSolverDirect` calls `mnaSetIntegrationRule()` on every component,
re-stamps the whole matrix and factorises it, then switches back on the next
step. `partialRefactorize()` cannot be used, since the base matrix holds the
static components with the old rule.

`mnaSetIntegrationRule()` defaults to returning false, and the component stays
trapezoidal. Only `DP::Ph1::Inductor` and `DP::Ph1::Capacitor` implement it. The
solver logs how many components declined.

The damped step removes the undamped trapezoidal ringing but not the wrong state.
In `DP_Switch_CorrectionMeasurements` at `50 us`, against a `0.2 us` EMT
reference of 12.92 kV, current zero alone is 265 % off, the damped step 127 %
and the quadrature correction 5.3 %.

## Loads

`RXLoad` is a `CompositePowerComp`. In `initializeFromNodesAndTerminals` it converts the powers to
element values and builds sub-components:

- a resistor, **only if** the active power is non-zero
- an inductor if the reactance is positive, a capacitor if negative, and **nothing** if the reactive
  power is zero

{{% alert title="Watch out: a zero power silently drops a branch" color="warning" %}}
Each is registered with `addMNASubComponent` and connected between ground and the load terminal. The
conditionals are the trap: a load configured with `P` or `Q` at zero silently omits that branch. It
does not error, and the missing branch is only visible as a load that draws less than expected.
{{% /alert %}}

`PQLoadCS` wraps a current source and sets its reference in `updateSetPoint` from
`conj(S / mNomVoltage)`. The nominal voltage, not the terminal voltage, is deliberate; the line
using the terminal voltage is present but commented out. Changing it would make the component
nonlinear and require an iterative solve.

`Shunt` takes a conductance and a susceptance directly and additionally carries per-unit attributes,
since it is the form the powerflow solver consumes.

## Source

- Switches: `{SP,DP,EMT}_Ph{1,3}_Switch`, `DP_Ph3_SeriesSwitch`, `EMT_Ph3_SeriesSwitch`, `{DP,SP}_Ph1_varResSwitch` under `dpsim-models/src/`
- Loads: `DP_Ph1_RXLoad`, `EMT_Ph3_RXLoad`, `DP_Ph1_RXLoadSwitch`, `DP_Ph1_PQLoadCS`, `SP_Ph1_Load`, `{SP,DP}_Ph1_Shunt`, `EMT_Ph3_Shunt`
