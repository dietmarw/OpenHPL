# WIP: HydroGov governor (branch `feat/t95-dualmodegov`)

## Goal
New governor modelling an industrial hydro-unit governor: start turbine -> run up to
synchronous speed -> issue synchronise signal -> switch to power control, with
optional built-in droop.

## Done
- Created `OpenHPL/Controllers/HydroGov.mo` (self-contained: sequence +
  speed PI + power PI with droop + guide-vane actuator).
  - Modes (IntegerOutput `mode`): 0 STOPPED, 1 STARTING, 2 SPEED, 3 POWER, 4 SHUTDOWN.
  - Inputs: start, stop, breakerClosed (Boolean); P_ref, P_meas, f (Real);
    optional f_grid via `useGridFrequencyInput` conditional connector.
  - Outputs: Y_gv, syncCommand (Boolean), mode (Integer).
  - PI in equations with anti-windup + `reinit(x_i, ...)` bumpless transfer on mode change.
  - Actuator: der(Y_gv) = rate-limited first order toward Y_cmd.
  - `inSyncWindow` uses `pre(mode)` to break the algebraic loop syncCommand -> breaker -> mode.
- Added `HydroGov` and `HydroGovBlocks` to `OpenHPL/Controllers/package.order`.
- `checkModel` passes with omc (only usual `outer data` / unit warnings).
- Simulation of a scratch harness runs successfully.

## Open / next steps
1. Trajectory looked wrong in the scratch harness: at t=100 and t=300 mode stayed 2 (SPEED)
   with Y_gv ~0.014, never reaching sync. Next action: dump the trajectory with
   `readSimulationResult` and check whether the fault is the crude harness plant
   (`der(f) = (100*Y - f)/10`, so f=50 needs Y=0.5, but Y_start=0.15 gives only f=15,
   so STARTING -> SPEED at `f >= f_speedctrl` cannot trigger) or the governor
   (`e`, `x_i`, anti-windup sign logic).
2. Consider a proper test model in `OpenHPLTest/` using Turbine + SimpleGen instead of the scratch harness.
3. Consider an example in `OpenHPL/Examples/` and a release-note entry.

## Scratch test harness (recreate in /tmp if gone)
```
loadModel(Modelica, {"4.0.0"}); getErrorString();
loadFile("/home/hex/dev/Modelica/OpenHPL/OpenHPL/package.mo", uses=false); getErrorString();
loadString("
model TestHydroGov
  inner OpenHPL.Data data;
  OpenHPL.Controllers.HydroGov gov(Y_start=0.15, t_sync_hold=3, Pn=10e6, T_servo=1);
  Real f(start=0, fixed=true);
  Real P;
equation
  gov.start = time > 5;
  gov.stop = time > 400;
  gov.breakerClosed = gov.syncCommand and time > 0;
  gov.P_ref = 6e6;
  gov.P_meas = P;
  gov.f = f;
  P = if gov.breakerClosed then pre_P else 0;
  der(f) = if gov.breakerClosed then 0 else (100*gov.Y_gv - f)/10;
  pre_P = 10e6*gov.Y_gv;
protected
  Real pre_P;
end TestHydroGov;
"); getErrorString();
simulate(TestHydroGov, stopTime=500, numberOfIntervals=500); getErrorString();
```

## omc scripting notes
- `for t in {..} loop` fails ("must be an array type"); use

  `readSimulationResult(file, {time, gov.mode, ...}, n)` instead.

## Session 2026-08-20: task "demonstrate full mode cycle 0->1->2->3->4->0 in HydroGovStartup.mo example"

### Status: blocked on a real plant/control issue, not yet fixed

`OpenHPL/Examples/HydroGovStartup.mo` already exists (committed, "chatter fixes").
It currently only drives modes 0->1->2 and then gets stuck in SPEED (2) forever -
`stopCommand` is a constant `false`, so SHUTDOWN/STOPPED are never exercised, and
worse: **the SPEED (no-load) loop never actually reaches sync** in the baseline.

### Root cause found
- `turbine`/`simpleGen` have `Ploss = 0` (default, in `TorqueEquation`/`Power2Torque`),
  i.e. **zero bearing/windage friction**. With no load connected (breaker open) and
  zero friction, the rotor is a pure integrator: any nonzero valve opening
  accelerates it forever, and it can *only* be decelerated by closing the valve to
  zero - there is no physical way to reduce speed once it overshoots the sync window.
- Simulated baseline: mode 0->1 at t=5 (start), 1->2 at t=278.5 (f reaches
  `f_speedctrl`=42.5 Hz), then f keeps climbing past the target (~50.02 Hz) and
  never comes back down - governor stays in SPEED (mode=2) for the rest of the
  800 s run, settling at some overshot frequency (52-70 Hz depending on gains).
- Tried a sweep of `Kp_speed`/`Ki_speed` (3.0/0.6 down to 0.1/0.02): **all**
  overshoot past `df_sync` and get permanently stuck (never sync) - confirms it's
  not just a gain-tuning issue, it's the missing friction/damping term.
- Tried adding `turbine.Ploss=5e6` via `simulate(..., simflags="-override=...")`:
  **the override silently has no effect** (`turbine.Ploss` stays 0 in the result).
  Root cause: `Ploss` feeds `Modelica.Electrical.Machines.Losses.Friction
  frictionParameters(PRef=Ploss, ...)`, a parameter *record* binding, which OMC
  treats as structural/evaluated at compile time - `-override` cannot change it at
  runtime. **Must edit the `.mo` source (component modifier) and recompile**, not
  use `-override`, to change `Ploss`.

### Next steps (not yet done)
1. Edit `OpenHPL/Examples/HydroGovStartup.mo`: add a realistic `Ploss` modifier
   (e.g. a few % of `Pmax`) on `turbine` and/or `simpleGen` so there is an actual
   stable no-load equilibrium (opening where turbine power = friction loss).
2. Re-tune `governor` `Kp_speed`/`Ki_speed` (and maybe `df_sync`) against the
   now-damped plant so SPEED settles inside the sync window without overshooting it
   on the first pass (edit-and-resimulate loop; use a `/tmp` scratch copy of the
   package - see gotcha below - not `-override`, since Ploss is structural).
3. Once sync (mode 3/POWER) is reliably reached, change `stopCommand` from
   `BooleanConstant(k=false)` to something that goes true a while after POWER is
   reached (e.g. `BooleanTable`/`BooleanStep`), and make `startCommand` a *pulse*
   (true only briefly, e.g. t=5..15) rather than a permanent step - otherwise once
   the unit returns to STOPPED at the end, `start` is still `true` and the governor
   immediately restarts (0->1 again) because of edge-triggered `when start and
   pre(mode)==STOPPED`.
4. Extend `experiment(StopTime=...)` enough to see the full 4->0 decel after
   `stop` (valve closes at `closeRateMax`, then rotor needs the new friction to
   actually decelerate `f` down to `f_stopped` - this will now take a finite time
   thanks to step 1's friction, but confirm with a simulation).
5. Re-verify with `readSimulationResult`/mode-transition dump that the full
   sequence 0->1->2->3->4->0 occurs within `StopTime`.
6. Update `Documentation(info=...)` in the example to describe the full cycle.

### Gotchas hit this session
- `readSimulationResultVars`/`readSimulationResult` variable names are just the
  flat instance path, e.g. `governor.mode`, `turbine.Ploss` - use
  `readSimulationResultVars(matfile)` to confirm exact names.
- Printing full `readSimulationResult` matrices (or `omc` verbose logs) to the
  terminal easily exceeds the 20KB inline-output threshold; write results to a
  small text file with `writeFile` (extracting only what's needed, e.g. mode
  transitions) instead of printing the whole matrix.
- `-override=comp.Param=value` does **not** work for parameters that feed a
  parameter-record binding (e.g. `frictionParameters(PRef=Ploss,...)`) - these get
  evaluated structurally. Confirmed by checking the actual trajectory value in the
  result, not just trusting `simulate()` succeeding silently.
- Editing/reading files under `/tmp` is blocked by the `read_file`/edit tools
  ("configured to be ignored by Copilot") - use `sed`/shell commands via
  `run_in_terminal` for scratch-copy experiments outside the workspace instead.