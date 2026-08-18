within OpenHPL.Controllers;
model GovernorHydrotrol "Unit governor with start sequence, speed-no-load, synchronisation and power control"
  extends OpenHPL.Icons.Governor;
  outer Data data "Using standard class with constants";

  constant Integer STOPPED = 0 "Unit at standstill";
  constant Integer STARTING = 1 "Open-loop start opening";
  constant Integer SPEED = 2 "Speed-no-load control, ready to synchronise";
  constant Integer POWER = 3 "Grid connected power control";
  constant Integer SHUTDOWN = 4 "Closing guide vanes";

  parameter SI.Frequency f_n = data.f_0 "Rated (synchronous) frequency" annotation (
    Dialog(group = "System settings"));
  parameter SI.Power Pn = 104e6 "Rated active power" annotation (
    Dialog(group = "System settings"));

  parameter Boolean useGridFrequencyInput = false "Use measured grid frequency for synchronisation" annotation (
    choices(checkBox = true),
    Dialog(group = "System settings"));

  parameter Boolean enableDroop = true "Enable droop in power control mode" annotation (
    choices(checkBox = true),
    Dialog(group = "Droop"));
  parameter Real droop(min = 1e-3) = 0.06 "Permanent droop" annotation (
    Dialog(group = "Droop", enable = enableDroop));

  parameter Real Kp_speed = 3.0 "Speed loop proportional gain [pu opening / pu speed]" annotation (
    Dialog(group = "Speed control"));
  parameter Real Ki_speed = 0.6 "Speed loop integral gain [pu opening / (pu speed . s)]" annotation (
    Dialog(group = "Speed control"));
  parameter Real Kp_power = 0.6 "Power loop proportional gain [pu opening / pu power]" annotation (
    Dialog(group = "Power control"));
  parameter Real Ki_power = 0.1 "Power loop integral gain [pu opening / (pu power . s)]" annotation (
    Dialog(group = "Power control"));

  parameter Real Y_start = 0.25 "Start opening used until speed control takes over" annotation (
    Dialog(group = "Start sequence"));
  parameter SI.Frequency f_speedctrl = 0.85 * f_n "Speed where the start opening is released to speed control" annotation (
    Dialog(group = "Start sequence"));
  parameter SI.Frequency f_stopped = 0.05 * f_n "Speed below which the unit is considered stopped" annotation (
    Dialog(group = "Start sequence"));

  parameter SI.Frequency df_sync = 0.05 "Allowed speed deviation for the synchronising command" annotation (
    Dialog(group = "Synchronisation"));
  parameter SI.Frequency f_bias = 0.02 "Speed reference bias above grid frequency before synchronising" annotation (
    Dialog(group = "Synchronisation"));
  parameter SI.Time t_sync_hold = 5 "Time the speed must stay inside the window before the sync command" annotation (
    Dialog(group = "Synchronisation"));

  parameter SI.Time T_servo = 0.2 "Guide vane servo time constant" annotation (
    Dialog(group = "Actuator"));
  parameter Real openRateMax(unit = "1/s") = 0.05 "Maximum opening rate" annotation (
    Dialog(group = "Actuator"));
  parameter Real closeRateMax(unit = "1/s") = 0.2 "Maximum closing rate" annotation (
    Dialog(group = "Actuator"));
  parameter Real u_min = 0 "Minimum guide vane opening" annotation (
    Dialog(group = "Actuator"));
  parameter Real u_max = 1 "Maximum guide vane opening" annotation (
    Dialog(group = "Actuator"));
  parameter Real Y_gv_start = 0 "Initial guide vane opening" annotation (
    Dialog(group = "Initialization"));
  parameter Integer mode_start = STOPPED "Initial sequence state" annotation (
    Dialog(group = "Initialization"));

  Modelica.Blocks.Interfaces.BooleanInput start "Start command" annotation (
    Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput stop "Stop command" annotation (
    Placement(transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput breakerClosed "Generator breaker status" annotation (
    Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput P_ref(unit = "W") "Active power reference" annotation (
    Placement(transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput P_meas(unit = "W") "Measured active power" annotation (
    Placement(transformation(origin = {-120, -70}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -70}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f(unit = "Hz") "Measured unit speed as electrical frequency" annotation (
    Placement(transformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f_grid(unit = "Hz") if useGridFrequencyInput "Measured grid frequency" annotation (
    Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));

  Modelica.Blocks.Interfaces.RealOutput Y_gv(start = Y_gv_start, fixed = true) "Guide vane opening" annotation (
    Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
  Modelica.Blocks.Interfaces.BooleanOutput syncCommand "Request to close the generator breaker" annotation (
    Placement(transformation(extent = {{100, 50}, {120, 70}}), iconTransformation(extent = {{100, 50}, {120, 70}})));
  Modelica.Blocks.Interfaces.IntegerOutput mode(start = mode_start, fixed = true) "Active sequence state" annotation (
    Placement(transformation(extent = {{100, -70}, {120, -50}}), iconTransformation(extent = {{100, -70}, {120, -50}})));

  Real e "Control error in pu";
  Real y_pi "Unlimited controller output";

protected
  Modelica.Blocks.Interfaces.RealInput f_grid_int(unit = "Hz") "Internal grid frequency";
  Real Kp "Active proportional gain";
  Real Ki "Active integral gain";
  Real x_i(start = Y_gv_start, fixed = true) "Integral state";
  Real Y_cmd "Opening command before actuator";
  Boolean closedLoop "True when the PI controller drives the guide vanes";
  Boolean inSyncWindow "True while the speed is inside the synchronising window";
  discrete Real t_window(start = Modelica.Constants.inf, fixed = true) "Time when the speed entered the window";

equation
  connect(f_grid, f_grid_int);
  if not useGridFrequencyInput then
    f_grid_int = f_n;
  end if;

algorithm
  when {stop and pre(mode) <> STOPPED,
        start and pre(mode) == STOPPED,
        pre(mode) == STARTING and f >= f_speedctrl,
        pre(mode) == SPEED and breakerClosed,
        pre(mode) == POWER and not breakerClosed,
        pre(mode) == SHUTDOWN and f <= f_stopped} then
    mode := if stop and pre(mode) <> STOPPED then SHUTDOWN
            elseif pre(mode) == STOPPED then STARTING
            elseif pre(mode) == STARTING then SPEED
            elseif pre(mode) == SPEED then POWER
            elseif pre(mode) == POWER then SPEED
            else STOPPED;
  end when;

equation
  Kp = if mode == POWER then Kp_power else Kp_speed;
  Ki = if mode == POWER then Ki_power else Ki_speed;
  closedLoop = mode == SPEED or mode == POWER;

  // droop shifts the power reference proportionally to the frequency deviation
  e = if mode == POWER then (P_ref - P_meas) / Pn + (if enableDroop then (f_n - f) / (f_n * droop) else 0)
      else (f_grid_int + f_bias - f) / f_n;

  y_pi = Kp * e + x_i;
  der(x_i) = if closedLoop and not ((y_pi > u_max and e > 0) or (y_pi < u_min and e < 0)) then Ki * e else 0;

  Y_cmd = if mode == STARTING then Y_start
          elseif closedLoop then min(u_max, max(u_min, y_pi))
          else u_min;

  der(Y_gv) = min(openRateMax, max(-closeRateMax, (Y_cmd - Y_gv) / T_servo));

  // pre(mode) avoids an algebraic loop when the breaker is closed directly by syncCommand
  inSyncWindow = pre(mode) == SPEED and abs(f - f_grid_int) <= df_sync;
  syncCommand = inSyncWindow and time - t_window >= t_sync_hold;

  when change(mode) then
    // bumpless transfer: keep the controller output equal to the present opening
    reinit(x_i, min(u_max, max(u_min, Y_gv)) - Kp * e);
  end when;
algorithm
  when inSyncWindow then
    t_window := time;
  elsewhen not inSyncWindow then
    t_window := Modelica.Constants.inf;
  end when;

  annotation (
    preferredView = "info",
    Documentation(info="<html>
<h4>Hydrotrol-like Unit Governor</h4>
<p>
This governor reproduces the main operating logic of a modern digital unit governor
(similar to the Hymatek Hydrotrol family). It contains an internal sequence, both control
modes and the guide-vane actuator, so it can be connected directly between the unit
sequencing signals and the turbine.
</p>
<h5>Sequence states (output <code>mode</code>)</h5>
<ul>
  <li><code>0 STOPPED</code> - guide vanes closed, waiting for <code>start</code>.</li>
  <li><code>1 STARTING</code> - open-loop start opening <code>Y_start</code> until the speed reaches <code>f_speedctrl</code>.</li>
  <li><code>2 SPEED</code> - speed-no-load control towards the grid frequency plus <code>f_bias</code>.</li>
  <li><code>3 POWER</code> - grid-connected power control, entered when <code>breakerClosed</code> becomes true.</li>
  <li><code>4 SHUTDOWN</code> - guide vanes ramped closed after a <code>stop</code> command.</li>
</ul>
<h5>Synchronisation</h5>
<p>
While in speed mode the governor raises <code>syncCommand</code> once the speed has stayed within
<code>df_sync</code> of the grid frequency for <code>t_sync_hold</code> seconds. This output is meant to
drive the breaker (MCB) model. When the breaker feedback <code>breakerClosed</code> arrives, the
governor switches to power control with bumpless transfer of the integral state.
</p>
<h5>Droop</h5>
<p>
Droop is optional and only active in power control mode. The power reference is biased by
<code>(f_n - f)/(f_n*droop)</code> in per unit, giving the usual permanent droop characteristic.
Set <code>enableDroop = false</code> for pure power (isochronous reference) control.
</p>
<p>
If <code>useGridFrequencyInput</code> is false the rated frequency <code>f_n</code> is used as the
synchronising reference and the <code>f_grid</code> connector is removed.
</p>
</html>"));
end GovernorHydrotrol;
