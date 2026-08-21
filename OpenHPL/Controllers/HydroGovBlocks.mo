within OpenHPL.Controllers;
model HydroGovBlocks "Block-based HydroGov governor with start sequence, synchronisation and power control"
  extends OpenHPL.Icons.Governor;
  outer Data data "Using standard class with constants";

  constant Integer STOPPED = 0 "Unit at standstill";
  constant Integer STARTING = 1 "Open-loop start opening";
  constant Integer SPEED = 2 "Speed-no-load control, ready to synchronise";
  constant Integer POWER = 3 "Grid-connected power control";
  constant Integer SHUTDOWN = 4 "Closing guide vanes";

  parameter SI.Frequency f_n = data.f_0 "Rated (synchronous) frequency" annotation (Dialog(group = "System settings"));
  parameter SI.Power Pn = 104e6 "Rated active power" annotation (Dialog(group = "System settings"));
  parameter Boolean useGridFrequencyInput = false "Use measured grid frequency for synchronisation" annotation (choices(checkBox = true), Dialog(group = "System settings"));
  parameter Boolean enableDroop = true "Enable droop in power control mode" annotation (choices(checkBox = true), Dialog(group = "Droop"));
  parameter Real droop(min = 1e-3) = 0.06 "Permanent droop" annotation (Dialog(group = "Droop", enable = enableDroop));
  parameter Real Kp_speed = 3.0 "Speed loop proportional gain [pu opening / pu speed]" annotation (Dialog(group = "Speed control"));
  parameter Real Ki_speed = 0.6 "Speed loop integral gain [pu opening / (pu speed . s)]" annotation (Dialog(group = "Speed control"));
  parameter Real Kaw = 2.0 "Anti-windup back-calculation gain [1/s]" annotation (Dialog(group = "Control settings"));
  parameter Real Kp_power = 0.6 "Power loop proportional gain [pu opening / pu power]" annotation (Dialog(group = "Power control"));
  parameter Real Ki_power = 0.1 "Power loop integral gain [pu opening / (pu power . s)]" annotation (Dialog(group = "Power control"));
  parameter Real Y_start = 0.25 "Start opening used until speed control takes over" annotation (Dialog(group = "Start sequence"));
  parameter SI.Frequency f_speedctrl = 0.85 * f_n "Speed where the start opening is released to speed control" annotation (Dialog(group = "Start sequence"));
  parameter SI.Frequency f_stopped = 0.05 * f_n "Speed below which the unit is considered stopped" annotation (Dialog(group = "Start sequence"));
  parameter SI.Frequency df_sync = 0.05 "Allowed speed deviation for the synchronising command" annotation (Dialog(group = "Synchronisation"));
  parameter SI.Frequency f_bias = 0.02 "Speed reference bias above grid frequency before synchronising" annotation (Dialog(group = "Synchronisation"));
  parameter SI.Time t_sync_hold = 5 "Time the speed must stay inside the window before the sync command" annotation (Dialog(group = "Synchronisation"));
  parameter SI.Time T_servo = 0.2 "Guide vane servo time constant" annotation (Dialog(group = "Actuator"));
  parameter Real openRateMax(unit = "1/s") = 0.05 "Maximum opening rate" annotation (Dialog(group = "Actuator"));
  parameter Real closeRateMax(unit = "1/s") = 0.2 "Maximum closing rate" annotation (Dialog(group = "Actuator"));
  parameter Real u_min = 0 "Minimum guide vane opening" annotation (Dialog(group = "Actuator"));
  parameter Real u_max = 1 "Maximum guide vane opening" annotation (Dialog(group = "Actuator"));
  parameter Real Y_gv_start = 0 "Initial guide vane opening" annotation (Dialog(group = "Initialization"));
  parameter Integer mode_start = STOPPED "Initial sequence state" annotation (Dialog(group = "Initialization"));

  Modelica.Blocks.Interfaces.BooleanInput start "Start command" annotation (Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput stop "Stop command" annotation (Placement(transformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput breakerClosed "Generator breaker status" annotation (Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput P_ref(unit = "W") "Active power reference" annotation (Placement(transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput P_meas(unit = "W") "Measured active power" annotation (Placement(transformation(origin = {-120, -70}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -70}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f(unit = "Hz") "Measured unit speed as electrical frequency" annotation (Placement(transformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -100}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f_grid(unit = "Hz") if useGridFrequencyInput "Measured grid frequency" annotation (Placement(transformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90), iconTransformation(origin = {0, -120}, extent = {{-20, -20}, {20, 20}}, rotation = 90)));
  Modelica.Blocks.Interfaces.RealOutput Y_gv(start = Y_gv_start, fixed = true) "Guide vane opening" annotation (Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
  Modelica.Blocks.Interfaces.BooleanOutput syncCommand "Request to close the generator breaker" annotation (Placement(transformation(extent = {{100, 50}, {120, 70}}), iconTransformation(extent = {{100, 50}, {120, 70}})));
  Modelica.Blocks.Interfaces.IntegerOutput mode(start = mode_start, fixed = true) "Active sequence state" annotation (Placement(transformation(extent = {{100, -70}, {120, -50}}), iconTransformation(extent = {{100, -70}, {120, -50}})));

  Modelica.Blocks.Continuous.LimPID piSpeed(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = Kp_speed, Ti = if Ki_speed > 0 then Kp_speed / Ki_speed else 1e9, Ni = if Kaw > 0 then 1 / Kaw else 1, yMax = u_max, yMin = u_min, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = Y_gv_start) annotation (Placement(transformation(origin = {-20, -53}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.LimPID piPower(controllerType = Modelica.Blocks.Types.SimpleController.PI, k = Kp_power, Ti = if Ki_power > 0 then Kp_power / Ki_power else 1e9, Ni = if Kaw > 0 then 1 / Kaw else 1, yMax = u_max, yMin = u_min, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = Y_gv_start) annotation (Placement(transformation(origin = {-30, 25}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain speedRefNorm(k = 1 / f_n) annotation (Placement(transformation(extent = {{-90, -70}, {-78, -58}})));
  Modelica.Blocks.Math.Add speedReference(k1 = 1, k2 = 1) annotation (Placement(transformation(extent = {{-110, -70}, {-98, -58}})));
  Modelica.Blocks.Math.Gain speedMeasNorm(k = 1 / f_n) annotation (Placement(transformation(extent = {{-90, -100}, {-78, -88}})));
  Modelica.Blocks.Math.Gain powerRefNorm(k = 1 / Pn) annotation (Placement(transformation(extent = {{-90, 40}, {-78, 52}})));
  Modelica.Blocks.Math.Gain powerMeasNorm(k = 1 / Pn) annotation (Placement(transformation(extent = {{-90, 10}, {-78, 22}})));
  Modelica.Blocks.Math.Add droopError(k1 = 1, k2 = -1) annotation (Placement(transformation(extent = {{-90, -5}, {-78, 7}})));
  Modelica.Blocks.Math.Gain droopGain(k = if enableDroop then 1 / (f_n * droop) else 0) annotation (Placement(transformation(extent = {{-70, -5}, {-58, 7}})));
  Modelica.Blocks.Math.Add3 powerReference(k1 = 1, k2 = 1, k3 = 0) annotation (Placement(transformation(extent = {{-55, 35}, {-43, 47}})));
  Modelica.Blocks.Logical.Switch powerModeSwitch annotation (Placement(transformation(origin = {15, 19}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Logical.Switch closedLoopSwitch annotation (Placement(transformation(origin = {44, -19}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Logical.Switch startSwitch annotation (Placement(transformation(origin = {73, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant zero(k = u_min) annotation (Placement(transformation(origin = {-2, 0}, extent = {{10, -45}, {22, -33}})));
  Modelica.Blocks.Sources.Constant speedBias(k = f_bias) annotation (Placement(transformation(extent = {{-130, -55}, {-118, -43}})));
  Modelica.Blocks.Sources.Constant startOpening(k = Y_start) annotation (Placement(transformation(origin = {0, -2}, extent = {{35, 25}, {47, 37}})));
  ModeDetermination modeDetermination(f_speedctrl = f_speedctrl, f_stopped = f_stopped, mode_start = mode_start) annotation (Placement(transformation(origin = {11, 72}, extent = {{-10, -10}, {10, 10}})));

  Real y_pi "Selected unlimited controller output";
  Real Y_cmd "Opening command before actuator";
  Boolean inSyncWindow "True while speed is inside the synchronising window";
  discrete Real t_window(start = Modelica.Constants.inf, fixed = true);
protected
  Modelica.Blocks.Interfaces.RealInput f_grid_int(unit = "Hz");

equation
  connect(f_grid, f_grid_int) annotation (Line(points = {{0, -120}, {0, -110}}, color = {0, 0, 127}));
  if not useGridFrequencyInput then
    f_grid_int = f_n;
  end if;

equation
  connect(start, modeDetermination.start) annotation (Line(points = {{-120, 80}, {-115, 80}, {-115, 96}}, color = {255, 0, 255}));
  connect(stop, modeDetermination.stop) annotation (Line(points = {{-120, 40}, {-118, 40}, {-118, 92}, {-115, 92}}, color = {255, 0, 255}));
  connect(breakerClosed, modeDetermination.breakerClosed) annotation (Line(points = {{-120, 0}, {-120, 0}, {-115, 0}, {-115, 88}}, color = {255, 0, 255}));
  connect(f, modeDetermination.f) annotation (Line(points = {{-120, -100}, {-122, -100}, {-122, 84}, {-115, 84}}, color = {0, 0, 127}));
  mode = modeDetermination.mode;
  connect(modeDetermination.closedLoop, closedLoopSwitch.u2) annotation (Line(points = {{-95, 88}, {-20, 88}, {-20, -19}, {34, -19}}, color = {255, 0, 255}));
  connect(modeDetermination.powerMode, powerModeSwitch.u2) annotation (Line(points = {{-95, 84}, {-5, 84}, {-5, 19}, {5, 19}}, color = {255, 0, 255}));
  connect(modeDetermination.startingMode, startSwitch.u2) annotation (Line(points = {{-95, 80}, {-25, 80}, {-25, 0}, {63, 0}}, color = {255, 0, 255}));
  connect(f_grid_int, speedReference.u1) annotation (Line(points = {{0, -110}, {-116, -110}, {-116, -64}, {-111, -64}}, color = {0, 0, 127}));
  connect(speedBias.y, speedReference.u2) annotation (Line(points = {{-118, -49}, {-114, -49}, {-114, -60}, {-111, -60}}, color = {0, 0, 127}));
  connect(speedReference.y, speedRefNorm.u) annotation (Line(points = {{-97, -64}, {-90, -64}}, color = {0, 0, 127}));
  connect(f, speedMeasNorm.u) annotation (Line(points = {{-120, -100}, {-90, -94}}, color = {0, 0, 127}));
  connect(speedRefNorm.y, piSpeed.u_s) annotation (Line(points = {{-78, -64}, {-55, -64}}, color = {0, 0, 127}));
  connect(speedMeasNorm.y, piSpeed.u_m) annotation (Line(points = {{-78, -94}, {-55, -48}}, color = {0, 0, 127}));
  connect(P_ref, powerRefNorm.u) annotation (Line(points = {{-120, -40}, {-100, -40}, {-100, 46}, {-90, 46}}, color = {0, 0, 127}));
  connect(P_meas, powerMeasNorm.u) annotation (Line(points = {{-120, -70}, {-96, -70}, {-96, 16}, {-90, 16}}, color = {0, 0, 127}));
  connect(f_grid_int, droopError.u1) annotation (Line(points = {{0, -110}, {-104, -110}, {-104, 4}, {-90, 4}}, color = {0, 0, 127}));
  connect(f, droopError.u2) annotation (Line(points = {{-120, -100}, {-108, -100}, {-108, -2}, {-90, -2}}, color = {0, 0, 127}));
  connect(droopError.y, droopGain.u) annotation (Line(points = {{-78, 1}, {-70, 1}}, color = {0, 0, 127}));
  connect(powerRefNorm.y, powerReference.u1) annotation (Line(points = {{-78, 46}, {-55, 46}}, color = {0, 0, 127}));
  connect(droopGain.y, powerReference.u2) annotation (Line(points = {{-58, 1}, {-50, 1}, {-50, 42}, {-55, 42}}, color = {0, 0, 127}));
  connect(zero.y, powerReference.u3) annotation (Line(points = {{22, -39}, {28, -39}, {28, 35}, {-60, 35}, {-60, 38}, {-55, 38}}, color = {0, 0, 127}));
  connect(powerMeasNorm.y, piPower.u_m) annotation (Line(points = {{-78, 16}, {-62, 16}, {-62, 21}, {-40, 21}}, color = {0, 0, 127}));
  connect(powerReference.y, piPower.u_s) annotation (Line(points = {{-43, 41}, {-40, 41}, {-40, 29}}, color = {0, 0, 127}));
  connect(piPower.y, powerModeSwitch.u1) annotation (Line(points = {{-20, 25}, {5, 25}}, color = {0, 0, 127}));
  connect(piSpeed.y, powerModeSwitch.u3) annotation (Line(points = {{-20, -55}, {0, -55}, {0, 21}, {5, 21}}, color = {0, 0, 127}));
  connect(powerModeSwitch.y, closedLoopSwitch.u1) annotation (Line(points = {{25, 25}, {30, 25}, {30, -11}}, color = {0, 0, 127}));
  connect(zero.y, closedLoopSwitch.u3) annotation (Line(points = {{22, -39}, {30, -39}, {30, -19}}, color = {0, 0, 127}));
  connect(closedLoopSwitch.y, startSwitch.u3) annotation (Line(points = {{50, -15}, {55, -15}, {55, -4}}, color = {0, 0, 127}));
  connect(startOpening.y, startSwitch.u1) annotation (Line(points = {{47, 31}, {55, 31}, {55, 4}}, color = {0, 0, 127}));
  Y_cmd = startSwitch.y;
  y_pi = powerModeSwitch.y;
  der(Y_gv) = min(openRateMax, max(-closeRateMax, (Y_cmd - Y_gv) / T_servo));

  inSyncWindow = pre(mode) == SPEED and abs(f - f_grid_int) <= df_sync;
  syncCommand = inSyncWindow and time - t_window >= t_sync_hold;

algorithm
  when inSyncWindow then
    t_window := time;
  elsewhen not inSyncWindow then
    t_window := Modelica.Constants.inf;
  end when;

  annotation (preferredView = "info", Documentation(info = "<html>
<h4>Block-based HydroGov governor</h4>
<p>This is the Standard Library block implementation of <code>HydroGov</code>.
The public connectors, parameters, sequence states, synchronisation hold, droop and
guide-vane actuator have the same meaning as in the equation-based model.</p>
<p>The speed and power controllers use <code>Modelica.Blocks.Continuous.LimPID</code>.
The reference scaling, droop, mode selection and command selection use Standard
Library arithmetic, source and logical blocks. The discrete state machine, timer and
asymmetric servo rate law remain equations because they carry event/state semantics
not represented by a single Standard Library block.</p>
</html>"));
end HydroGovBlocks;