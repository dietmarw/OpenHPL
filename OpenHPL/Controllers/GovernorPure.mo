within OpenHPL.Controllers;
model GovernorPure "Pure governor with switchable speed/power mode and optional droop"
  extends OpenHPL.Icons.Governor;
  outer Data data "Using standard class with constants";

  parameter SI.Frequency f_ref_grid = data.f_0 "Reference grid frequency" annotation (
    Dialog(group = "System settings"));
  parameter SI.Power Pn = 104e6 "Reference power" annotation (
    Dialog(group = "System settings"));

  parameter Boolean enableDroop = true "Enable droop in power control mode" annotation (
    choices(checkBox = true),
    Dialog(group = "Controller settings"));
  parameter Real droop(min = 1e-3) = 0.1 "Droop" annotation (
    Dialog(group = "Controller settings", enable = enableDroop));

  parameter Real Kp_power = 0.8 "Power mode proportional gain" annotation (
    Dialog(group = "Controller settings"));
  parameter Real Ki_power = 0.06 "Power mode integral gain" annotation (
    Dialog(group = "Controller settings"));
  parameter Real Kp_speed = 2.0 "Speed mode proportional gain" annotation (
    Dialog(group = "Controller settings"));
  parameter Real Ki_speed = 0.4 "Speed mode integral gain" annotation (
    Dialog(group = "Controller settings"));
  parameter Real Kaw = 2.0 "Anti-windup back calculation gain" annotation (
    Dialog(group = "Controller settings"));

  parameter Real u_min = 0 "Minimum opening reference" annotation (
    Dialog(group = "System settings"));
  parameter Real u_max = 1 "Maximum opening reference" annotation (
    Dialog(group = "System settings"));
  parameter Real u_start = 0.1 "Initial opening reference" annotation (
    Dialog(group = "System settings"));

  Modelica.Blocks.Interfaces.RealInput P_ref(unit = "W") "Requested active power" annotation (
    Placement(transformation(extent = {{-140, 20}, {-100, 60}}), iconTransformation(extent = {{-140, 20}, {-100, 60}})));
  Modelica.Blocks.Interfaces.RealInput P_meas(unit = "W") "Measured active power" annotation (
    Placement(transformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 0}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f(unit = "Hz") "Measured frequency" annotation (
    Placement(transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f_ref_speed(unit = "Hz") "Speed mode frequency reference" annotation (
    Placement(transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput isGridConnected "True when connected to the grid" annotation (
    Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));

  Modelica.Blocks.Interfaces.RealOutput u_ref "Opening reference to HPU dynamics" annotation (
    Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));

  Modelica.Blocks.Math.Add fDroopErr(k1 = 1, k2 = -1) annotation (
    Placement(transformation(extent = {{-86, -54}, {-74, -42}})));
  Modelica.Blocks.Sources.Constant fRefGridConst(k = f_ref_grid) annotation (
    Placement(transformation(extent = {{-104, -50}, {-92, -38}})));
  Modelica.Blocks.Math.Gain fDroopNorm(k = if enableDroop then (1 / (f_ref_grid * droop)) else 0) annotation (
    Placement(transformation(extent = {{-68, -54}, {-56, -42}})));
  Modelica.Blocks.Math.Gain droopToPower(k = Pn) annotation (
    Placement(transformation(extent = {{-50, -54}, {-38, -42}})));
  Modelica.Blocks.Math.Add pRefEff annotation (
    Placement(transformation(extent = {{-30, -12}, {-18, 0}})));

  Modelica.Blocks.Math.Add pErr(k1 = 1, k2 = -1) annotation (
    Placement(transformation(extent = {{-86, 20}, {-74, 32}})));
  Modelica.Blocks.Math.Gain pErrNorm(k = 1 / Pn) annotation (
    Placement(transformation(extent = {{-68, 20}, {-56, 32}})));
  Modelica.Blocks.Continuous.LimPID piPower(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = Kp_power,
    Ti = if Ki_power > 0 then Kp_power / Ki_power else 1e9,
    Ni = if Kaw > 0 then 1 / Kaw else 1,
    yMax = u_max,
    yMin = u_min,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = u_start) annotation (
    Placement(transformation(extent = {{-40, 16}, {-24, 32}})));

  Modelica.Blocks.Math.Add fErr annotation (
    Placement(transformation(extent = {{-86, -92}, {-74, -80}})));
  Modelica.Blocks.Math.Gain fErrNorm(k = 1 / f_ref_grid) annotation (
    Placement(transformation(extent = {{-68, -92}, {-56, -80}})));
  Modelica.Blocks.Continuous.LimPID piSpeed(
    controllerType = Modelica.Blocks.Types.SimpleController.PI,
    k = Kp_speed,
    Ti = if Ki_speed > 0 then Kp_speed / Ki_speed else 1e9,
    Ni = if Kaw > 0 then 1 / Kaw else 1,
    yMax = u_max,
    yMin = u_min,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = u_start) annotation (
    Placement(transformation(extent = {{-40, -96}, {-24, -80}})));

  Modelica.Blocks.Logical.Switch modeSwitch annotation (
    Placement(transformation(extent = {{20, -10}, {40, 10}})));

equation
  connect(fRefGridConst.y, fDroopErr.u1) annotation (Line(points={{-91.4,-44},{-87.2,-44},{-87.2,-44.4}}, color={0,0,127}));
  connect(f, fDroopErr.u2) annotation (Line(points={{-120,-40},{-112,-40},{-112,-51.6},{-87.2,-51.6}}, color={0,0,127}));
  connect(fDroopErr.y, fDroopNorm.u) annotation (Line(points={{-73.4,-48},{-69.2,-48}}, color={0,0,127}));
  connect(fDroopNorm.y, droopToPower.u) annotation (Line(points={{-55.4,-48},{-51.2,-48}}, color={0,0,127}));

  connect(P_ref, pRefEff.u1) annotation (Line(points={{-120,40},{-44,40},{-44,-2.4},{-31.2,-2.4}}, color={0,0,127}));
  connect(droopToPower.y, pRefEff.u2) annotation (Line(points={{-37.4,-48},{-34,-48},{-34,-9.6},{-31.2,-9.6}}, color={0,0,127}));

  connect(pRefEff.y, pErr.u1) annotation (Line(points={{-17.4,-6},{-12,-6},{-12,29.6},{-87.2,29.6}}, color={0,0,127}));
  connect(P_meas, pErr.u2) annotation (Line(points={{-120,0},{-94,0},{-94,22.4},{-87.2,22.4}}, color={0,0,127}));
  connect(pErr.y, pErrNorm.u) annotation (Line(points={{-73.4,26},{-69.2,26}}, color={0,0,127}));
  connect(pErrNorm.y, piPower.u_s) annotation (Line(points={{-55.4,26},{-41.6,26}}, color={0,0,127}));

  connect(f_ref_speed, fErr.u1) annotation (Line(points={{-120,-80},{-92,-80},{-92,-82.4},{-87.2,-82.4}}, color={0,0,127}));
  connect(f, fErr.u2) annotation (Line(points={{-120,-40},{-100,-40},{-100,-89.6},{-87.2,-89.6}}, color={0,0,127}));
  connect(fErr.y, fErrNorm.u) annotation (Line(points={{-73.4,-86},{-69.2,-86}}, color={0,0,127}));
  connect(fErrNorm.y, piSpeed.u_s) annotation (Line(points={{-55.4,-86},{-41.6,-86}}, color={0,0,127}));

  connect(piPower.y, modeSwitch.u1) annotation (Line(points={{-23.2,24},{4,24},{4,8},{18,8}}, color={0,0,127}));
  connect(piSpeed.y, modeSwitch.u3) annotation (Line(points={{-23.2,-88},{4,-88},{4,-8},{18,-8}}, color={0,0,127}));
  connect(isGridConnected, modeSwitch.u2) annotation (Line(points={{-120,80},{10,80},{10,0},{18,0}}, color={255,0,255}));
  connect(modeSwitch.y, u_ref) annotation (Line(points={{41,0},{110,0}}, color={0,0,127}));

  annotation (preferredView="info", Documentation(info="<html>
<h4>Pure Governor</h4>
<p>
This model contains only the governor logic. It does not include HPU actuator dynamics,
rate limits, or guide-vane hardware constraints.
</p>
<p>
The mode switches with <code>isGridConnected</code>:
</p>
<ul>
  <li><strong>Power mode</strong> (connected): PI on active-power error, with optional droop.</li>
  <li><strong>Speed mode</strong> (islanded): PI on frequency error.</li>
</ul>
<p>
The implementation is block-based using Modelica Standard Library blocks.
</p>
<p>
Use <code>u_ref</code> as input to <code>HPUDynamics</code>.
</p>
</html>"));
end GovernorPure;
