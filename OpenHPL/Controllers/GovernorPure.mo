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

protected
  Real Kp_act;
  Real Ki_act;
  Real e_ctrl;
  Real droopTerm;
  SI.Power P_ref_eff;
  Real u_unsat;
  Real xi(start = 0);

equation
  Kp_act = if isGridConnected then Kp_power else Kp_speed;
  Ki_act = if isGridConnected then Ki_power else Ki_speed;

  droopTerm = if isGridConnected and enableDroop then (f_ref_grid - f) / f_ref_grid / droop else 0;
  P_ref_eff = P_ref + Pn * droopTerm;

  e_ctrl = if isGridConnected then
             (P_ref_eff - P_meas) / Pn
           else
             (f_ref_speed - f) / f_ref_grid;

  u_unsat = u_start + Kp_act * e_ctrl + xi;
  u_ref = min(u_max, max(u_min, u_unsat));

  der(xi) = Ki_act * e_ctrl + Kaw * (u_ref - u_unsat);

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
Use <code>u_ref</code> as input to <code>HPUDynamics</code>.
</p>
</html>"));
end GovernorPure;
