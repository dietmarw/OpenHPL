within OpenHPL.Controllers;
model HPUDynamics "HPU actuator dynamics and guide-vane restrictions"
  parameter SI.Time T_servo = 0.2 "Actuator first-order time constant" annotation (
    Dialog(group = "Dynamics"));
  parameter Real openRateMax = 0.05 "Maximum opening rate [1/s]" annotation (
    Dialog(group = "Restrictions"));
  parameter Real closeRateMax = 0.2 "Maximum closing rate [1/s]" annotation (
    Dialog(group = "Restrictions"));
  parameter Real u_min = 0 "Minimum guide-vane opening" annotation (
    Dialog(group = "Restrictions"));
  parameter Real u_max = 1 "Maximum guide-vane opening" annotation (
    Dialog(group = "Restrictions"));
  parameter Real u_start = 0.1 "Initial guide-vane opening" annotation (
    Dialog(group = "Initialization"));

  Modelica.Blocks.Interfaces.RealInput u_ref "Opening reference from governor" annotation (
    Placement(transformation(extent = {{-140, -20}, {-100, 20}}), iconTransformation(extent = {{-140, -20}, {-100, 20}})));
  Modelica.Blocks.Interfaces.RealOutput u_t "Guide-vane command to turbine" annotation (
    Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));

  Modelica.Blocks.Continuous.FirstOrder servo(
    T = T_servo,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = u_start) annotation (Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
  Modelica.Blocks.Nonlinear.SlewRateLimiter slew(
    Rising = openRateMax,
    Falling = closeRateMax,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = u_start) annotation (Placement(transformation(extent = {{-40, -10}, {-20, 10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter(uMax = u_max, uMin = u_min) annotation (
    Placement(transformation(extent = {{20, -10}, {40, 10}})));

equation
  connect(u_ref, servo.u) annotation (Line(points = {{-120, 0}, {-82, 0}}, color = {0, 0, 127}));
  connect(servo.y, slew.u) annotation (Line(points = {{-59, 0}, {-42, 0}}, color = {0, 0, 127}));
  connect(slew.y, limiter.u) annotation (Line(points = {{-19, 0}, {18, 0}}, color = {0, 0, 127}));
  connect(limiter.y, u_t) annotation (Line(points = {{41, 0}, {110, 0}}, color = {0, 0, 127}));

  annotation (preferredView="info", Documentation(info="<html>
<h4>HPU Dynamics</h4>
<p>
This model contains actuator dynamics and hardware restrictions for guide-vane motion:
first-order servo lag, opening/closing rate limits, and physical saturation.
</p>
<p>
Use it between a pure governor and the turbine input.
</p>
</html>"), Icon(graphics={
    Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}),
    Text(extent={{-88,64},{88,30}}, textString="HPU", textStyle={TextStyle.Bold}),
    Text(extent={{-92,10},{92,-10}}, textString="servo + limits"),
    Text(extent={{-100,-110},{100,-140}}, textColor={127,0,0}, textStyle={TextStyle.Bold}, textString="%name")}));
end HPUDynamics;
