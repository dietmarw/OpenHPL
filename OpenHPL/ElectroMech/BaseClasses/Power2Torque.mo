within OpenHPL.ElectroMech.BaseClasses;
partial model Power2Torque "Converts a power signal to a torque in the rotational domain"
  extends TorqueEquation;
  outer Data data "Using standard class with global parameters";
  Modelica.Blocks.Math.Division power2torque annotation(
    Placement(transformation(extent = {{-76, -6}, {-64, 6}})));
  Modelica.Blocks.Nonlinear.Limiter div0protect(uMax = Modelica.Constants.inf, uMin = Modelica.Constants.small) annotation(
    Placement(transformation(extent = {{-6, -6}, {6, 6}}, rotation = 180, origin = {-50, -40})));
  Modelica.Blocks.Nonlinear.Limiter torqueLimit(uMax=Pmax/(2*C.pi*f_nom))
                                                                 annotation (
    Placement(transformation(extent = {{6, 6}, {-6, -6}}, rotation = 180, origin = {-50, 0})));
  Modelica.Blocks.Sources.RealExpression power annotation(
    Placement(transformation(extent = {{-60, 20}, {-80, 40}})));
equation
  connect(div0protect.y, power2torque.u2) annotation(
    Line(points = {{-56.6, -40}, {-88, -40}, {-88, -3.6}, {-77.2, -3.6}}, color = {0, 0, 127}));
  connect(power2torque.y, torqueLimit.u) annotation(
    Line(points = {{-63.4, 0}, {-64, 0}, {-64, 8.88178e-16}, {-57.2, 8.88178e-16}}, color = {0, 0, 127}));
  connect(power.y, power2torque.u1) annotation(
    Line(points = {{-81, 30}, {-88, 30}, {-88, 3.6}, {-77.2, 3.6}}, color = {0, 0, 127}));
  connect(torqueLimit.y, torque.tau) annotation(
    Line(points = {{-44, 0}, {-38, 0}}, color = {0, 0, 127}));
  connect(div0protect.u, speedSensor.w) annotation(
    Line(points = {{-42, -40}, {10, -40}, {10, -30}}, color = {0, 0, 127}));
  annotation(
    Icon(graphics = {Text(visible = enable_w, extent = {{80, 50}, {100, 30}}, textColor = {0, 0, 0}, textString = "w"), Text(visible = enable_f, extent = {{80, -30}, {100, -50}}, textColor = {0, 0, 0}, textString = "f"), Text(visible = enable_f_in, extent = {{-100, -70}, {-60, -90}}, textColor = {0, 0, 0}, textString = "f_in")}));
end Power2Torque;
