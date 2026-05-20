within OpenHPL.Controllers;
model GovernorDualMode "Governor with droop and automatic speed/power mode switching"
  extends OpenHPL.Icons.Governor;
  outer Data data "Using standard class with constants";

  parameter SI.Time T_p = 0.04 "Pilot servomotor time constant" annotation (
    Dialog(group = "Controller settings"));
  parameter SI.Time T_g = 0.2 "Main servomotor integration time" annotation (
    Dialog(group = "Controller settings"));
  parameter SI.Time T_r = 1.75 "Transient droop time constant" annotation (
    Dialog(group = "Controller settings"));
  parameter SI.Time T_mode = 2 "Mode transition smoothing time constant" annotation (
    Dialog(group = "Controller settings"));
  parameter Real lookup_table[:, :] = [0.0, 0.0; 0.01, 0.06; 0.22, 0.25; 0.53, 0.5; 0.8, 0.75; 1.0, 0.95; 1.05, 1.0] "Table matrix (grid = first column; e.g., table=[0, 0; 1, 1; 2, 4])" annotation (
    Dialog(group = "System settings"));
  parameter Real droop = 0.1 "Droop" annotation (
    Dialog(group = "Controller settings"));
  parameter Real delta = 0.04 "Transient droop" annotation (
    Dialog(group = "Controller settings"));
  parameter Real Y_gv_max = 0.05 "Max guide vane opening rate" annotation (
    Dialog(group = "System settings"));
  parameter Real Y_gv_min = 0.2 "Max guide vane closing rate" annotation (
    Dialog(group = "System settings"));
  parameter Real Y_gv_ref = 0.72151 "Initial guide vane opening rate" annotation (
    Dialog(group = "System settings"));
  parameter SI.Frequency f_ref_grid = data.f_0 "Reference grid frequency" annotation (
    Dialog(group = "System settings"));
  parameter SI.Frequency f_ref_min = 0.2 * data.f_0 "Minimum allowed speed reference" annotation (
    Dialog(group = "System settings"));
  parameter SI.Power Pn = 104e6 "Reference power" annotation (
    Dialog(group = "System settings"));
  parameter Real K_speed = 8 "Speed loop gain [pu power / pu frequency error]" annotation (
    Dialog(group = "Controller settings"));
  parameter Real P_speed_bias_pu = 0.15 "Power bias used in speed mode" annotation (
    Dialog(group = "System settings"));
  parameter Real P_speed_max_pu = 1.05 "Maximum power command used in speed mode" annotation (
    Dialog(group = "System settings"));

  Modelica.Blocks.Interfaces.RealInput P_ref annotation (
    Placement(transformation(extent = {{-140, 20}, {-100, 60}}), iconTransformation(extent = {{-140, 20}, {-100, 60}})));
  Modelica.Blocks.Interfaces.RealInput f annotation (
    Placement(transformation(origin = {-120, -40}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f_ref_speed annotation (
    Placement(transformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -80}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput isGridConnected annotation (
    Placement(transformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 80}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealOutput Y_gv annotation (
    Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));

  Modelica.Blocks.Sources.RealExpression pSpeedCmd(y = min(P_speed_max_pu * Pn, max(0, Pn * (P_speed_bias_pu + K_speed * (f_ref_speed - f) / f_ref_grid)))) annotation (
    Placement(transformation(origin = {-94, -14}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.RealExpression fRefSelected(y = if isGridConnected then f_ref_grid else max(f_ref_speed, f_ref_min)) annotation (
    Placement(transformation(origin = {-94, -92}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Logical.Switch pCmdSwitch annotation (
    Placement(transformation(origin = {-78, 12}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.FirstOrder pCmdFilter(T = T_mode, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = Y_gv_ref * Pn) annotation (
    Placement(transformation(origin = {-56, 12}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.FirstOrder fRefFilter(T = T_mode, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = f_ref_grid) annotation (
    Placement(transformation(origin = {-56, -78}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Tables.CombiTable1Dv look_up_table(table = lookup_table) annotation (
    Placement(transformation(origin = {-14, 40}, extent = {{10, -10}, {-10, 10}}, rotation = 180)));
  Modelica.Blocks.Continuous.TransferFunction pilot_servo(a = {T_p, 1}, b = {1}, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 0) annotation (
    Placement(transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.TransferFunction main_servo(a = {1, 0}, b = {1}, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = Y_gv_ref) annotation (
    Placement(transformation(origin = {46, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Gain gain_T_s(k = 1 / T_g) annotation (
    Placement(transformation(origin = {20, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter_dotY_gv(uMax = Y_gv_max, uMin = -Y_gv_min) annotation (
    Placement(transformation(origin = {18, -20}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Nonlinear.Limiter limiter_Y_gv(uMax = 1, uMin = 0) annotation (
    Placement(transformation(origin = {76, 0}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Continuous.TransferFunction control(a = {T_r, 1}, b = {delta * T_r, 0}, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = 0) annotation (
    Placement(transformation(origin = {62, -30}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Gain gain_droop(k = droop) annotation (
    Placement(transformation(origin = {62, -56}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Gain gain_P(k = 1 / Pn) annotation (
    Placement(transformation(origin = {-36, 40}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Math.Add add1 annotation (
    Placement(transformation(origin = {8, -44}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Add3 add2(k2 = -1, k3 = -1) annotation (
    Placement(transformation(origin = {-18, -24}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Blocks.Math.Add add3(k1 = -1, k2 = +1) annotation (
    Placement(transformation(origin = {-18, -60}, extent = {{10, -10}, {-10, 10}}, rotation = -90)));
  Modelica.Blocks.Sources.Constant const(k = 1) annotation (
    Placement(transformation(origin = {0, -80}, extent = {{10, -10}, {-10, 10}})));
  Modelica.Blocks.Math.Gain gain_droop2(k = droop) annotation (
    Placement(transformation(origin = {-40, -4}, extent = {{-10, 10}, {10, -10}}, rotation = -90)));
  Modelica.Blocks.Math.Division fRatio annotation (
    Placement(transformation(origin = {-40, -60}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(P_ref, pCmdSwitch.u1) annotation (
    Line(points = {{-120, 40}, {-90, 40}, {-90, 20}}, color = {0, 0, 127}));
  connect(pSpeedCmd.y, pCmdSwitch.u3) annotation (
    Line(points = {{-83, -14}, {-70, -14}, {-70, 4}}, color = {0, 0, 127}));
  connect(isGridConnected, pCmdSwitch.u2) annotation (
    Line(points = {{-120, 80}, {-78, 80}, {-78, 24}}, color = {255, 0, 255}));
  connect(pCmdSwitch.y, pCmdFilter.u) annotation (
    Line(points = {{-67, 12}, {-68, 12}}, color = {0, 0, 127}));

  connect(fRefSelected.y, fRefFilter.u) annotation (
    Line(points = {{-83, -92}, {-68, -92}, {-68, -78}}, color = {0, 0, 127}));

  connect(pCmdFilter.y, gain_P.u) annotation (
    Line(points = {{-45, 12}, {-36, 12}, {-36, 28}}, color = {0, 0, 127}));
  connect(gain_P.y, look_up_table.u[1]) annotation (
    Line(points = {{-25, 40}, {-26, 40}}, color = {0, 0, 127}));
  connect(look_up_table.y[1], gain_droop2.u) annotation (
    Line(points = {{-3, 40}, {0, 40}, {0, 20}, {-40, 20}, {-40, 8}}, color = {0, 0, 127}));

  connect(gain_droop2.y, add2.u1) annotation (
    Line(points = {{-40, -15}, {-40, -44}, {-26, -44}, {-26, -36}}, color = {0, 0, 127}));

  connect(f, fRatio.u1) annotation (
    Line(points = {{-120, -40}, {-80, -40}, {-80, -54}, {-52, -54}}, color = {0, 0, 127}));
  connect(fRefFilter.y, fRatio.u2) annotation (
    Line(points = {{-45, -78}, {-20, -78}, {-20, -66}, {-52, -66}}, color = {0, 0, 127}));
  connect(fRatio.y, add3.u2) annotation (
    Line(points = {{-29, -60}, {-24, -60}, {-24, -72}}, color = {0, 0, 127}));
  connect(const.y, add3.u1) annotation (
    Line(points = {{-11, -80}, {-12, -80}, {-12, -72}}, color = {0, 0, 127}));

  connect(add3.y, add2.u2) annotation (
    Line(points = {{-18, -49}, {-18, -36}}, color = {0, 0, 127}));
  connect(add1.y, add2.u3) annotation (
    Line(points = {{-3, -44}, {-10, -44}, {-10, -36}}, color = {0, 0, 127}));
  connect(add2.y, pilot_servo.u) annotation (
    Line(points = {{-18, -13}, {-18, 0}, {-12, 0}}, color = {0, 0, 127}));

  connect(pilot_servo.y, gain_T_s.u) annotation (
    Line(points = {{11, 0}, {8, 0}}, color = {0, 0, 127}));
  connect(gain_T_s.y, limiter_dotY_gv.u) annotation (
    Line(points = {{31, 0}, {34, 0}, {34, -20}, {30, -20}}, color = {0, 0, 127}));
  connect(limiter_dotY_gv.y, main_servo.u) annotation (
    Line(points = {{7, -20}, {0, -20}, {0, 0}, {34, 0}}, color = {0, 0, 127}));
  connect(main_servo.y, limiter_Y_gv.u) annotation (
    Line(points = {{57, 0}, {64, 0}}, color = {0, 0, 127}));

  connect(limiter_Y_gv.y, control.u) annotation (
    Line(points = {{87, 0}, {92, 0}, {92, -30}, {74, -30}}, color = {0, 0, 127}));
  connect(limiter_Y_gv.y, gain_droop.u) annotation (
    Line(points = {{87, 0}, {92, 0}, {92, -56}, {74, -56}}, color = {0, 0, 127}));
  connect(control.y, add1.u1) annotation (
    Line(points = {{51, -30}, {34, -30}, {34, -38}, {20, -38}}, color = {0, 0, 127}));
  connect(gain_droop.y, add1.u2) annotation (
    Line(points = {{51, -56}, {36, -56}, {36, -50}, {20, -50}}, color = {0, 0, 127}));

  connect(limiter_Y_gv.y, Y_gv) annotation (
    Line(points = {{87, 0}, {110, 0}}, color = {0, 0, 127}));

  annotation (preferredView="info", Documentation(info="<html>
<h4>Dual-mode Governor</h4>
<p>
This governor model adds mode switching on top of the standard OpenHPL governor architecture.
When connected to the grid, the controller behaves as a power governor with built-in droop.
When islanded, it shifts to speed control by generating an internal power command from speed error,
while keeping the same actuator dynamics and guide vane limits.
</p>
<p>
The input <code>isGridConnected</code> should normally be driven by the MCB status.
The transition smoothing time <code>T_mode</code> reduces abrupt command steps when changing mode.
</p>
</html>"));
end GovernorDualMode;
