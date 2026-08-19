within OpenHPL.Examples;
model HydrotrolStartup "Startup sequence using the Hydrotrol-like governor"
  extends Modelica.Icons.Example;

  OpenHPL.Waterway.Reservoir reservoir(h_0 = 10, fixElevation = true, z_0 = 100)
    annotation (Placement(transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.Pipe intake(H = 10, D_i = 5)
    annotation (Placement(transformation(origin = {-75, 0}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600, D_i = 5)
    annotation (Placement(transformation(origin = {75, 0}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.Reservoir tail(h_0 = 5)
    annotation (Placement(transformation(origin = {105, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  OpenHPL.Waterway.Pipe penstock(D_i = 3, D_o = 3, H = 80, L = 200, slanted = true)
    annotation (Placement(transformation(origin = {-5, 0}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.SurgeTank surgeTank(H = 25, L = 30, h_0 = 20)
    annotation (Placement(transformation(origin = {-45, 0}, extent = {{-10, -10}, {10, 10}})));

  OpenHPL.ElectroMech.Turbines.Turbine turbine(
    ValveCapacity = false,
    enable_nomSpeed = false,
    enable_P_out = true)
    annotation (Placement(transformation(origin = {25, 0}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.ElectroMech.Generators.SimpleGen simpleGen(
    enable_f = true,
    f_0 = 0,
    fixed_iniSpeed = true)
    annotation (Placement(transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Controllers.GovernorHydrotrol governor(
    f_n = data.f_grid,
    Pn = 10e6,
    Y_start = 0.6,
    f_speedctrl = 0.85 * data.f_grid,
    t_sync_hold = 5)
    annotation (Placement(transformation(origin = {-16, 49}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Math.Gain fToHz(k = data.f_grid)
    annotation (Placement(transformation(origin = {50, 35}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Logical.Switch loadSwitch
    annotation (Placement(transformation(origin = {50, 65}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant zeroLoad(k = 0)
    annotation (Placement(transformation(origin = {11, 85}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant connectedLoad(k = 6e6)
    annotation (Placement(transformation(origin = {11, 59}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.BooleanStep startCommand(startTime = 5)
    annotation (Placement(transformation(origin = {-55, 85}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.BooleanConstant stopCommand(k = false)
    annotation (Placement(transformation(origin = {-65, 60}, extent = {{-10, -10}, {10, 10}})));

  inner OpenHPL.Data data(SteadyState = false, Vdot_0 = 0, f_0 = 0)
    annotation (Placement(transformation(origin = {-100, 85}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(reservoir.o, intake.i) annotation (Line(points = {{-90, 0}, {-85, 0}}, color = {28, 108, 200}));
  connect(intake.o, surgeTank.i) annotation (Line(points = {{-65, 0}, {-55, 0}}, color = {28, 108, 200}));
  connect(surgeTank.o, penstock.i) annotation (Line(points = {{-35, 0}, {-15, 0}}, color = {28, 108, 200}));
  connect(penstock.o, turbine.i) annotation (Line(points = {{5, 0}, {15, 0}}, color = {28, 108, 200}));
  connect(turbine.o, discharge.i) annotation (Line(points = {{35, 0}, {65, 0}}, color = {28, 108, 200}));
  connect(discharge.o, tail.o) annotation (Line(points = {{85, 0}, {95, 0}}, color = {28, 108, 200}));

  connect(turbine.flange, simpleGen.flange) annotation (Line(points = {{25, 0}, {50, 0}}, color = {0, 0, 0}));
  connect(governor.Y_gv, turbine.u_t) annotation (Line(points = {{-5, 49}, {27, 49}, {27, 12}, {25, 12}}, color = {0, 0, 127}));
  connect(simpleGen.f, fToHz.u) annotation (Line(points = {{61, 0}, {70, 0}, {70, 35}, {62, 35}}, color = {0, 0, 127}));
  connect(fToHz.y, governor.f) annotation (Line(points = {{38, 35}, {4, 35}, {4, 39}, {-28, 39}}, color = {0, 0, 127}));

  connect(startCommand.y, governor.start) annotation (Line(points = {{-45, 85}, {-35, 85}, {-35, 57}, {-28, 57}}, color = {255, 0, 255}));
  connect(stopCommand.y, governor.stop) annotation (Line(points = {{-54, 60}, {-37.5, 60}, {-37.5, 53}, {-28, 53}}, color = {255, 0, 255}));
  connect(governor.syncCommand, governor.breakerClosed) annotation (Line(points = {{-5, 55}, {0, 55}, {0, 40}, {-30, 40}, {-30, 45}, {-25, 45}}, color = {255, 0, 255}));
  connect(governor.syncCommand, loadSwitch.u2) annotation (Line(points = {{-5, 55}, {15, 55}, {15, 65}, {40, 65}}, color = {255, 0, 255}));
  connect(zeroLoad.y, loadSwitch.u3) annotation (Line(points = {{22, 85}, {35, 85}, {35, 73}, {40, 73}}, color = {0, 0, 127}));
  connect(connectedLoad.y, loadSwitch.u1) annotation (Line(points = {{22, 59}, {32, 59}, {32, 57}, {40, 57}}, color = {0, 0, 127}));
  connect(loadSwitch.y, simpleGen.Pload) annotation (Line(points = {{60, 65}, {75, 65}, {75, 12}, {50, 12}}, color = {0, 0, 127}));

  connect(loadSwitch.y, governor.P_meas) annotation (Line(points = {{60, 65}, {80, 65}, {80, 45}, {-35, 45}, {-35, 42}, {-28, 42}}, color = {0, 0, 127}));
  connect(connectedLoad.y, governor.P_ref) annotation (Line(points = {{22, 59}, {30, 59}, {30, 45}, {-28, 45}}, color = {0, 0, 127}));
  annotation (experiment(
    StopTime = 800,
    StartTime = 0,
    Tolerance = 1e-06,
    Interval = 0.5), Documentation(info = "<html>
<h4>Hydrotrol Startup Example</h4>
<p>
This example uses the simple turbine and generator models to demonstrate a complete
unit startup. At <code>t=5 s</code> the governor opens the guide vanes, accelerates
the unit, waits for synchronisation, and then applies a 6 MW electrical load.
</p>
<p>
The ideal breaker is represented by the feedback from <code>syncCommand</code> to
<code>breakerClosed</code>. A production model would replace this connection with
an electrical breaker and its measured status.
</p>
</html>"));
end HydrotrolStartup;