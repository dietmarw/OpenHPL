within OpenHPL.Examples;
model HydroGovStartup "Startup sequence using the HydroGov governor"
  extends Modelica.Icons.Example;

  OpenHPL.Waterway.Reservoir reservoir(h_0 = 10, fixElevation = true, z_0 = 100)
    annotation (Placement(transformation(origin = {-102, -62}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.Pipe intake(H = 10, D_i = 5)
    annotation (Placement(transformation(origin = {-77, -62}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600, D_i = 5)
    annotation (Placement(transformation(origin = {73, -62}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.Reservoir tail(h_0 = 5)
    annotation (Placement(transformation(origin = {103, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  OpenHPL.Waterway.Pipe penstock(D_i = 3, D_o = 3, H = 80, L = 200, slanted = true)
    annotation (Placement(transformation(origin = {-7, -62}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.Waterway.SurgeTank surgeTank(H = 25, L = 30, h_0 = 20)
    annotation (Placement(transformation(origin = {-47, -62}, extent = {{-10, -10}, {10, 10}})));

  OpenHPL.ElectroMech.Turbines.Turbine turbine(
    ValveCapacity = false,
    enable_nomSpeed = false,
    enable_P_out = true)
    annotation (Placement(transformation(origin = {23, -62}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.ElectroMech.Generators.SimpleGen simpleGen(
    enable_f = true,
    f_0 = 0,
    fixed_iniSpeed = true)
    annotation (Placement(transformation(origin = {32, -24}, extent = {{-10, -10}, {10, 10}})));
  OpenHPL.ElectroMech.PowerSystem.MCB mcb(
    useExternalControl = true,
    deltaSpeed = 0.001)
    annotation (Placement(transformation(origin = {6, -52}, extent = {{50, 44}, {70, 64}})));
  OpenHPL.ElectroMech.PowerSystem.Grid grid(
    Pgrid = 10e6,
    useLambda = true,
    Lambda = 10e6)
    annotation (Placement(transformation(origin = {-4, -78}, extent = {{80, 44}, {100, 64}})));
  OpenHPL.Controllers.HydroGovBlocks governor(
    f_n = data.f_grid,
    Pn = 10e6,
    Y_start = 0.6,
    f_speedctrl = 0.85 * data.f_grid,
    t_sync_hold = 5)
    annotation (Placement(transformation(origin = {-16, 49}, extent = {{-10, -10}, {10, 10}})));

  Modelica.Blocks.Math.Gain fToHz(k = data.f_grid)
    annotation (Placement(transformation(origin = {-52, -1}, extent = {{-10, -10}, {10, 10}}, rotation = 180)));
  Modelica.Blocks.Logical.Switch loadSwitch
    annotation (Placement(transformation(origin = {52, 69}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant zeroLoad(k = 0)
    annotation (Placement(transformation(origin = {11, 85}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.Constant connectedLoad(k = 6e6)
    annotation (Placement(transformation(origin = {11, 59}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.BooleanExpression startCommand(y = time >= 5 and time < 600)
    annotation (Placement(transformation(origin = {-55, 85}, extent = {{-10, -10}, {10, 10}})));
  Modelica.Blocks.Sources.BooleanExpression stopCommand(y = time >= 600)
    annotation (Placement(transformation(origin = {-65, 60}, extent = {{-10, -10}, {10, 10}})));

  inner OpenHPL.Data data(SteadyState = false, Vdot_0 = 0, f_0 = 0)
    annotation (Placement(transformation(origin = {-100, 85}, extent = {{-10, -10}, {10, 10}})));

equation
  connect(reservoir.o, intake.i) annotation (Line(points = {{-92, -62}, {-87, -62}}, color = {28, 108, 200}));
  connect(intake.o, surgeTank.i) annotation (Line(points = {{-67, -62}, {-57, -62}}, color = {28, 108, 200}));
  connect(surgeTank.o, penstock.i) annotation (Line(points = {{-37, -62}, {-17, -62}}, color = {28, 108, 200}));
  connect(penstock.o, turbine.i) annotation (Line(points = {{3, -62}, {13, -62}}, color = {28, 108, 200}));
  connect(turbine.o, discharge.i) annotation (Line(points = {{33, -62}, {63, -62}}, color = {28, 108, 200}));
  connect(discharge.o, tail.o) annotation (Line(points = {{83, -62}, {93, -62}}, color = {28, 108, 200}));

  connect(turbine.flange, simpleGen.flange) annotation (Line(points = {{23, -62}, {35.5, -62}, {35.5, -24}, {32, -24}}));
  connect(simpleGen.flange, mcb.genFlange) annotation (Line(points = {{32, -24}, {32, 27}, {56, 27}, {56, 2}}));
  connect(mcb.gridFlange, grid.flange) annotation (Line(points = {{76, 2}, {81, 2}, {81, -24}, {86, -24}}));
  connect(governor.Y_gv, turbine.u_t) annotation (Line(points = {{-5, 49}, {27, 49}, {27, -50}, {15, -50}}, color = {0, 0, 127}));
  connect(simpleGen.f, fToHz.u) annotation (Line(points = {{43, -28}, {70, -28}, {70, -1}, {-40, -1}}, color = {0, 0, 127}));
  connect(fToHz.y, governor.f) annotation (Line(points = {{-63, -1}, {4, -1}, {4, 39}, {-28, 39}}, color = {0, 0, 127}));

  connect(startCommand.y, governor.start) annotation (Line(points = {{-45, 85}, {-35, 85}, {-35, 57}, {-28, 57}}, color = {255, 0, 255}));
  connect(stopCommand.y, governor.stop) annotation (Line(points = {{-54, 60}, {-37.5, 60}, {-37.5, 53}, {-28, 53}}, color = {255, 0, 255}));
  connect(governor.syncCommand, mcb.closeCommand) annotation (Line(points = {{-5, 55}, {10, 55}, {10, 10}, {56, 10}}, color = {255, 0, 255}));
  connect(stopCommand.y, mcb.openCommand) annotation (Line(points = {{-54, 60}, {-30, 60}, {-30, 7}, {56, 7}}, color = {255, 0, 255}));
  connect(mcb.isClosed, governor.breakerClosed) annotation (Line(points = {{76, 8}, {75, 8}, {75, 90}, {-32, 90}, {-32, 45}, {-25, 45}}, color = {255, 0, 255}));
  connect(mcb.isClosed, loadSwitch.u2) annotation (Line(points = {{76, 8}, {76, 62}, {40, 62}, {40, 69}}, color = {255, 0, 255}));
  connect(zeroLoad.y, loadSwitch.u3) annotation (Line(points = {{22, 85}, {35, 85}, {35, 61}, {40, 61}}, color = {0, 0, 127}));
  connect(connectedLoad.y, loadSwitch.u1) annotation (Line(points = {{22, 59}, {30, 59}, {30, 77}, {40, 77}}, color = {0, 0, 127}));
  connect(zeroLoad.y, simpleGen.Pload) annotation (Line(points = {{22, 85}, {27, 85}, {27, -12}, {32, -12}}, color = {0, 0, 127}));
  connect(loadSwitch.y, grid.Pload) annotation (Line(points = {{63, 69}, {93, 69}, {93, -12}, {86, -12}}, color = {0, 0, 127}));

  connect(loadSwitch.y, governor.P_meas) annotation (Line(points = {{63, 69}, {-35, 69}, {-35, 42}, {-28, 42}}, color = {0, 0, 127}));
  connect(connectedLoad.y, governor.P_ref) annotation (Line(points = {{22, 59}, {30, 59}, {30, 45}, {-28, 45}}, color = {0, 0, 127}));
  annotation (experiment(
    StopTime = 800,
    StartTime = 0,
    Tolerance = 1e-06,
    Interval = 0.5), Documentation(info = "<html>
<h4>HydroGov Startup Example</h4>
<p>
This example uses the simple turbine and generator models, the MCB, and the grid
equivalent to demonstrate a unit cycle. At <code>t=5 s</code> the governor opens
the guide vanes. The governor's synchronisation request closes the MCB when relative shaft speed
is within its configured limit. Its <code>isClosed</code> output feeds the
governor's breaker feedback and switches the 6 MW grid load.
At <code>t=600 s</code> the MCB opens and the stop command closes the guide vanes.
</p>
<p>
The generator is connected to the turbine shaft, and the MCB connects the generator
to the mechanical grid equivalent. At <code>t=600 s</code>, the stop command opens
the MCB while the governor shuts down the unit.
</p>
</html>"));
end HydroGovStartup;