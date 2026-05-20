within OpenHPL.Examples;
model GridOperationalCycle "Hydropower unit startup, synchronization, grid run, disconnect, idle and shutdown"
  extends Modelica.Icons.Example;

  parameter SI.Time t_startup = 180 "End time of startup ramp";
  parameter SI.Time t_sync = 220 "Time to request synchronization and breaker close";
  parameter SI.Time t_disconnect = 700 "Time to disconnect from grid";
  parameter SI.Time t_idle_end = 900 "End time of post-disconnect idle phase";
  parameter SI.Time t_shutdown_end = 1000 "End time of shutdown ramp";

  OpenHPL.Waterway.Reservoir reservoir(h_0 = 10, fixElevation = true, z_0 = 100) annotation (Placement(transformation(
        origin={-90,30},
        extent={{-10,-10},{10,10}})));
  OpenHPL.Waterway.Pipe intake(H = 10, D_i = 5) annotation (Placement(transformation(extent={{-70,20},{-50,40}})));
  OpenHPL.Waterway.Pipe discharge(H = 0.5, L = 600, D_i = 5) annotation (Placement(transformation(extent={{50,-10},{70,10}})));
  OpenHPL.Waterway.Reservoir tail(h_0 = 5) annotation (Placement(transformation(
        origin={90,0},
        extent={{-10,10},{10,-10}},
        rotation=180)));
  replaceable OpenHPL.Waterway.Pipe penstock(
    D_i = 3,
    D_o = 3,
    H = 80,
    L = 200,
    slanted = true) constrainedby Interfaces.TwoContacts annotation (Placement(transformation(origin={0,30}, extent={{-10,-10},{10,10}})));
  OpenHPL.Waterway.SurgeTank surgeTank(H = 25, L = 30, h_0 = 20)
    annotation (Placement(transformation(
        origin={-30,30},
        extent={{-10,-10},{10,10}})));

  OpenHPL.ElectroMech.Turbines.Turbine turbine(ValveCapacity = false, enable_nomSpeed = false, enable_P_out = true)
    annotation (Placement(transformation(origin={30,10}, extent={{-10,-10},{10,10}})));
  OpenHPL.ElectroMech.Generators.SimpleGen simpleGen(enable_f = true) annotation (Placement(transformation(extent={{20,46},{40,66}})));
  OpenHPL.ElectroMech.PowerSystem.MCB mcb(t_close = t_sync, t_open = t_disconnect, deltaSpeed = 0.01) annotation (Placement(transformation(extent={{50,44},{70,64}})));
  OpenHPL.ElectroMech.PowerSystem.Grid grid(
    Pgrid = 400e6,
    useLambda = true,
    Lambda = 266.6e6,
    J = 1e6,
    enable_f = true) annotation (Placement(transformation(extent={{80,44},{100,64}})));

  OpenHPL.Controllers.GovernorDualMode governorDualMode(Pn = 103e6, Y_gv_ref = 0.12, T_mode = 2) annotation (Placement(transformation(extent={{-16,54},{4,74}})));
  OpenHPL.Controllers.UnitSequenceController unitSequenceController(
    t_startup = t_startup,
    t_sync = t_sync,
    t_disconnect = t_disconnect,
    t_idle_end = t_idle_end,
    t_shutdown_end = t_shutdown_end,
    P_start = 4e6,
    P_sync = 12e6,
    P_run = 90e6,
    P_idle = 8e6) annotation (Placement(transformation(extent={{-54,54},{-34,74}})));

  Modelica.Blocks.Math.Gain fToHz(k = data.f_0) annotation (Placement(transformation(extent={{-16,26},{4,46}})));
  Modelica.Blocks.Sources.Constant noLocalLoad(k = 0) annotation (Placement(transformation(extent={{-10,88},{10,108}})));
  Modelica.Blocks.Sources.Step gridLoadStep(
    height = 60e6,
    offset = 120e6,
    startTime = 420) annotation (Placement(transformation(extent={{110,80},{90,100}})));

  inner OpenHPL.Data data(SteadyState = false, Vdot_0 = 3) annotation (Placement(transformation(origin={-90,90}, extent={{-10,-10},{10,10}})));

equation
  connect(reservoir.o, intake.i) annotation (Line(points={{-80,30},{-70,30}}, color={28,108,200}));
  connect(intake.o, surgeTank.i) annotation (Line(points={{-50,30},{-40,30}}, color={28,108,200}));
  connect(surgeTank.o, penstock.i) annotation (Line(points={{-20,30},{-10,30}}, color={28,108,200}));
  connect(penstock.o, turbine.i) annotation (Line(points={{10,30},{15,30},{15,10},{20,10}}, color={28,108,200}));
  connect(turbine.o, discharge.i) annotation (Line(points={{40,10},{44,10},{44,0},{50,0}}, color={28,108,200}));
  connect(discharge.o, tail.o) annotation (Line(points={{70,0},{80,0}}, color={28,108,200}));

  connect(turbine.flange, simpleGen.flange) annotation (Line(points={{30,10},{30,56}}, color={0,0,0}));
  connect(simpleGen.flange, mcb.genFlange) annotation (Line(points={{30,56},{50,54}}, color={0,0,0}));
  connect(mcb.gridFlange, grid.flange) annotation (Line(points={{69.8,54},{90,54}}, color={0,0,0}));

  connect(unitSequenceController.P_ref, governorDualMode.P_ref) annotation (Line(points={{-33,68},{-18,68}}, color={0,0,127}));
  connect(unitSequenceController.f_ref_speed, governorDualMode.f_ref_speed) annotation (Line(points={{-33,64},{-24,64},{-24,56},{-18,56}}, color={0,0,127}));
  connect(mcb.isClosed, governorDualMode.isGridConnected) annotation (Line(points={{70,60},{74,60},{74,82},{-26,82},{-26,72},{-18,72}}, color={255,0,255}));
  connect(simpleGen.f, fToHz.u) annotation (Line(points={{41,50},{52,50},{52,14},{-26,14},{-26,36},{-18,36}}, color={0,0,127}));
  connect(fToHz.y, governorDualMode.f) annotation (Line(points={{5,36},{8,36},{8,60},{-18,60}}, color={0,0,127}));
  connect(governorDualMode.Y_gv, turbine.u_t) annotation (Line(points={{5,64},{12,64},{12,22},{22,22}}, color={0,0,127}));

  connect(noLocalLoad.y, simpleGen.Pload) annotation (Line(points={{11,98},{30,98},{30,68}}, color={0,0,127}));
  connect(gridLoadStep.y, grid.Pload) annotation (Line(points={{89,90},{80,90},{80,66}}, color={0,0,127}));

  annotation (experiment(StopTime = 1100, StartTime = 0, Tolerance = 1e-06, Interval = 0.1), Documentation(info="<html>
<h4>Grid Operational Cycle</h4>
<p>
This example demonstrates a complete unit cycle using the dual-mode governor and sequence controller:
startup, synchronization, connected run, disconnection, idling, and shutdown.
</p>
<p>
The governor mode is driven by the MCB status output.
When the breaker is closed, power+droop control is active.
When open, the governor follows the speed reference from the sequence controller.
</p>
</html>"));
end GridOperationalCycle;
