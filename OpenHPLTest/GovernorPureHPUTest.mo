within OpenHPLTest;
model GovernorPureHPUTest "Focused test for GovernorPure + HPUDynamics without hydraulic network"
  extends Modelica.Icons.Example;

  parameter Modelica.Units.SI.Power Pn = 103e6 "Rated power used in synthetic plant";
  parameter Modelica.Units.SI.Power P_init = 6e6 "Initial generated power";

  OpenHPL.Controllers.GovernorPure governor(
    Pn = Pn,
    enableDroop = true,
    Kp_power = 0.8,
    Ki_power = 0.06,
    Kp_speed = 2.0,
    Ki_speed = 0.4,
    u_start = 0.12) annotation (Placement(transformation(extent={{-12,20},{8,40}})));
  OpenHPL.Controllers.HPUDynamics hpu(
    T_servo = 0.25,
    openRateMax = 0.05,
    closeRateMax = 0.2,
    u_start = 0.12) annotation (Placement(transformation(extent={{20,20},{40,40}})));

  Modelica.Blocks.Sources.TimeTable pRefSched(
    table = [0, 8e6; 120, 15e6; 260, 70e6; 520, 90e6; 700, 15e6; 860, 5e6],
    offset = 0) annotation (Placement(transformation(extent={{-96,52},{-76,72}})));
  Modelica.Blocks.Sources.TimeTable fRefSched(
    table = [0, 0.985*data.f_0; 120, data.f_0; 700, 0.985*data.f_0; 860, 0.94*data.f_0],
    offset = 0) annotation (Placement(transformation(extent={{-96,-2},{-76,18}})));
  Modelica.Blocks.Sources.BooleanTable gridConn(table={220, 700}, startValue=false) annotation (Placement(transformation(extent={{-96,24},{-76,44}})));

  Modelica.Blocks.Math.Gain openingToPower(k = Pn) annotation (Placement(transformation(extent={{52,20},{72,40}})));
  Modelica.Blocks.Continuous.FirstOrder pPlant(
    T = 8,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = P_init) annotation (Placement(transformation(extent={{86,20},{106,40}})));
  Modelica.Blocks.Sources.Step pDisturbance(
    height = -10e6,
    offset = 0,
    startTime = 430) annotation (Placement(transformation(extent={{86,54},{106,74}})));
  Modelica.Blocks.Math.Add pMeasured(k1=1, k2=1) annotation (Placement(transformation(extent={{118,22},{138,42}})));

  Modelica.Blocks.Math.Add pMismatch(k1 = 1, k2 = -1) annotation (Placement(transformation(extent={{-40,-48},{-20,-28}})));
  Modelica.Blocks.Math.Gain mismatchToHz(k = 0.03*data.f_0/Pn) annotation (Placement(transformation(extent={{-8,-48},{12,-28}})));
  Modelica.Blocks.Math.Add islandFreq(k1 = 1, k2 = -1) annotation (Placement(transformation(extent={{24,-48},{44,-28}})));
  Modelica.Blocks.Sources.Constant fNom(k = data.f_0) annotation (Placement(transformation(extent={{-8,-78},{12,-58}})));
  Modelica.Blocks.Continuous.FirstOrder islandFreqDyn(
    T = 5,
    initType = Modelica.Blocks.Types.Init.InitialOutput,
    y_start = data.f_0) annotation (Placement(transformation(extent={{56,-48},{76,-28}})));

  Modelica.Blocks.Logical.Switch fSelect annotation (Placement(transformation(extent={{92,-10},{112,10}})));
  Modelica.Blocks.Sources.Constant fGrid(k = data.f_0) annotation (Placement(transformation(extent={{56,6},{76,26}})));

  inner OpenHPL.Data data annotation (Placement(transformation(extent={{-100,86},{-80,106}})));

equation
  connect(pRefSched.y, governor.P_ref) annotation (Line(points={{-75,62},{-22,62},{-22,34},{-14,34}}, color={0,0,127}));
  connect(fRefSched.y, governor.f_ref_speed) annotation (Line(points={{-75,8},{-28,8},{-28,22},{-14,22}}, color={0,0,127}));
  connect(gridConn.y, governor.isGridConnected) annotation (Line(points={{-75,34},{-26,34},{-26,38},{-14,38}}, color={255,0,255}));

  connect(governor.u_ref, hpu.u_ref) annotation (Line(points={{9,30},{18,30}}, color={0,0,127}));
  connect(hpu.u_t, openingToPower.u) annotation (Line(points={{41,30},{50,30}}, color={0,0,127}));
  connect(openingToPower.y, pPlant.u) annotation (Line(points={{73,30},{84,30}}, color={0,0,127}));
  connect(pPlant.y, pMeasured.u1) annotation (Line(points={{107,30},{116,38}}, color={0,0,127}));
  connect(pDisturbance.y, pMeasured.u2) annotation (Line(points={{107,64},{112,64},{112,26},{116,26}}, color={0,0,127}));
  connect(pMeasured.y, governor.P_meas) annotation (Line(points={{139,32},{146,32},{146,-12},{-28,-12},{-28,30},{-14,30}}, color={0,0,127}));

  connect(governor.P_ref, pMismatch.u1) annotation (Line(points={{-14,34},{-52,34},{-52,-32},{-42,-32}}, color={0,0,127}));
  connect(governor.P_meas, pMismatch.u2) annotation (Line(points={{-14,30},{-16,30},{-16,6},{-52,6},{-52,-44},{-42,-44}}, color={0,0,127}));
  connect(pMismatch.y, mismatchToHz.u) annotation (Line(points={{-19,-38},{-10,-38}}, color={0,0,127}));
  connect(fNom.y, islandFreq.u1) annotation (Line(points={{13,-68},{18,-68},{18,-32},{22,-32}}, color={0,0,127}));
  connect(mismatchToHz.y, islandFreq.u2) annotation (Line(points={{13,-38},{22,-38},{22,-44}}, color={0,0,127}));
  connect(islandFreq.y, islandFreqDyn.u) annotation (Line(points={{45,-38},{54,-38}}, color={0,0,127}));

  connect(gridConn.y, fSelect.u2) annotation (Line(points={{-75,34},{82,34},{82,0},{90,0}}, color={255,0,255}));
  connect(fGrid.y, fSelect.u1) annotation (Line(points={{77,16},{84,16},{84,8},{90,8}}, color={0,0,127}));
  connect(islandFreqDyn.y, fSelect.u3) annotation (Line(points={{77,-38},{84,-38},{84,-8},{90,-8}}, color={0,0,127}));
  connect(fSelect.y, governor.f) annotation (Line(points={{113,0},{124,0},{124,-22},{-24,-22},{-24,26},{-14,26}}, color={0,0,127}));

  annotation (experiment(StopTime=1000, StartTime=0, Tolerance=1e-06, Interval=0.1),
    Documentation(info="<html>
<h4>GovernorPureHPUTest</h4>
<p>
Focused tuning model for the split control architecture.
It avoids waterway and turbine-fluid dynamics by using a synthetic plant for power and frequency feedback.
</p>
<p>
Use this test for fast tuning of:
</p>
<ul>
  <li>speed mode PI gains</li>
  <li>power mode PI gains</li>
  <li>droop enable/disable behavior</li>
  <li>HPU actuator and slew-rate settings</li>
</ul>
</html>"));
end GovernorPureHPUTest;
