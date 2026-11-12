within OpenHPL.ElectroMech.PowerSystem;
model MCB "Mechanical equivalent of an electrical Main Circuit Breaker"
  parameter SI.Time t_close = 100 "Time at which the MCB should close";
  parameter SI.Time t_open = 300 "Time at which the MCB should open";
  parameter SI.PerUnit deltaSpeed = 0.01 "Max allowed speed difference for closing the MCB";
  parameter Boolean useExternalControl = false "Use external close and open commands instead of fixed times";

  Modelica.Blocks.Logical.TriggeredTrapezoid triggeredTrapezoid(rising=0.01)
                                                                annotation (Placement(transformation(extent={{40,-40},{60,-20}})));
  Modelica.Blocks.Sources.ContinuousClock clock annotation (Placement(transformation(extent={{-92,-44},{-80,-32}})));
  Modelica.Blocks.Logical.GreaterEqualThreshold closeMCB(threshold=t_close) annotation (Placement(transformation(extent={{-66,-34},{-54,-22}})));
  Modelica.Blocks.Logical.GreaterEqualThreshold openMCB(threshold=t_open) annotation (Placement(transformation(extent={{-66,-54},{-54,-42}})));
  Modelica.Blocks.Logical.RSFlipFlop rSFlipFlop annotation (Placement(transformation(extent={{0,-46},{20,-26}})));
  Modelica.Blocks.Logical.LessThreshold lessThreshold(threshold=deltaSpeed*2*C.pi*data.f_0)
                                                                                      annotation (Placement(transformation(
        extent={{10,-10},{-10,10}},
        origin={0,50})));
  Modelica.Blocks.Logical.And and1 annotation (Placement(transformation(extent={{-40,-30},{-20,-10}})));
  Modelica.Mechanics.Rotational.Sensors.RelSpeedSensor relSpeedSensor annotation (Placement(transformation(extent={{60,40},{80,20}})));
  Modelica.Mechanics.Rotational.Components.Clutch clutch(fn_max=1e12)
                                                         annotation (Placement(transformation(extent={{60,10},{80,-10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a genFlange "Flange to be connected with the generator" annotation (Placement(transformation(extent={{-110,-10},{-90,10}}), iconTransformation(extent={{-110,-10},{-90,10}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b gridFlange "Flange to be connected with the grid" annotation (Placement(transformation(extent={{90,-10},{110,10}}), iconTransformation(extent={{90,-10},{110,10}})));
  Modelica.Blocks.Interfaces.BooleanInput closeCommand if useExternalControl "Request to close the MCB" annotation (Placement(transformation(extent={{-140,40},{-100,80}}),iconTransformation(extent={{-140,40},{-100,80}})));
  Modelica.Blocks.Interfaces.BooleanInput openCommand if useExternalControl "Request to open the MCB" annotation (Placement(transformation(extent={{-140,-80},{-100,-40}}),
                                                                                                                                                                         iconTransformation(extent={{-140,-80},{-100,-40}})));
  Modelica.Blocks.Interfaces.BooleanOutput isClosed "True when the MCB is closed" annotation (Placement(transformation(extent={{100,50},{120,70}}),iconTransformation(extent={{90,50},{110,70}})));

  inner Data data annotation (Placement(transformation(extent={{-100,80},{-80,100}})));
equation
  if useExternalControl then
    connect(closeCommand, and1.u2) annotation (Line(points={{-120,60},{-50,60},{-50,-28},{-42,-28}}, color={255,0,255}));
    connect(openCommand, rSFlipFlop.R) annotation (Line(points={{-120,-60},{-20,-60},{-20,-42},{-2,-42}},
                                                                                                        color={255,0,255}));
  else
    connect(closeMCB.y, and1.u2) annotation (Line(points={{-53.4,-28},{-42,-28}}, color={255,0,255}));
    connect(openMCB.y, rSFlipFlop.R) annotation (Line(points={{-53.4,-48},{-20,-48},{-20,-42},{-2,-42}},
                                                                                     color={255,0,255}));
  end if;
  connect(clock.y, closeMCB.u) annotation (Line(points={{-79.4,-38},{-72,-38},{-72,-28},{-67.2,-28}},
                                                                                                  color={0,0,127}));
  connect(relSpeedSensor.flange_a, clutch.flange_a) annotation (Line(points={{60,30},{50,30},{50,0},{60,0}}, color={0,0,0}));
  connect(relSpeedSensor.flange_b, clutch.flange_b) annotation (Line(points={{80,30},{88,30},{88,0},{80,0}}, color={0,0,0}));
  connect(clutch.flange_a, genFlange) annotation (Line(points={{60,0},{-100,0}}, color={0,0,0}));
  connect(clutch.flange_b, gridFlange) annotation (Line(points={{80,0},{100,0}},color={0,0,0}));
  connect(relSpeedSensor.w_rel, lessThreshold.u) annotation (Line(points={{70,41},{70,50},{12,50}}, color={0,0,127}));
  connect(triggeredTrapezoid.y, clutch.f_normalized) annotation (Line(points={{61,-30},{70,-30},{70,-11}},
                                                                                                        color={0,0,127}));
  connect(clock.y, openMCB.u) annotation (Line(points={{-79.4,-38},{-72,-38},{-72,-48},{-67.2,-48}},
                                                                                                 color={0,0,127}));
  connect(rSFlipFlop.Q, triggeredTrapezoid.u) annotation (Line(points={{21,-30},{38,-30}}, color={255,0,255}));
  connect(and1.y, rSFlipFlop.S) annotation (Line(points={{-19,-20},{-10,-20},{-10,-30},{-2,-30}}, color={255,0,255}));
  connect(lessThreshold.y, and1.u1) annotation (Line(points={{-11,50},{-46,50},{-46,-20},{-42,-20}}, color={255,0,255}));
  connect(rSFlipFlop.Q, isClosed) annotation (Line(points={{21,-30},{30,-30},{30,60},{110,60}}, color={255,0,255}));
  annotation (
    Documentation(info="<html>
<h4>Main Circuit Breaker</h4>
<p>
By default, the breaker requests closure after <code>t_close</code> and opening
after <code>t_open</code>. Closure is additionally gated by the relative-speed
limit <code>deltaSpeed</code>.
</p>
<p>
Set <code>useExternalControl</code> to use <code>closeCommand</code> and
<code>openCommand</code> instead of the fixed times. The relative-speed limit
still applies to a close request. <code>isClosed</code> reports the breaker state.
</p>
</html>"),
    Diagram,
    Icon(graphics={
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}),
        Polygon(
          points={{6,-20},{-14,-44},{1.375,-44},{-14,-62},{14,-38},{0,-38},{20,-20},{6,-20}},
          lineColor={0,0,0},
          fillColor={238,46,47},
          fillPattern=FillPattern.Solid),
        Line(points={{-100,0},{-40,0},{32,60}}, color={0,0,0}),
        Line(points={{38,0},{100,0}}),
        Text(
          textColor={127,0,0},
          extent={{-100,-110},{100,-150}},
          textStyle={TextStyle.Bold},
          textString="%name"),
        Text(
          extent={{-90,30},{-50,8}},
          textColor={0,0,0},
          textString="gen"),
        Text(
          extent={{50,32},{90,10}},
          textColor={0,0,0},
          textString="grid")}));
end MCB;
