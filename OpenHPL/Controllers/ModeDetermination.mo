within OpenHPL.Controllers;
block ModeDetermination "Unit operating-mode state machine"
  parameter SI.Frequency f_speedctrl = 0 "Speed threshold for speed control" annotation (Dialog(group = "Sequence"));
  parameter SI.Frequency f_stopped = 0 "Speed threshold for stopped state" annotation (Dialog(group = "Sequence"));
  parameter Integer mode_start = STOPPED "Initial operating mode" annotation (Dialog(group = "Initialization"));

  constant Integer STOPPED = 0 "Unit at standstill";
  constant Integer STARTING = 1 "Open-loop start opening";
  constant Integer SPEED = 2 "Speed-no-load control";
  constant Integer POWER = 3 "Grid-connected power control";
  constant Integer SHUTDOWN = 4 "Closing guide vanes";

  Modelica.Blocks.Interfaces.BooleanInput start "Start command" annotation (Placement(transformation(origin={-180,80},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput stop "Stop command" annotation (Placement(transformation(origin={-180,20},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput breakerClosed "Generator breaker status" annotation (Placement(transformation(origin={-180,-20},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f "Measured unit frequency" annotation (Placement(transformation(origin={-180,-80},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.IntegerOutput mode(start = mode_start, fixed = true) "Current operating mode" annotation (Placement(transformation(origin={180,80},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {120, 60}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanOutput startingMode "True in starting mode" annotation (Placement(transformation(origin={180,40},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanOutput closedLoop "True in speed or power mode" annotation (Placement(transformation(origin={180,-20},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanOutput powerMode "True in power mode" annotation (Placement(transformation(origin={180,-80},    extent={{-20,-20},{20,20}}),      iconTransformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}})));

  Modelica.StateGraph.InitialStep stoppedStep(nIn = 1, nOut = 1) annotation (Placement(transformation(origin={-130,0},    extent={{-10,-10},{10,10}})));
  Modelica.StateGraph.Step startingStep(nIn = 1, nOut = 2) annotation (Placement(transformation(origin={-70,0},     extent={{-10,-10},{10,10}})));
  Modelica.StateGraph.Step speedStep(nIn = 2, nOut = 2) annotation (Placement(transformation(                  extent={{-20,-10},{0,10}})));
  Modelica.StateGraph.Step powerStep(nIn = 1, nOut = 2) annotation (Placement(transformation(origin={50,0},     extent={{-10,-10},{10,10}})));
  Modelica.StateGraph.Step shutdownStep(nIn = 3, nOut = 1) annotation (Placement(transformation(origin={110,0},    extent={{-10,-10},{10,10}})));
  Modelica.StateGraph.TransitionWithSignal startTransition annotation (Placement(transformation(origin={-100,0},    extent={{-6,6},{6,-6}})));
  Modelica.StateGraph.TransitionWithSignal speedTransition annotation (Placement(transformation(origin={-40,0},     extent={{-6,-6},{6,6}})));
  Modelica.StateGraph.TransitionWithSignal powerTransition annotation (Placement(transformation(origin={20,0},     extent={{-6,-6},{6,6}})));
  Modelica.StateGraph.TransitionWithSignal disconnectTransition annotation (Placement(transformation(                   extent={{-6,6},{6,-6}},      rotation=180,
        origin={40,-30})));
  Modelica.StateGraph.TransitionWithSignal shutdownTransition annotation (Placement(transformation(origin={20,20},    extent={{6,-6},{-6,6}},      rotation=180)));
  Modelica.StateGraph.TransitionWithSignal startingShutdownTransition annotation (Placement(transformation(origin={-40,40},    extent={{6,-6},{-6,6}},      rotation=180)));
  Modelica.StateGraph.TransitionWithSignal powerShutdownTransition annotation (Placement(transformation(origin={80,0},     extent={{6,-6},{-6,6}},      rotation=180)));
  Modelica.StateGraph.TransitionWithSignal stoppedTransition annotation (Placement(transformation(origin={128,-40},  extent={{-6,6},{6,-6}},      rotation=180)));
  Modelica.Blocks.Logical.Not breakerOpen annotation (Placement(transformation(origin={10,-60},     extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Logical.GreaterEqualThreshold speedReached(threshold = f_speedctrl) annotation (Placement(transformation(origin={-60,-60},    extent={{-8,-8},{8,8}})));
  Modelica.Blocks.Logical.LessEqualThreshold stoppedSpeed(threshold = f_stopped) annotation (Placement(transformation(origin={70,-80},    extent={{-8,-8},{8,8}})));

equation
  connect(stoppedStep.outPort[1], startTransition.inPort) annotation (Line(points={{-119.5,0},{-102.4,0}},  color = {0, 0, 0}));
  connect(startTransition.outPort, startingStep.inPort[1]) annotation (Line(points={{-99.1,0},{-81,0}},      color = {0, 0, 0}));
  connect(startingStep.outPort[1], speedTransition.inPort) annotation (Line(points={{-59.5,-0.125},{-48,0},{-42.4,0}},
                                                                                                             color = {0, 0, 0}));
  connect(speedTransition.outPort, speedStep.inPort[1]) annotation (Line(points={{-39.1,0},{-16,0},{-16,-0.25},{-21,-0.25}},
                                                                                                          color = {0, 0, 0}));
  connect(speedStep.outPort[1], powerTransition.inPort) annotation (Line(points={{0.5,-0.125},{14,-0.125},{14,0},{17.6,0}},
                                                                                                        color = {0, 0, 0}));
  connect(powerTransition.outPort, powerStep.inPort[1]) annotation (Line(points={{20.9,0},{39,0}},      color = {0, 0, 0}));
  connect(powerStep.outPort[1], disconnectTransition.inPort) annotation (Line(points={{60.5,-0.125},{66,-0.125},{66,-30},{42.4,-30}},
                                                                                                                       color = {0, 0, 0}));
  connect(disconnectTransition.outPort, speedStep.inPort[2]) annotation (Line(points={{39.1,-30},{39.1,-32},{-26,-32},{-26,0.25},{-21,0.25}},
                                                                                                                               color = {0, 0, 0}));
  connect(speedStep.outPort[2], shutdownTransition.inPort) annotation (Line(points={{0.5,0.125},{6,0.125},{6,20},{17.6,20}},
                                                                                                                     color = {0, 0, 0}));
  connect(shutdownTransition.outPort, shutdownStep.inPort[1]) annotation (Line(points={{20.9,20},{90,20},{90,-0.333333},{99,-0.333333}}));
  connect(startingStep.outPort[2], startingShutdownTransition.inPort) annotation (Line(points={{-59.5,0.125},{-54,0.125},{-54,40},{-42.4,40}},
                                                                                                                        color = {0, 0, 0}));
  connect(startingShutdownTransition.outPort, shutdownStep.inPort[2]) annotation (Line(points={{-39.1,40},{90,40},{90,0},{99,0}}));
  connect(powerStep.outPort[2], powerShutdownTransition.inPort) annotation (Line(points={{60.5,0.125},{62,0.125},{62,0},{77.6,0}},
                                                                                                                          color = {0, 0, 0}));
  connect(powerShutdownTransition.outPort, shutdownStep.inPort[3]) annotation (Line(points={{80.9,0},{80,0},{80,0.333333},{99,0.333333}}));
  connect(shutdownStep.outPort[1], stoppedTransition.inPort) annotation (Line(points={{120.5,0},{140,0},{140,-40},{130.4,-40}}));
  connect(stoppedTransition.outPort, stoppedStep.inPort[1]) annotation (Line(points={{127.1,-40},{-150,-40},{-150,0},{-141,0}}, color = {0, 0, 0}));
  connect(start, startTransition.condition) annotation (Line(points={{-180,80},{-100,80},{-100,7.2}},
                                                                                               color = {255, 0, 255}));
  connect(f, speedReached.u) annotation (Line(points={{-180,-80},{-80,-80},{-80,-60},{-69.6,-60}},
                                                                                              color = {0, 0, 127}));
  connect(speedReached.y, speedTransition.condition) annotation (Line(points={{-51.2,-60},{-40,-60},{-40,-7.2}},    color = {255, 0, 255}));
  connect(breakerClosed, breakerOpen.u) annotation (Line(points={{-180,-20},{-12,-20},{-12,-60},{0.4,-60}},
                                                                                             color = {255, 0, 255}));
  connect(breakerClosed, powerTransition.condition) annotation (Line(points={{-180,-20},{20,-20},{20,-7.2}},      color = {255, 0, 255}));
  connect(breakerOpen.y, disconnectTransition.condition) annotation (Line(points={{18.8,-60},{40,-60},{40,-37.2}},    color = {255, 0, 255}));
  connect(stop, startingShutdownTransition.condition) annotation (Line(points={{-180,20},{-140,20},{-140,60},{-40,60},{-40,47.2}},
                                                                                                                    color = {255, 0, 255}));
  connect(f, stoppedSpeed.u) annotation (Line(points={{-180,-80},{60.4,-80}},                 color = {0, 0, 127}));
  connect(stoppedSpeed.y, stoppedTransition.condition) annotation (Line(points={{78.8,-80},{128,-80},{128,-47.2}},color = {255, 0, 255}));
  mode = if stoppedStep.active then STOPPED else if startingStep.active then STARTING else if speedStep.active then SPEED else if powerStep.active then POWER else SHUTDOWN;
  startingMode = startingStep.active;
  closedLoop = speedStep.active or powerStep.active;
  powerMode = powerStep.active;

  connect(shutdownTransition.condition, startingShutdownTransition.condition) annotation (Line(points={{20,27.2},{20,60},{-40,60},{-40,47.2}}, color={255,0,255}));
  connect(powerShutdownTransition.condition, startingShutdownTransition.condition) annotation (Line(points={{80,7.2},{80,60},{-40,60},{-40,47.2}}, color={255,0,255}));
  annotation (Icon(coordinateSystem(extent={{-160,-100},{160,100}}),
                   graphics = {
    Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}),
    Text(extent = {{-86, 40}, {86, -10}}, textString = "MODE", textStyle = {TextStyle.Bold}),
    Text(extent = {{-90, -20}, {90, -60}}, textString = "0-1-2-3-4")}),
    Documentation(info = "<html>
<h4>Unit mode determination</h4>
<p>This reusable block determines the unit operating mode from the start, stop,
breaker and measured-frequency signals. The surrounding controller can use the
Boolean outputs to select Standard Library control blocks.</p>
</html>"),
    Diagram(coordinateSystem(extent={{-160,-100},{160,100}})));
end ModeDetermination;
