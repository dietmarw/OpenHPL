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

  Modelica.Blocks.Interfaces.BooleanInput start "Start command" annotation (Placement(transformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 60}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput stop "Stop command" annotation (Placement(transformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, 20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanInput breakerClosed "Generator breaker status" annotation (Placement(transformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.RealInput f "Measured unit frequency" annotation (Placement(transformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {-120, -60}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.IntegerOutput mode(start = mode_start, fixed = true) "Current operating mode" annotation (Placement(transformation(origin = {120, 60}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {120, 60}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanOutput startingMode "True in starting mode" annotation (Placement(transformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {120, 20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanOutput closedLoop "True in speed or power mode" annotation (Placement(transformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {120, -20}, extent = {{-20, -20}, {20, 20}})));
  Modelica.Blocks.Interfaces.BooleanOutput powerMode "True in power mode" annotation (Placement(transformation(origin = {118, -58}, extent = {{-20, -20}, {20, 20}}), iconTransformation(origin = {120, -60}, extent = {{-20, -20}, {20, 20}})));

  Modelica.StateGraph.InitialStep stoppedStep(nIn = 1, nOut = 1) annotation (Placement(transformation(origin = {-70, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.StateGraph.Step startingStep(nIn = 1, nOut = 2) annotation (Placement(transformation(origin = {-35, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.StateGraph.Step speedStep(nIn = 2, nOut = 2) annotation (Placement(transformation(origin = {0, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.StateGraph.Step powerStep(nIn = 1, nOut = 2) annotation (Placement(transformation(origin = {35, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.StateGraph.Step shutdownStep(nIn = 3, nOut = 1) annotation (Placement(transformation(origin = {74, 60}, extent = {{-10, -10}, {10, 10}})));
  Modelica.StateGraph.TransitionWithSignal startTransition annotation (Placement(transformation(origin = {-52, 60}, extent = {{-6, -6}, {6, 6}})));
  Modelica.StateGraph.TransitionWithSignal speedTransition annotation (Placement(transformation(origin = {-17, 60}, extent = {{-6, -6}, {6, 6}})));
  Modelica.StateGraph.TransitionWithSignal powerTransition annotation (Placement(transformation(origin = {18, 60}, extent = {{-6, -6}, {6, 6}})));
  Modelica.StateGraph.TransitionWithSignal disconnectTransition annotation (Placement(transformation(origin = {18, 25}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.StateGraph.TransitionWithSignal shutdownTransition annotation (Placement(transformation(origin = {35, 25}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.StateGraph.TransitionWithSignal startingShutdownTransition annotation (Placement(transformation(origin = {-35, 25}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.StateGraph.TransitionWithSignal powerShutdownTransition annotation (Placement(transformation(origin = {52, 25}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.StateGraph.TransitionWithSignal stoppedTransition annotation (Placement(transformation(origin = {70, 25}, extent = {{-6, -6}, {6, 6}}, rotation = 90)));
  Modelica.Blocks.Logical.Not breakerOpen annotation (Placement(transformation(origin = {-30, -20}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Logical.GreaterEqualThreshold speedReached(threshold = f_speedctrl) annotation (Placement(transformation(origin = {-65, -20}, extent = {{-8, -8}, {8, 8}})));
  Modelica.Blocks.Logical.LessEqualThreshold stoppedSpeed(threshold = f_stopped) annotation (Placement(transformation(origin = {41, -30}, extent = {{-8, -8}, {8, 8}})));

equation
  connect(stoppedStep.outPort[1], startTransition.inPort) annotation (Line(points = {{-60, 60}, {-58, 60}}, color = {0, 0, 0}));
  connect(startTransition.outPort, startingStep.inPort[1]) annotation (Line(points = {{-46, 60}, {-45, 60}}, color = {0, 0, 0}));
  connect(startingStep.outPort[1], speedTransition.inPort) annotation (Line(points = {{-25, 60}, {-23, 60}}, color = {0, 0, 0}));
  connect(speedTransition.outPort, speedStep.inPort[1]) annotation (Line(points = {{-11, 60}, {-10, 60}}, color = {0, 0, 0}));
  connect(speedStep.outPort[1], powerTransition.inPort) annotation (Line(points = {{10, 60}, {12, 60}}, color = {0, 0, 0}));
  connect(powerTransition.outPort, powerStep.inPort[1]) annotation (Line(points = {{24, 60}, {25, 60}}, color = {0, 0, 0}));
  connect(powerStep.outPort[1], disconnectTransition.inPort) annotation (Line(points = {{45, 60}, {45, 25}, {24, 25}}, color = {0, 0, 0}));
  connect(disconnectTransition.outPort, speedStep.inPort[2]) annotation (Line(points = {{18, 31}, {18, 40}, {0, 40}, {0, 50}}, color = {0, 0, 0}));
  connect(speedStep.outPort[2], shutdownTransition.inPort) annotation (Line(points = {{10, 60}, {35, 60}, {35, 31}}, color = {0, 0, 0}));
  connect(shutdownTransition.outPort, shutdownStep.inPort[1]) annotation (Line(points = {{35, 19}, {35, 10}, {63, 10}, {63, 60}}));
  connect(startingStep.outPort[2], startingShutdownTransition.inPort) annotation (Line(points = {{-35, 50}, {-35, 31}}, color = {0, 0, 0}));
  connect(startingShutdownTransition.outPort, shutdownStep.inPort[2]) annotation (Line(points = {{-35, 19}, {-35, 10}, {63, 10}, {63, 60}}));
  connect(powerStep.outPort[2], powerShutdownTransition.inPort) annotation (Line(points = {{35, 50}, {52, 50}, {52, 31}}, color = {0, 0, 0}));
  connect(powerShutdownTransition.outPort, shutdownStep.inPort[3]) annotation (Line(points = {{52, 19}, {52, 10}, {63, 10}, {63, 60}}));
  connect(shutdownStep.outPort[1], stoppedTransition.inPort) annotation (Line(points = {{84.5, 60}, {90, 60}, {90, 25}, {76, 25}}));
  connect(stoppedTransition.outPort, stoppedStep.inPort[1]) annotation (Line(points = {{70, 19}, {70, 0}, {-70, 0}, {-70, 50}}, color = {0, 0, 0}));
  connect(start, startTransition.condition) annotation (Line(points = {{-120, 60}, {-58, 60}}, color = {255, 0, 255}));
  connect(f, speedReached.u) annotation (Line(points = {{-120, -60}, {-65, -60}, {-65, -30}}, color = {0, 0, 127}));
  connect(speedReached.y, speedTransition.condition) annotation (Line(points = {{-55, -20}, {-17, -20}, {-17, 54}}, color = {255, 0, 255}));
  connect(breakerClosed, breakerOpen.u) annotation (Line(points = {{-120, -20}, {-38, -20}}, color = {255, 0, 255}));
  connect(breakerClosed, powerTransition.condition) annotation (Line(points = {{-120, -20}, {18, -20}, {18, 54}}, color = {255, 0, 255}));
  connect(breakerOpen.y, disconnectTransition.condition) annotation (Line(points = {{-21, -20}, {18, -20}, {18, 19}}, color = {255, 0, 255}));
  connect(stop, shutdownTransition.condition) annotation (Line(points = {{-120, 20}, {35, 20}, {35, 19}}, color = {255, 0, 255}));
  connect(stop, startingShutdownTransition.condition) annotation (Line(points = {{-120, 20}, {-35, 20}, {-35, 19}}, color = {255, 0, 255}));
  connect(stop, powerShutdownTransition.condition) annotation (Line(points = {{-120, 20}, {52, 20}, {52, 19}}, color = {255, 0, 255}));
  connect(f, stoppedSpeed.u) annotation (Line(points = {{-120, -60}, {-120, -30}, {31, -30}}, color = {0, 0, 127}));
  connect(stoppedSpeed.y, stoppedTransition.condition) annotation (Line(points = {{50, -30}, {50, 19}, {70, 19}}, color = {255, 0, 255}));
  mode = if stoppedStep.active then STOPPED else if startingStep.active then STARTING else if speedStep.active then SPEED else if powerStep.active then POWER else SHUTDOWN;
  startingMode = startingStep.active;
  closedLoop = speedStep.active or powerStep.active;
  powerMode = powerStep.active;

  annotation (Icon(graphics = {
    Rectangle(extent = {{-100, 100}, {100, -100}}, lineColor = {0, 0, 0}),
    Text(extent = {{-86, 40}, {86, -10}}, textString = "MODE", textStyle = {TextStyle.Bold}),
    Text(extent = {{-90, -20}, {90, -60}}, textString = "0-1-2-3-4")}),
    Documentation(info = "<html>
<h4>Unit mode determination</h4>
<p>This reusable block determines the unit operating mode from the start, stop,
breaker and measured-frequency signals. The surrounding controller can use the
Boolean outputs to select Standard Library control blocks.</p>
</html>"));
end ModeDetermination;