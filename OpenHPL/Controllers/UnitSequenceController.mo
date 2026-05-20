within OpenHPL.Controllers;
model UnitSequenceController "Time-sequenced startup, run, disconnect, idle and shutdown references"
  outer Data data "Using standard class with constants";

  parameter SI.Time t_startup = 180 "End time of startup ramp" annotation (
    Dialog(group = "Timing"));
  parameter SI.Time t_sync = 220 "Time to request synchronization and breaker close" annotation (
    Dialog(group = "Timing"));
  parameter SI.Time t_disconnect = 700 "Time to disconnect from grid" annotation (
    Dialog(group = "Timing"));
  parameter SI.Time t_idle_end = 900 "End time of post-disconnect idle phase" annotation (
    Dialog(group = "Timing"));
  parameter SI.Time t_shutdown_end = 1000 "End time of shutdown ramp" annotation (
    Dialog(group = "Timing"));

  parameter SI.Power P_start = 4e6 "Initial startup power reference" annotation (
    Dialog(group = "Power settings"));
  parameter SI.Power P_sync = 12e6 "Power reference near synchronization" annotation (
    Dialog(group = "Power settings"));
  parameter SI.Power P_run = 90e6 "Power reference during connected operation" annotation (
    Dialog(group = "Power settings"));
  parameter SI.Power P_idle = 8e6 "Power reference in idle mode" annotation (
    Dialog(group = "Power settings"));

  parameter SI.Frequency f_idle = 0.985 * data.f_0 "Speed reference in idle mode" annotation (
    Dialog(group = "Speed settings"));
  parameter SI.Frequency f_sync = data.f_0 "Speed reference near synchronization" annotation (
    Dialog(group = "Speed settings"));
  parameter SI.Frequency f_shutdown = 0.92 * data.f_0 "Target speed reference at shutdown" annotation (
    Dialog(group = "Speed settings"));

  Modelica.Blocks.Interfaces.RealOutput P_ref(unit = "W") annotation (
    Placement(transformation(extent = {{100, 30}, {120, 50}}), iconTransformation(extent = {{100, 30}, {120, 50}})));
  Modelica.Blocks.Interfaces.RealOutput f_ref_speed(unit = "Hz") annotation (
    Placement(transformation(extent = {{100, -10}, {120, 10}}), iconTransformation(extent = {{100, -10}, {120, 10}})));
  Modelica.Blocks.Interfaces.BooleanOutput startupActive annotation (
    Placement(transformation(extent = {{100, -50}, {120, -30}}), iconTransformation(extent = {{100, -50}, {120, -30}})));
  Modelica.Blocks.Interfaces.BooleanOutput connectedRunActive annotation (
    Placement(transformation(extent = {{100, -90}, {120, -70}}), iconTransformation(extent = {{100, -90}, {120, -70}})));
  Modelica.Blocks.Sources.TimeTable powerSchedule(table = [0, P_start; t_startup, P_sync; t_disconnect, P_run; t_idle_end, P_idle; t_shutdown_end, 0], offset = 0) annotation (
    Placement(transformation(extent = {{-80, 30}, {-60, 50}})));
  Modelica.Blocks.Sources.TimeTable speedSchedule(table = [0, f_idle; t_startup, f_sync; t_disconnect, f_idle; t_idle_end, f_idle; t_shutdown_end, f_shutdown], offset = 0) annotation (
    Placement(transformation(extent = {{-80, -10}, {-60, 10}})));
  Modelica.Blocks.Sources.BooleanTable startupSchedule(table = {t_sync}, startValue = true) annotation (
    Placement(transformation(extent = {{-80, -50}, {-60, -30}})));
  Modelica.Blocks.Sources.BooleanTable connectedRunSchedule(table = {t_sync, t_disconnect}, startValue = false) annotation (
    Placement(transformation(extent = {{-80, -90}, {-60, -70}})));

equation
  connect(powerSchedule.y, P_ref) annotation (Line(points={{-59,40},{110,40}}, color={0,0,127}));
  connect(speedSchedule.y, f_ref_speed) annotation (Line(points={{-59,0},{20,0},{20,0},{110,0}}, color={0,0,127}));
  connect(startupSchedule.y, startupActive) annotation (Line(points={{-59,-40},{110,-40}}, color={255,0,255}));
  connect(connectedRunSchedule.y, connectedRunActive) annotation (Line(points={{-59,-80},{110,-80}}, color={255,0,255}));

  annotation (Documentation(info="<html>
<h4>Unit Sequence Controller</h4>
<p>
This controller generates smooth references for a typical operating cycle:
startup, synchronization window, connected run, disconnection, idling, and shutdown.
</p>
<p>
The schedules are implemented with explicit time-table and Boolean-table blocks, so the
transitions are driven by time events only.
</p>
<p>
Use this model together with <code>GovernorDualMode</code> and the MCB component.
The MCB is still responsible for the physical connect/disconnect event.
</p>
</html>"), Icon(graphics={
    Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}),
    Text(extent={{-88,76},{88,34}}, textString="SEQ", textStyle={TextStyle.Bold}),
    Text(extent={{-90,14},{90,-6}}, textString="startup-run-idle"),
    Text(extent={{-100,-110},{100,-140}}, textColor={127,0,0}, textStyle={TextStyle.Bold}, textString="%name")}));
end UnitSequenceController;
