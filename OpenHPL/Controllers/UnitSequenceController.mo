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

protected
  SI.Time startupSpan;
  SI.Time runSpan;
  SI.Time shutdownSpan;
  Real startupAlpha;
  Real runAlpha;
  Real shutdownAlpha;

equation
  startupSpan = max(t_startup, 1e-3);
  runSpan = max(t_disconnect - t_startup, 1e-3);
  shutdownSpan = max(t_shutdown_end - t_idle_end, 1e-3);

  startupAlpha = min(max(time / startupSpan, 0), 1);
  runAlpha = min(max((time - t_startup) / runSpan, 0), 1);
  shutdownAlpha = min(max((time - t_idle_end) / shutdownSpan, 0), 1);

  P_ref = if time < t_startup then
            P_start + (P_sync - P_start) * startupAlpha
          elseif time < t_disconnect then
            P_sync + (P_run - P_sync) * runAlpha
          elseif time < t_idle_end then
            P_idle
          elseif time < t_shutdown_end then
            P_idle * (1 - shutdownAlpha)
          else
            0;

  f_ref_speed = if time < t_startup then
                  f_idle + (f_sync - f_idle) * startupAlpha
                elseif time < t_disconnect then
                  f_sync
                elseif time < t_idle_end then
                  f_idle
                elseif time < t_shutdown_end then
                  f_idle + (f_shutdown - f_idle) * shutdownAlpha
                else
                  f_shutdown;

  startupActive = time < t_sync;
  connectedRunActive = time >= t_sync and time < t_disconnect;

  annotation (Documentation(info="<html>
<h4>Unit Sequence Controller</h4>
<p>
This controller generates smooth references for a typical operating cycle:
startup, synchronization window, connected run, disconnection, idling, and shutdown.
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
