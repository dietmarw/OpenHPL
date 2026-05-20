within OpenHPLTest;
model GovernorDualModeTest "Focused test of dual-mode governor transitions"
  extends OpenHPL.Examples.GridOperationalCycle(
    t_startup = 60,
    t_sync = 90,
    t_disconnect = 260,
    t_idle_end = 340,
    t_shutdown_end = 420);

  annotation (experiment(StopTime = 450, StartTime = 0, Tolerance = 1e-06, Interval = 0.05));
end GovernorDualModeTest;
