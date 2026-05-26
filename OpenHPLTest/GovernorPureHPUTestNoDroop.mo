within OpenHPLTest;
model GovernorPureHPUTestNoDroop "Focused test variant with droop disabled"
  extends GovernorPureHPUTest(governor(enableDroop = false));

  annotation (experiment(StopTime=1000, StartTime=0, Tolerance=1e-06, Interval=0.1),
    Documentation(info="<html>
<h4>GovernorPureHPUTestNoDroop</h4>
<p>
A/B companion case for <code>GovernorPureHPUTest</code> with droop disabled.
Use this model to compare connected-mode frequency-power behavior against the droop-enabled case.
</p>
</html>"));
end GovernorPureHPUTestNoDroop;
