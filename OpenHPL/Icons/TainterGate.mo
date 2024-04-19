within OpenHPL.Icons;
partial class TainterGate "Icon of a tainter gate"
  annotation (Icon(graphics={
        Rectangle(
          extent={{-100,-2},{-46,-72}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Ellipse(
          extent={{-60,140},{220,-140}},
          lineColor={135,135,135},
          startAngle=155,
          endAngle=196,
          fillColor={175,175,175},
          fillPattern=FillPattern.Solid),
        Rectangle(
          extent={{-100,-72},{100,-80}},
          lineColor={0,0,0},
          fillColor={215,215,215},
          fillPattern=FillPattern.Forward),
        Rectangle(
          extent={{-46,-60},{100,-72}},
          lineColor={28,108,200},
          fillColor={28,108,200},
          fillPattern=FillPattern.Solid),
        Rectangle(extent={{-100,100},{100,-100}}, lineColor={0,0,0}),
                     Text(
          lineColor={28,108,200},
          extent={{-90,100},{90,60}},
          textString="%name",
          textStyle={TextStyle.Bold}),
        Ellipse(
          extent={{-2,-4},{2,-8}},
          lineColor={0,0,0},
          fillColor={0,0,0},
          fillPattern=FillPattern.Solid),
        Line(
          points={{0,-6},{0,100}},
          color={0,0,0},
          pattern=LinePattern.Dash)}));
end TainterGate;
