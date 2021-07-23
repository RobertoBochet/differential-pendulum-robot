within PendulumRobot;

model RollingWheelSetTiltingMass
  "Ideal rolling wheel set consisting of two ideal rolling wheels connected together by an axis"
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frameMiddle
    "Frame fixed in middle of axis connecting both wheels (y-axis: along wheel axis, z-axis: upwards)"
    annotation (Placement(transformation(extent={{-16,16},{16,-16}}),
        iconTransformation(extent={{-16,-16},{16,16}},
        rotation=90,
        origin={0,-20})));

  parameter Boolean animation=true
    "= true, if animation of wheel set shall be enabled";

  parameter Modelica.SIunits.Radius wheelRadius "Radius of one wheel";
  parameter Modelica.SIunits.Thickness wheelThickness=0.01 "Thickness of one wheel";
  parameter Modelica.SIunits.Distance wheelDistance "Distance between the two wheels";
  parameter StateSelect stateSelect=StateSelect.prefer
    "Priority to use the generalized coordinates as states";
  Modelica.SIunits.Angle theta1(
    start=0,
    stateSelect=stateSelect) "Angle of wheel 1";
  Modelica.SIunits.Angle theta2(
    start=0,
    stateSelect=stateSelect) "Angle of wheel 2";
  Modelica.SIunits.AngularVelocity der_theta1(
    start=0,
    stateSelect=stateSelect) "Derivative of theta 1";
  Modelica.SIunits.AngularVelocity der_theta2(
    start=0,
    stateSelect=stateSelect) "Derivative of theta 2";

  
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame1
    "Frame fixed in center point of left wheel (y-axis: along wheel axis, z-axis: upwards)"
    annotation (Placement(transformation(extent={{-96,16},{-64,-16}}),
        iconTransformation(extent={{-96,16},{-64,-16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame2
    "Frame fixed in center point of right wheel (y-axis: along wheel axis, z-axis: upwards)"
    annotation (Placement(transformation(extent={{64,16},{96,-16}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis1
    "1-dim. rotational flange that drives the left wheel"
    annotation (Placement(transformation(extent={{-110,90},{-90,110}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis2
    "1-dim. rotational flange that drives the right wheel"
    annotation (Placement(transformation(extent={{90,90},{110,110}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b support
    "Support of 1D axes" annotation (Placement(transformation(extent={{-10,70},
            {10,90}}), iconTransformation(extent={{-10,70},{10,90}})));
  PendulumRobot.RollingWheelSetTilting wheelSetJoint(animation = animation, stateSelect = stateSelect, theta1(displayUnit = "rad"), theta2(displayUnit = "rad"), wheelDistance = wheelDistance, wheelRadius = wheelRadius)  annotation(
    Placement(visible = true, transformation(origin = {0, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder wheelL(animation = animation,diameter = wheelRadius * 2, r = {0, -wheelThickness, 0})  annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Parts.BodyCylinder wheelR(animation = animation,diameter = wheelRadius * 2, r = {0, -wheelThickness, 0}) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  wheelSetJoint.theta1 = theta1;
  wheelSetJoint.theta2 = theta2;
  der_theta1 = der(theta1);
  der_theta2 = der(theta2);
  connect(frame1, wheelL.frame_a) annotation(
    Line(points = {{-80, 0}, {-60, 0}}));
  connect(wheelL.frame_b, wheelSetJoint.frame1) annotation(
    Line(points = {{-40, 0}, {-20, 0}, {-20, 40}, {-8, 40}}, color = {95, 95, 95}));
  connect(wheelSetJoint.frame2, wheelR.frame_a) annotation(
    Line(points = {{8, 40}, {20, 40}, {20, 0}, {40, 0}}, color = {95, 95, 95}));
  connect(wheelSetJoint.support, support) annotation(
    Line(points = {{0, 48}, {0, 80}}));
  connect(wheelSetJoint.axis1, axis1) annotation(
    Line(points = {{-10, 50}, {-20, 50}, {-20, 100}, {-100, 100}}));
  connect(wheelSetJoint.axis2, axis2) annotation(
    Line(points = {{10, 50}, {20, 50}, {20, 100}, {100, 100}}));
  connect(wheelR.frame_b, frame2) annotation(
    Line(points = {{60, 0}, {80, 0}}));
  connect(frameMiddle, wheelSetJoint.frameMiddle) annotation(
    Line(points = {{0, 0}, {0, 38}}));
  annotation (
    defaultComponentName="wheelSet",
    Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{100,
            100}}), graphics={
        Line(
          points={{0,76},{0,4}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Sphere, extent = {{42, 80}, {118, -80}}),
        Ellipse(lineColor = {64, 64, 64}, extent = {{42, 80}, {118, -80}}),
        Rectangle(fillColor = {175, 175, 175}, pattern = LinePattern.None, fillPattern = FillPattern.Solid, extent = {{-100, -80}, {100, -100}}),
        Text(lineColor = {0, 0, 255}, extent = {{-150, -105}, {150, -145}}, textString = "%name"),
        Line(points={{86,24},{64,24},{64,12},{56,12}}),
        Line(points={{86,-24},{64,-24},{64,-12},{56,-12}}),
        Line(
          points={{100,100},{80,100},{80,-2}}),
        Polygon(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-62, 6}, {64, 6}, {64, -6}, {6, -6}, {6, -20}, {-6, -20}, {-6, -6}, {-62, -6}, {-62, 6}}),
        Ellipse(lineColor = {64, 64, 64}, fillColor = {215, 215, 215}, fillPattern = FillPattern.Sphere, extent = {{-118, 80}, {-42, -80}}),
        Line(
          points={{-96,100},{-80,100},{-80,4}}),
        Ellipse(lineColor = {64, 64, 64}, extent = {{-118, 80}, {-42, -80}}),
        Line(
          points={{-100,-80},{100,-80}})}),
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},{
            100,100}}), graphics={Line(
            points={{0,-106},{0,-78}},
            color={0,0,255}),Polygon(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{0, -60}, {-6, -78}, {6, -78}, {0, -60}}),Text(lineColor = {0, 0, 255}, extent = {{12, -68}, {30, -80}}, textString = "x"),Line(
            points={{6,-100},{-26,-100}},
            color={0,0,255}),Polygon(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-22, -94}, {-22, -106}, {-40, -100}, {-22, -94}}),Text(lineColor = {0, 0, 255}, extent = {{-46, -80}, {-28, -92}}, textString = "y")}),
    Documentation(info="<html>
<p>
Two wheels are connected by an axis and can rotate around this axis.
The wheels are rolling on the x-y plane of the world frame.
The coordinate system attached to the center of the wheel axis (frameMiddle)
is constrained so that it is always parallel to the x-y plane.
If all generalized coordinates are zero, frameMiddle is parallel
to the world frame.
</p>

<h4>Note</h4>
<p>
To work properly, the gravity acceleration vector g of the world must point in the negative z-axis, i.e.
</p>
<blockquote><pre>
<span style=\"font-family:'Courier New',courier; color:#0000ff;\">inner</span> <span style=\"font-family:'Courier New',courier; color:#ff0000;\">Modelica.Mechanics.MultiBody.World</span> world(n={0,0,-1});
</pre></blockquote>
</html>"));
end RollingWheelSetTiltingMass;