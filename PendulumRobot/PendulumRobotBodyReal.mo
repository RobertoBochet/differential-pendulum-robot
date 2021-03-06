within PendulumRobot;

model PendulumRobotBodyReal
  "Ideal rolling wheel set consisting of two ideal rolling wheels connected together by an axis"
  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frameMiddle
    "Frame fixed in middle of axis connecting both wheels (y-axis: along wheel axis, z-axis: upwards)"
    annotation (Placement(visible = true,transformation(extent = {{-16, -32}, {16, -64}}, rotation = 0),
        iconTransformation(
        origin={0,-20},extent={{-16,-16},{16,16}},
        rotation=90)));

  parameter Boolean animation=true
    "= true, if animation of wheel set shall be enabled";

  parameter Modelica.SIunits.Radius wheelRadius "Radius of one wheel";
  parameter Modelica.SIunits.Thickness wheelThickness=0.01 "Thickness of one wheel";
  parameter Modelica.SIunits.Distance wheelDistance "Distance between the two wheels";
  parameter Modelica.SIunits.Diameter driveShaftDiameter = 0.1 "Drive shaft diameter";
  parameter Modelica.SIunits.Diameter pendulumDiameter = 0.2 "Pendulum diameter";
  parameter Modelica.SIunits.Length pendulumLength = 2 "Pendulum length";
  parameter Modelica.SIunits.RotationalDampingConstant dampingConstant = 0.3 "Rotarional joints damping constant";
  
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
    "Support of 1D axes" annotation (Placement(visible = true,transformation(extent = {{-10, 30}, {10, 50}}, rotation = 0), iconTransformation(extent = {{-10, 70}, {10, 90}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frameTop annotation(
    Placement(visible = true, transformation(origin = {0, 98}, extent = {{-16, -16}, {16, 16}}, rotation = -90), iconTransformation(origin = {0, 164}, extent = {{-16, -16}, {16, 16}}, rotation = -90)));
  PendulumRobot.PendulumRobotBody pendulumRobotBody(animation = animation,driveShaftDiameter = driveShaftDiameter, pendulumDiameter = pendulumDiameter, pendulumLength = pendulumLength, stateSelect = stateSelect, wheelDistance = wheelDistance, wheelRadius = wheelRadius, wheelThickness = wheelThickness) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damperL(d = dampingConstant, phi_rel(displayUnit = "rad"))  annotation(
    Placement(visible = true, transformation(origin = {-30, 40}, extent = {{10, 10}, {-10, -10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Components.Damper damperR(d = dampingConstant, phi_rel(displayUnit = "rad"))  annotation(
    Placement(visible = true, transformation(origin = {40, 40}, extent = {{-10, 10}, {10, -10}}, rotation = 0)));
equation
  pendulumRobotBody.theta1 = theta1;
  pendulumRobotBody.theta2 = theta2;
  der_theta1 = der(theta1);
  der_theta2 = der(theta2);
  connect(pendulumRobotBody.frameTop, frameTop) annotation(
    Line(points = {{0, 16}, {20, 16}, {20, 98}, {0, 98}}, color = {95, 95, 95}));
  connect(pendulumRobotBody.frameMiddle, frameMiddle) annotation(
    Line(points = {{0, -2}, {0, -48}}, color = {95, 95, 95}));
  connect(pendulumRobotBody.support, support) annotation(
    Line(points = {{0, 8}, {-4, 8}, {-4, 40}, {0, 40}}));
  connect(pendulumRobotBody.axis1, axis1) annotation(
    Line(points = {{-10, 10}, {-60, 10}, {-60, 100}, {-100, 100}}));
  connect(pendulumRobotBody.axis2, axis2) annotation(
    Line(points = {{10, 10}, {60, 10}, {60, 100}, {100, 100}}));
  connect(pendulumRobotBody.frame2, frame2) annotation(
    Line(points = {{8, 0}, {80, 0}}));
  connect(frame1, pendulumRobotBody.frame1) annotation(
    Line(points = {{-80, 0}, {-8, 0}}));
  connect(damperL.flange_b, pendulumRobotBody.axis1) annotation(
    Line(points = {{-40, 40}, {-40, 10}, {-10, 10}}));
  connect(damperL.flange_a, support) annotation(
    Line(points = {{-20, 40}, {0, 40}}));
  connect(support, damperR.flange_a) annotation(
    Line(points = {{0, 40}, {30, 40}}));
  connect(damperR.flange_b, pendulumRobotBody.axis2) annotation(
    Line(points = {{50, 40}, {50, 10}, {10, 10}}));
  annotation (
    defaultComponentName="pendulumRobotBodyReal",
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
        Rectangle(origin = {10, 736},fillColor = {215, 215, 215}, fillPattern = FillPattern.Sphere, extent = {{-20, -570}, {0, -730}}),
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
end PendulumRobotBodyReal;