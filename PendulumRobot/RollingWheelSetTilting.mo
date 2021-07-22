within PendulumRobot;

model RollingWheelSetTilting
"Joint (no mass, no inertia) that describes an ideal rolling wheel set (two ideal rolling wheels connected together by an axis)"
 Modelica.Mechanics.MultiBody.Interfaces.Frame_a frameMiddle
  "Frame fixed in middle of axis connecting both wheels (y-axis: along wheel axis, z-Axis: upwards)"
    annotation (Placement(transformation(extent={{-16,16},{16,-16}}),
        iconTransformation(extent={{-16,-16},{16,16}},
      rotation=90,
      origin={0,-20})));

  parameter Boolean animation=true
  "= true, if animation of wheel set shall be enabled";

  parameter Modelica.SIunits.Radius wheelRadius "Radius of one wheel";
  parameter Modelica.SIunits.Distance wheelDistance "Distance between the two wheels";

  parameter StateSelect stateSelect = StateSelect.default
  "Priority to use the generalized coordinates as states";
  
  Modelica.SIunits.Angle theta1(start=0, stateSelect=stateSelect)
  "Angle of wheel 1";
  Modelica.SIunits.Angle theta2(start=0, stateSelect=stateSelect)
  "Angle of wheel 2";
  Modelica.SIunits.AngularVelocity der_theta1(start=0, stateSelect=stateSelect)
  "Derivative of theta 1";
  Modelica.SIunits.AngularVelocity der_theta2(start=0, stateSelect=stateSelect)
  "Derivative of theta 2";

  Modelica.Mechanics.MultiBody.Interfaces.Frame_a frame1
  "Frame fixed in center point of left wheel (y-axis: along wheel axis, z-Axis: upwards)"
    annotation (Placement(transformation(extent={{-96,16},{-64,-16}}),
        iconTransformation(extent={{-96,16},{-64,-16}})));
  Modelica.Mechanics.MultiBody.Interfaces.Frame_b frame2
  "Frame fixed in center point of right wheel (y-axis: along wheel axis, z-Axis: upwards)"
    annotation (Placement(transformation(extent={{64,16},{96,-16}})));
  Modelica.Mechanics.MultiBody.Parts.Fixed fixed(                  animation= false,r={-10,0,
        wheelRadius})
                      annotation (Placement(visible = true, transformation(origin = {0, -90}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rod1(                 r={
        0,wheelDistance/2,0}, animation=animation)
    annotation (Placement(transformation(extent={{-10,-10},{-30,10}})));
  Modelica.Mechanics.MultiBody.Parts.FixedTranslation rod2(                 r={
        0,-wheelDistance/2,0}, animation=animation)
    annotation (Placement(visible = true, transformation(extent = {{10, -10}, {30, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute1(
    n={0,1,0},
    useAxisFlange=true,
    animation=animation)
    annotation (Placement(transformation(extent={{-40,-10},{-60,10}})));
  Modelica.Mechanics.MultiBody.Joints.Revolute revolute2(
    n={0,1,0},
    useAxisFlange=true,
    animation=animation)
    annotation (Placement(transformation(extent={{40,-10},{60,10}})));
  Modelica.Mechanics.MultiBody.Joints.Internal.RollingConstraintVerticalWheel
  rolling1(                             radius=wheelRadius)
    annotation (Placement(transformation(extent={{-80,-60},{-60,-40}})));
  Modelica.Mechanics.MultiBody.Joints.Internal.RollingConstraintVerticalWheel
  rolling2(                             radius=wheelRadius,
      lateralSlidingConstraint=false)
    annotation (Placement(transformation(extent={{60,-60},{80,-40}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis1
  "1-dim. rotational flange that drives the joint"
    annotation (Placement(transformation(extent={{-110,90},{-90,110}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_a axis2
  "1-dim. rotational flange that drives the joint"
    annotation (Placement(transformation(extent={{90,90},{110,110}})));
  Modelica.Mechanics.MultiBody.Parts.Mounting1D mounting1D
    annotation (Placement(transformation(extent={{-10,38},{10,58}})));
  Modelica.Mechanics.Rotational.Interfaces.Flange_b support
  "Support of 1D axes" annotation (Placement(transformation(extent={{-10,70},
          {10,90}}),       iconTransformation(extent={{-10,70},{10,90}})));
 Modelica.Mechanics.MultiBody.Joints.Prismatic prismatic(animation = false, s(start = 10))  annotation(
    Placement(visible = true, transformation(origin = {-22, -44}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
 Modelica.Mechanics.MultiBody.Joints.Universal universal(animation = false,n_a = {0, 0, 1}, n_b = {0, 1, 0}, phi_a(displayUnit = "deg", start = 0), phi_b(displayUnit = "deg", start = 0.1745329251994329)) annotation(
    Placement(visible = true, transformation(origin = {10, -26}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
 Modelica.Mechanics.MultiBody.Joints.Revolute revolute3(animation = false, phi(displayUnit = "deg")) annotation(
    Placement(visible = true, transformation(origin = {0, -62}, extent = {{-10, -10}, {10, 10}}, rotation = 90)));
equation
  revolute1.phi = theta1;
  revolute2.phi = theta2;
  der_theta1 = der(theta1);
  der_theta2 = der(theta2);
  connect(rod1.frame_a, frameMiddle) annotation(
    Line(points = {{-10, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
 connect(rod2.frame_a, frameMiddle) annotation(
    Line(points = {{10, 0}, {0, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rod1.frame_b, revolute1.frame_a) annotation(
    Line(points = {{-30, 0}, {-40, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute1.frame_b, frame1) annotation(
    Line(points = {{-60, 0}, {-80, 0}}, color = {95, 95, 95}, thickness = 0.5));
 connect(revolute2.frame_a, rod2.frame_b) annotation(
    Line(points = {{40, 0}, {30, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute2.frame_b, frame2) annotation(
    Line(points = {{60, 0}, {80, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rolling1.frame_a, revolute1.frame_b) annotation(
    Line(points = {{-70, -48}, {-70, 0}, {-60, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(rolling2.frame_a, revolute2.frame_b) annotation(
    Line(points = {{70, -48}, {70, 0}, {60, 0}}, color = {95, 95, 95}, thickness = 0.5));
  connect(revolute1.axis, axis1) annotation(
    Line(points = {{-50, 10}, {-50, 100}, {-100, 100}}));
  connect(revolute2.axis, axis2) annotation(
    Line(points = {{50, 10}, {50, 100}, {100, 100}}));
  connect(frameMiddle, mounting1D.frame_a) annotation(
    Line(points = {{0, 0}, {0, 38}}, color = {95, 95, 95}, thickness = 0.5));
  connect(mounting1D.flange_b, support) annotation(
    Line(points = {{10, 48}, {16, 48}, {16, 80}, {0, 80}}));
 connect(fixed.frame_b, revolute3.frame_a) annotation(
    Line(points = {{0, -80}, {0, -72}}, color = {95, 95, 95}));
 connect(revolute3.frame_b, prismatic.frame_a) annotation(
    Line(points = {{0, -52}, {0, -44}, {-12, -44}}, color = {95, 95, 95}));
 connect(prismatic.frame_b, universal.frame_a) annotation(
    Line(points = {{-32, -44}, {-40, -44}, {-40, -26}, {0, -26}}, color = {95, 95, 95}));
 connect(frameMiddle, universal.frame_b) annotation(
    Line(points = {{0, 0}, {0, -14}, {30, -14}, {30, -26}, {20, -26}}));
  annotation (defaultComponentName="wheelSet",Icon(coordinateSystem(preserveAspectRatio=true, extent={{-100,
            -100},{100,100}}), graphics={Rectangle(fillColor = {175, 175, 175}, fillPattern = FillPattern.Solid, extent = {{-100, -80}, {100, -100}}),
        Text(lineColor = {0, 0, 255}, extent = {{-146, -98}, {154, -138}}, textString = "%name"),
        Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{42, 80}, {118, -80}}),
        Line(points={{86,24},{64,24},{64,12},{56,12}}),
        Line(points={{86,-24},{64,-24},{64,-12},{56,-12}}),
        Line(
          points={{100,100},{80,100},{80,-2}}),
        Line(
          points={{0,76},{0,4}}),
      Polygon(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-62, 6}, {64, 6}, {64, -6}, {6, -6}, {6, -20}, {-6, -20}, {-6, -6}, {-62, -6}, {-62, 6}}),
        Ellipse(fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, extent = {{-118, 80}, {-42, -80}}),
        Line(
          points={{-96,100},{-80,100},{-80,4}})}),
    Diagram(coordinateSystem(preserveAspectRatio=true, extent={{-100,-100},
            {100,100}}), graphics={
        Line(
          points={{-68,24},{-68,52}},
          color={0,0,255}),
        Polygon(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-68, 70}, {-74, 52}, {-62, 52}, {-68, 70}}),
        Text(lineColor = {0, 0, 255}, extent = {{-68, 70}, {-50, 58}}, textString = "x"),
        Line(
          points={{-62,30},{-94,30}},
          color={0,0,255}),
        Polygon(lineColor = {0, 0, 255}, fillColor = {255, 255, 255}, fillPattern = FillPattern.Solid, points = {{-90, 36}, {-90, 24}, {-108, 30}, {-90, 36}}),
        Text(lineColor = {0, 0, 255}, extent = {{-114, 50}, {-96, 38}}, textString = "y")}),
  Documentation(info="<html>
<p>
An assembly joint for a wheelset rolling on the x-y plane of the world frame.
The frames frame1 and frame2 are connected to rotating wheels; the frameMiddle moves
in a plane parallel to the x-y plane of the world and should be connected to the vehicle body.
</p>

<h4>Note</h4>
<p>
To work properly, the gravity acceleration vector g of the world must point in the negative z-axis, i.e.
</p>
<blockquote><pre>
<span style=\"font-family:'Courier New',courier; color:#0000ff;\">inner</span> <span style=\"font-family:'Courier New',courier; color:#ff0000;\">Modelica.Mechanics.MultiBody.World</span> world(n={0,0,-1});
</pre></blockquote>
</html>"));
end RollingWheelSetTilting;