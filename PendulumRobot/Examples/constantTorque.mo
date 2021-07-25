within PendulumRobot.Examples;

model constantTorque
  extends Modelica.Icons.Example;
  parameter Modelica.SIunits.Radius wheelRadius = 0.2;
  parameter Modelica.SIunits.Distance wheelDistance = 0.5;
  parameter Modelica.SIunits.Thickness wheelThickness = 0.05;
  parameter Modelica.SIunits.Diameter driveShaftDiameter = 0.2;
  parameter Modelica.SIunits.Length pendulumLength = 1;
  parameter Modelica.SIunits.Diameter pendulumDiameter = 0.1;
  
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.Ground ground annotation(
    Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedFrame3(animation = false) annotation(
    Placement(visible = true, transformation(origin = {30, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PendulumRobot.PendulumRobotBodyReal pendulumRobotBodyReal( driveShaftDiameter = driveShaftDiameter, pendulumDiameter = pendulumDiameter, pendulumLength = pendulumLength, theta1(displayUnit = "rad"), theta2(displayUnit = "rad"), wheelDistance = wheelDistance, wheelRadius = wheelRadius, wheelThickness = wheelThickness) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedArrow fixedArrow(length = wheelRadius)  annotation(
    Placement(visible = true, transformation(origin = {-50, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedArrow fixedArrow1(length = wheelRadius) annotation(
    Placement(visible = true, transformation(origin = {50, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Joints.FreeMotionScalarInit freeMotionScalarInit( angle_2(fixed = true, start = 0.1), use_angle = true)  annotation(
    Placement(visible = true, transformation(origin = {-30, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorqueL(tau_constant = 5, useSupport = true)  annotation(
    Placement(visible = true, transformation(origin = {-30, 50}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Mechanics.Rotational.Sources.ConstantTorque constantTorqueR(tau_constant = 2, useSupport = true)  annotation(
    Placement(visible = true, transformation(origin = {30, 50}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(pendulumRobotBodyReal.frameTop, fixedFrame3.frame_a) annotation(
    Line(points = {{0, 16}, {0, 90}, {20, 90}}, color = {95, 95, 95}));
  connect(fixedArrow.frame_a, pendulumRobotBodyReal.frame1) annotation(
    Line(points = {{-40, 0}, {-8, 0}}, color = {95, 95, 95}));
  connect(fixedArrow1.frame_a, pendulumRobotBodyReal.frame2) annotation(
    Line(points = {{40, 0}, {8, 0}}));
  connect(world.frame_b, freeMotionScalarInit.frame_a) annotation(
    Line(points = {{-60, -70}, {-40, -70}}, color = {95, 95, 95}));
  connect(freeMotionScalarInit.frame_b, pendulumRobotBodyReal.frameMiddle) annotation(
    Line(points = {{-20, -70}, {0, -70}, {0, -2}}));
  connect(constantTorqueL.support, pendulumRobotBodyReal.support) annotation(
    Line(points = {{-30, 40}, {0, 40}, {0, 8}}));
  connect(constantTorqueR.support, pendulumRobotBodyReal.support) annotation(
    Line(points = {{30, 40}, {0, 40}, {0, 8}}));
  connect(constantTorqueL.flange, pendulumRobotBodyReal.axis1) annotation(
    Line(points = {{-40, 50}, {-40, 10}, {-10, 10}}));
  connect(constantTorqueR.flange, pendulumRobotBodyReal.axis2) annotation(
    Line(points = {{40, 50}, {40, 10}, {10, 10}}));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})),
    experiment(StopTime = 10));
end constantTorque;