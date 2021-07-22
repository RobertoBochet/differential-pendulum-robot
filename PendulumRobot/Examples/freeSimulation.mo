within PendulumRobot.Examples;

model freeSimulation
  extends Modelica.Icons.Example;
  parameter Modelica.SIunits.Length wheelRadius = 0.2;
  parameter Modelica.SIunits.Length wheelDistance = 0.5;
  parameter Modelica.SIunits.Mass wheelThickness = 0.05;
  parameter Modelica.SIunits.Length driveShaftDiameter = 0.1;
  parameter Modelica.SIunits.Length pendulumLength = 1;
  parameter Modelica.SIunits.Length pendulumDiameter = 0.1;
  inner Modelica.Mechanics.MultiBody.World world(n = {0, 0, -1}) annotation(
    Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.Ground ground annotation(
    Placement(visible = true, transformation(origin = {70, -70}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Mechanics.MultiBody.Visualizers.FixedFrame fixedFrame3(animation = false) annotation(
    Placement(visible = true, transformation(origin = {30, 90}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  PendulumRobot.PendulumRobotBodyReal pendulumRobotBodyReal(dampingConstant = 0.1, driveShaftDiameter = driveShaftDiameter, pendulumDiameter = pendulumDiameter, pendulumLength = pendulumLength, theta1(displayUnit = "rad"), theta2(displayUnit = "rad"), wheelDistance = wheelDistance, wheelRadius = wheelRadius, wheelThickness = wheelThickness) annotation(
    Placement(visible = true, transformation(origin = {0, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(pendulumRobotBodyReal.frameTop, fixedFrame3.frame_a) annotation(
    Line(points = {{0, 16}, {0, 90}, {20, 90}}, color = {95, 95, 95}));
  annotation(
    Diagram(coordinateSystem(preserveAspectRatio = true, extent = {{-100, -100}, {100, 100}})),
    experiment(StopTime = 10));
end freeSimulation;