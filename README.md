This code describe the single-track model of a car.

What here is trying to achieve is to create a phone shape of a car trajectory.

You have the parameters wheelbase and the speed of the 4 wheels each.

And also the wheel angle in front and in the back:

beta_R = 0.0  # Initial rear wheel angle (no steering)

beta_F = 1.0  # Initial front wheel angle for turning

The direction of the car is depended on the value of beta_F. If beta_F is 0, the car will drive straight, if not, it will make a turn.
For creating a phone shape, we need two turns and two times straight line.

For turning point, we fixed a duration and the angle for turning left in this example.
For driving straight, set beta_F zero for specific time.

In this implementation, there is also 4 other subgraphs, that shows us omega (angle speed), r_RM (this is the distance between point and the middle point), theta (heading) and beta_F and beta_R


