Single-Track Model of a Car

This code describes the single-track model of a car and simulates its trajectory to create a phone-like shape.

Overview

The goal is to simulate the car's movement based on given parameters, including the wheelbase, wheel speeds, and wheel angles. The parameters used are:

Wheelbase (r_b)

Speeds of the four wheels (vFL, vFR, vRL, vRR)

Initial wheel angles:

beta_R = 0.0 (Initial rear wheel angle - no steering)

beta_F = 1.0 (Initial front wheel angle - for turning)

The direction of the car depends on the value of beta_F. When beta_F is 0, the car drives straight. When beta_F is non-zero, the car makes a turn.

Creating a Phone Shape
To create a phone-shaped trajectory, the simulation consists of two turns and two straight segments:

Turning: The car turns left for a fixed duration and angle.
Straight Line: The car drives straight by setting beta_F to zero for a specified duration.
Additional Subgraphs
In addition to the trajectory, the implementation includes four subgraphs that display the following:

Omega (ω): Angular speed
r_RM: Distance between the reference point and the midpoint of the car
Theta (θ): Heading angle
Wheel Angles: beta_F (front wheel angle) and beta_R (rear wheel angle)
These subgraphs provide insights into the car's dynamics throughout the simulation.
