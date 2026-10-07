# pid-steering-simulation
A Python simulation of vehicle movement using a PID controller and a bicycle model.

## Description

The main goal is to control the steering of the vehicle so that it moves from its current position to a target point using a PID controller.
The simulation also includes acceleration, braking, maximum velocity, steering angle limits, and steering rate limits.

## How It Works
At each step, the model calculates the required steering angle based on the current position and heading relative to the target point.
The calculated control output is then subject to the specified constraints:
the maximum steering angle and the maximum steering rate. As a result, the model turns the steering by the angle allowed at that moment and updates its position and heading.

## PID Controller

The PID controller calculates the steering control based on three components:
Proportional (P) — responds to the current error. The larger the error, the stronger the control output.
Integral (I) — accumulates the error over time.
It can be understood as the accumulated area under the error graph and represents how long and how strongly the error has persisted.
Derivative (D) — responds to the rate of change of the error.
It shows how quickly the error is increasing or decreasing and helps reduce overshoot and prevent the system from moving too far in the opposite direction.

## Vehicle Model

The model keeps track of the vehicle's position, heading, velocity, and steering angle.
At each step, the PID controller calculates the desired steering angle based on the current state and the target position.
The desired steering angle is then limited by the maximum steering angle and steering rate to determine the actual steering angle.
Finally, the vehicle's position and heading are updated based on its velocity and steering angle.

## Parameters

PID controller parameters:
- `Kp` — proportional coefficient.
- `Ki` — integral coefficient.
- `Kd` — derivative coefficient.
- `max_steering_angle` — maximum steering angle.
- `max_velocity` — maximum vehicle velocity.

Initial state and target:
- `target_x`, `target_y` — target position.
- `current_x`, `current_y` — initial vehicle position.
- `heading` — initial vehicle heading.
- `velocity` — initial vehicle velocity.
  
## Example
```
The following example uses the default values:
Kp = 0.8
Ki = 0
Kd = 0.05
max_steering_angle = 30
max_velocity = 15
target_x = 20
target_y = 20
current_x = 0
current_y = 0
heading = 0
velocity = 2
```

Example output:
```
The object reached the target in 49 steps.
{'x': 0.2099872821113082, 'y': 0.0023111364100524986, 'step': 1, 'heading': 0.6305763589799966, 'steering_angle': 3.0, 'error': 45.0, 'velocity': 2.2}
{'x': 0.43984497223691377, 'y': 0.010400777324276841, 'step': 2, 'heading': 2.0156430496048627, 'steering_angle': 6.0, 'error': 44.668482348193244, 'velocity': 2.4000000000000004}
...
{'x': 19.982162289020003, 'y': 19.980412358265937, 'step': 48, 'heading': 47.67711024020043, 'steering_angle': -2.7553802963309896e-06, 'error': -2.9806832246985e-06, 'velocity': 3.6000000000000028}
{'x': 20.0, 'y': 20.0, 'step': 49, 'heading': 47.67711024020043, 'steering_angle': -2.7553802963309896e-06, 'error': -2.9806832246985e-06, 'velocity': 3.0000000000000027}
```

## Testing

Examples of tested target positions:
(20, 20) — diagonal movement.
(0, 20) — 90-degree turn.
(-20, 0) — 180-degree turn.
(20, 0) — straight movement.
The tests also included different initial velocities and PID parameters to check acceleration, braking, steering, and target reaching behavior.

## Possible Improvements

Add steering acceleration to make the steering dynamics more realistic.
Extend the model to represent a more detailed vehicle model.
Improve the target-reaching logic so that the vehicle approaches the target with a lower velocity instead of stopping the movement at the final step.
