# Self-Driving Car Path Planner

This repo contains a path-planning implementation, which can drive a simulated car around a track, changing lanes where necessary to maintain speed and avoid collisions.

The path-planner has been successfully tested with 10 miles of driving without incident.

![Screenshot](/figures/screenshot.png)

## Model documentation

The model works by using a finite state machine in [pathplanner.cpp](src/pathplanner.cpp) to choose between different possible actions at each update step.  The possible actions are:
* Accelerate;
* Decelerate;
* Decelerate hard (useful when another car pulls in front);
* Change lane to the right; and
* Change lane to the left.

Each time data is received from the simulation, the code calls the [update](src/pathplanner.cpp#L24) method to compute the next action.  This does the following:
* [Predicts](src/pathplanner.cpp#L299) the future positions of the other vehicles on the road during a 1 second time horizon;
* Checks which actions are [valid](src/pathplanner.cpp#L127), e.g. left-change is not valid if the car is already in the leftmost lane;
* For each possible action, computes a [trajectory](src/pathplanner.cpp#L217) for the car to follow using the [spline.h](src/spline.h) library to fit a smooth path between the car's current position and its future target location;
* Computes a [cost](src/pathplanner.cpp#L152) for each possible trajectory; and
* Chooses the path with lowest cost and sends this back to the simulator.

Each trajectory aims to get the car to its future target position within the 2 second window defined [here](src/pathplanner.h#L23), guaranteeing that lane changes can be executed quickly.

The performance of the model is heavily sensitive to the cost function, which includes the following components:
* Penalty terms on [low](src/pathplanner.cpp#L170) or [high](src/pathplanner.cpp#L172) speed;
* A [term](src/pathplanner.cpp#L177) that encourages deceleration when the distance to the car in front is too short;
* Penalties for [left](src/pathplanner.cpp#L182) and [right](src/pathplanner.cpp#L186) lane changes when there is another vehicle in the way;
* A check for the nearest vehicle in front at the [end of the manoeuvre](src/pathplanner.cpp#L195), to encourage the car to change lanes if it is behind a slow-moving car in the current lane;
* A penalty for [lane changes](src/pathplanner.cpp#L204) to discourage unnecessary manoeuvres; and
* A check on the [shortest distance](src/pathplanner.cpp#L211) to another vehicle during the manoeuvre, with a penalty on actions that could cause a collision.

A limitation of this method is that it can only 'see' one action into the future.  An improved system would be able to make more complicated decisions, e.g. realising that two lane changes might be required to find a clear path ahead.
