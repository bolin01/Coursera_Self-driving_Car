# Course4FinalProject
Motion Planning for Self-driving Cars (Course 4) Final Project 

**Goal**: have a functional motion planning stack that can avoid both static and dynamic obstacles while tracking the center line of a lane, while also handling stop signs. In the project, **behavior planning logic**, **path generation**, **static collision checking**, **path selection**, **velocity profile generation** are implemented.   

</br> <!--blank line-->

## Behavior Planning Logic (_behavioural_planner.py_)
Implemented the behvioral logic required to handle a stop sign. A state machine that transitions between lane following, deceleration to the stop sign, staying stopped, and back to lane following, when a stop sign is encountered. 

_**Longitudinal controller (PID)**_ and _**lateral controller (Stanley Model)**_ for a car to follow given waypoints and velocity profile in **CARLA** simulator (_Python_)  
