#!/usr/bin/env bash

source $HBP/GazeboRosPackages/devel/setup.bash

rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [0.1, 0.0, 0.0, 0.2, 0.3, 0.0]"
