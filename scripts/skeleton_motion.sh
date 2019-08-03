#!/usr/bin/env bash

source $HBP/GazeboRosPackages/devel/setup.bash

while true; do
    rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]"
    sleep 20
    rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [0.0, 0.0, 0.0, 0.1, 0.1, 0.0]"
    sleep 20
done
