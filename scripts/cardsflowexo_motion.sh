#!/usr/bin/env bash

source $HBP/GazeboRosPackages/devel/setup.bash

while true; do
    #unflex
    rosservice call /gazebo_muscle_interface/CARDSFlowExo/set_activations "activations: [0.3, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
    sleep 1.5
    # flex
    rosservice call /gazebo_muscle_interface/CARDSFlowExo/set_activations "activations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5, 0.5, 0.0, 0.0, 0.1, 0.1]"
    sleep 1.5
done
