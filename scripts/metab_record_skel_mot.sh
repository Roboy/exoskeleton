#!/usr/bin/env bash

source $HBP/GazeboRosPackages/devel/setup.bash

act="1.0"

rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [${act}, 0.0, 0.0, 0.0, 0.0, 0.0]"
sleep 20
rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [0.0, 0.0, 0.0, ${act}, ${act}, 0.0]"
sleep 20
rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [${act}, 0.0, 0.0, 0.0, 0.0, 0.0]"
sleep 20
rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [0.0, 0.0, 0.0, ${act}, ${act}, 0.0]"
sleep 20

rosservice call /gazebo_muscle_interface/arm26/set_activations "activations: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]"
echo "finished"
