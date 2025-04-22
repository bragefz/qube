#### Docs
This repo contains packages for visualizing and controlling a quanser qube. 

## qube_bringup
This package starts the the pid controller as well as a digital twin of the qube. 
If a real qube is not awailable, setting the launch parameter simulation:=true will allow control of the simulated qube alone. 
Also launches an rqt node that enables adjusting setpoitn at runtime with a slider or inserting values.

To launch use this command in the terminal:
ros2 launch qube_bringup bringup.launch.py

The following parameters can set when launching:
-simulation (true/false) default:=false
-device (string, your listed device for qube) default:=/dev/ttyACM0
-baud_rate (int) default:=115200
-kp (float, include decimal) default:=8.0
-ki (float) default:=0.2
-kd (float) default:=0.7
-setpoint (float) default:=0.0

## qube_description



## qube_controller
This package implements a pid controller which sets the control signal based on the positional error.
It includes a node which subscribes to the angle at /joint_states and publishes a desired velocity at /velocity_controller/commands.
