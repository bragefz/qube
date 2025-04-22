# Quanser Qube
This repo contains packages for visualizing and controlling a Quanser Qube.

## Packages

### qube_bringup
This package starts the PID controller as well as a digital twin of the qube.  
If a real qube is not available, setting the launch parameter `simulation:=true` will allow control of the simulated qube alone.  
Also launches an rqt node that enables adjusting setpoint at runtime with a slider or inserting values.  
Select 'qube_controller_node' in the rqt node to display the slider etc. 

To launch use this command in the terminal:
```
ros2 launch qube_bringup bringup.launch.py
```

The following parameters can set when launching:
- `simulation` (true/false) default:=false
- `device` (string, your listed device for qube) default:=/dev/ttyACM0
- `baud_rate` (int) default:=115200
- `kp` (float, include decimal) default:=8.0
- `ki` (float) default:=0.2
- `kd` (float) default:=0.7
- `setpoint` (float) default:=0.0

### qube_description

qube_description contains the model description for the Quanser Qube through the URDF model. The URDF model has been made by 2 files, one qube.macro.xacro file and a qube.urdf.xacro file. The qube.macro.xacro file contains details about the robot design with all the components, joints and materials used to make it. The qube.urdf.xacro file imports the macro and connects the robot to the world frame. To launch this and see the robot model in the RViz window use this command in the terminal:

```

ros2 launch qube_description view_qube.launch.py

```

After running this command the RViz window will open with the robot model displayed. A joint state publisher GUI window will also appear and the sliders can be used to rotate the disc. When you want to stop the visualization you can press ctrl + c in the terminal you used to run it.

### qube_controller
This package implements a PID controller which sets the control signal based on the positional error.
It includes a node which subscribes to the angle at `/joint_states` and publishes a desired velocity at `/velocity_controller/commands`.
