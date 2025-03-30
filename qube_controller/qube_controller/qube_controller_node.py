import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import numpy as np
from time import time


class PIDController:
    """Simple PID controller implementation"""

    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        # Initialize state variables
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time()

    def compute(self, current_value):
        """Compute the control signal based on the current value"""
        # Calculate time delta
        current_time = time()
        dt = current_time - self.prev_time
        dt = max(dt, 0.001)  # Avoid division by zero

        # Calculate error
        error = self.setpoint - current_value

        # Proportional term
        p_term = self.kp * error

        # Integral term
        self.integral += error * dt
        i_term = self.ki * self.integral

        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative

        # Sum all terms to get output
        output = p_term + i_term + d_term

        # Update state for next iteration
        self.prev_error = error
        self.prev_time = current_time

        return output

    def set_target(self, setpoint):
        """Update the setpoint/target value"""
        self.setpoint = setpoint

    def reset(self):
        """Reset the controller state"""
        self.prev_error = 0.0
        self.integral = 0.0
        self.prev_time = time()


class QubeControllerNode(Node):
    """ROS2 Node for controlling the Qube using PID control"""

    def __init__(self):
        super().__init__('qube_pid_controller')

        # Declare PID parameters
        self.declare_parameter('kp', 1.0)
        self.declare_parameter('ki', 0.1)
        self.declare_parameter('kd', 0.05)
        self.declare_parameter('setpoint', 0.0)

        # Create PID controller with initial parameter values
        self.position_pid = PIDController(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            setpoint=self.get_parameter('setpoint').value
        )

        # Create publishers and subscribers
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/command',
            10)

        # Create a timer for parameter updates
        self.timer = self.create_timer(1.0, self.update_parameters)

        self.get_logger().info('Qube PID Controller started')

    def joint_state_callback(self, msg):
        """Process joint state messages and compute control commands"""
        # Extract position and velocity from the message
        # Assuming the first joint is the one we're controlling
        if not msg.position:
            self.get_logger().warn('Received empty joint state message')
            return

        position = msg.position[0]
        velocity = msg.velocity[0] if msg.velocity else 0.0

        # Log current state
        self.get_logger().debug(f'Position: {position}, Velocity: {velocity}')

        # Compute control output
        control_signal = self.position_pid.compute(position)

        # Prepare and publish command
        self.publish_command(control_signal)

    def publish_command(self, control_signal):
        """Publish command to the velocity controller"""
        # Create and populate Float64MultiArray message
        cmd_msg = Float64MultiArray()

        # For Float64MultiArray, we need to set the data field as a list
        cmd_msg.data = [float(control_signal)]

        # Publish the message
        self.velocity_pub.publish(cmd_msg)
        self.get_logger().debug(f'Published command: {control_signal}')

    def update_parameters(self):
        """Update parameters from ROS2 parameter server"""
        # Check if setpoint has changed
        new_setpoint = self.get_parameter('setpoint').value
        self.position_pid.set_target(new_setpoint)

        # Update PID gains
        self.position_pid.kp = self.get_parameter('kp').value
        self.position_pid.ki = self.get_parameter('ki').value
        self.position_pid.kd = self.get_parameter('kd').value


def main(args=None):
    rclpy.init(args=args)

    node = QubeControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
