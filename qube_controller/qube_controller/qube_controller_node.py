import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import Float64MultiArray
from time import time
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange


class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint 

        self.prev_error = 0.0
        self.prev_measurement = 0.0
        self.integral = 0.0
        self.prev_time = time()

    def calculate_command(self, current_position):
        # Calculate command using pid with derivative on measurement
        current_time = time()
        dt = current_time - self.prev_time
        dt = max(dt, 0.001)  # Avoid division by zero

        error = self.setpoint - current_position

        p_term = self.kp * error

        self.integral += error * dt
        i_term = self.ki * self.integral

        # derivative = (error - self.prev_error) / dt # Derivative on error, less smooth
        # d_term = self.kd * derivative
        derivative = -(current_position - self.prev_measurement) / dt # Derivative on measurement to avoid derivative kick
        d_term = self.kd * derivative

        output = p_term + i_term + d_term

        self.prev_error = error
        self.prev_measurement = current_position
        self.prev_time = current_time

        return output

    def set_target(self, setpoint):
        self.setpoint = -setpoint # With inversion positive increase is correctly counterclockwise spinning
        

class QubeControllerNode(Node):
    def __init__(self):
        super().__init__('qube_controller_node')
        # Declare PID parameters
        self.declare_parameter('kp', 8.0)
        self.declare_parameter('ki', 0.2)
        self.declare_parameter('kd', 0.7)
        self.declare_parameter('setpoint',
                               0.0,
                                   ParameterDescriptor(floating_point_range=[
                                   FloatingPointRange(from_value=-3.14, to_value=3.14) # Set min and max angle
                               ])
        )
        # Callback to update parameters including setpoint
        self.add_on_set_parameters_callback(self.parameter_callback)

        # Create PID controller with initial parameter values
        self.pid = PIDController(
            kp=self.get_parameter('kp').value,
            ki=self.get_parameter('ki').value,
            kd=self.get_parameter('kd').value,
            setpoint=self.get_parameter('setpoint').value
        )

        # Create subscriber to read angle
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # Create publisher to write velocity command
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/velocity_controller/commands',
            10)

    def publish_command(self, control_signal):
        # Create msg
        cmd_msg = Float64MultiArray()
        cmd_msg.data = [float(control_signal)]

        # Publish message
        self.velocity_pub.publish(cmd_msg)

    def joint_state_callback(self, msg):
        position = msg.position[0]
        velocity = msg.velocity[0]

        # Compute control output
        control_signal = self.pid.calculate_command(position)

        # Prepare and publish command
        self.publish_command(control_signal)

    def parameter_callback(self, params):
        #cUpdate pid values and setpoint
        for param in params:
            if param.name == 'setpoint':
                self.pid.set_target(param.value)
            elif param.name == 'kp':
                self.pid.kp = param.value
            elif param.name == 'ki':
                self.pid.ki = param.value
            elif param.name == 'kd':
                self.pid.kd = param.value

        return SetParametersResult(successful=True)


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
