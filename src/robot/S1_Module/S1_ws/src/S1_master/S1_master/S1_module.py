"""This file launches a node to control a connected Robomaster-S1."""
import rclpy
from rclpy.node import Node
from robomaster import robot as robomaster_robot
import geometry_msgs.msg

# The max movement speed of the chassis and gimball, denoted with x,y,z axis
MAX_SPEED__CHASSIS = [30.0, 30.0, 0.0]
MAX_SPEED__GIMBAL = [30.0, 30.0, 0.0]

# The min movement speed of the chassis and gimball
MIN_SPEED__CHASSIS = [-30.0, -30.0, 0.0]
MIN_SPEED__GIMBAL = [-30.0, -30.0, 0.0]


class S1_driver(Node):
    """This class is used for all communications with a robomaster S1 module.

    It subscribes to the /cmd_vel topic to obtain movement instructions.
    It also subscribes to the /blast topic to obtain when to fire.
    """

    def __init__(self, sn):
        """All function calls."""
        # """This is an initializer function."""
        # Initialize the node
        super().__init__('S1_driver')
        # Initialize the robot
        print("Connecting to robomaster....")
        self.robot = robomaster_robot.Robot()
        # Initialize in USB-mode. For router mode, change conn_type to 'sta'
        self.robot.initialize(conn_type="rndis", sn=sn)
        print("Connected!")
        # Set the gimbal mode as "gimbal lead"
        self.robot.set_robot_mode(robomaster_robot.GIMBAL_LEAD)
        # Recenter gimbal
        self.robot.gimbal.recenter()
        # Create subscriber for the /cmd_vel movement commands
        self.twist_sub = self.create_subscription(geometry_msgs.msg.Twist, '/cmd_vel', self.twistCallback, 10)
        # Create an rclpy timer instance that triggers a safety mechanism when no input are received before the timer runs out
        # self.failsafe_timer = self.create_timer(3.0, self.failsafeCallback)
        # Prevent "unused variable" error
        self.twist_sub
        # Declare parameters for this S1_instance
        self.declare_parameters(
            namespace='',
            parameters=[
                ('conn_type', 'rndis'),
                ('sn', None),
                ('linear_max__x', 30.0),
                ('linear_max__y', 30.0),
                ('angular_max__pitch', 30.0),
                ('angular_max__yaw', 30.0)
            ]
        )
        self.get_logger().info("After initialization")

    def __del__(self):
        """All function calls."""
        # """this function is an destructor thats called at the end of the program."""
        # Close the robomaster connection
        print("Ending connection with robomaster...")
        self.robot.close()
        print("Connection closed!")

    def failsafeCallback(self):
        """All function calls."""
        # """This is a callback function that triggers when the failsafe timer method runs out.
        # It fully stops the S1's movement.
        # The timer resets after each succeful command execution, but prevents the system from damaging itself
        # after an unforseen event causes the S1 to driver aimlesly.
        # """
        # Stop the chassis's movement
        # stop the gimbal's movement
        # Report the failsafe trigger
        self.get_logger().info("Failsafe triggered after no commands were received before timer runnout")

    def stopMovement__Chassis(self):
        """All function calls."""
        # """Calling this function will set the linear speed of the S1's chassis to 0, fully halting its movement."""
        self.robot.chassis.driver_speed(0.0, 0.0)

    def stopMovement__Gimbal(self):
        """All function calls."""
        # """Calling this function will set the linear speed of the S1's gimbal to 0, fully halting its movement."""
        self.robot.gimbal.drive_speed(0.0, 0.0)

    def clampValue__float(self, value, max_val, min_val):
        """All function calls."""
        # """This function clamps a float value between a min and a max."""
        # Initialize the guard clause
        if not isinstance(value, float):
            raise Exception("Parameter 'value' is not of type 'float'. Instead got: '{}'".format(type(value)))
        if not isinstance(max_val, float):
            raise Exception("Parameter 'max_val' is not of type 'float'. Instead got: '{}'".format(type(max_val)))
        if not isinstance(min_val, float):
            raise Exception("Parameter 'min_val' is not of type 'float'. Instead got: '{}'".format(type(min_val)))
        if max_val < min_val:
            raise Exception("Parameter 'max_val' cannot be larger than parameter 'min_val'")
        # Clamp the value between the max and min values and return
        return max(min(max_val, value), min_val)

    def twistCallback(self, msg):
        """All function calls."""
        # """This is a callback function that triggers when twist messages are received on the /cmd_vel topic."""
        # Initialize the guard clause
        if not isinstance(msg, geometry_msgs.msg.Twist):
            raise Exception("'msg' parameter received from callback is not of type 'Twist'. Instead got: '{}'".format(type(msg)))
        # Clamp the linear and angular speeds to specified ranges
        linear_speed = [0.0, 0.0]
        angular_speed = [0.0, 0.0]
        linear_speed[0] = self.clampValue__float(msg.linear.x, MAX_SPEED__CHASSIS[0], MIN_SPEED__CHASSIS[0])
        linear_speed[1] = self.clampValue__float(msg.linear.y, MAX_SPEED__CHASSIS[1], MIN_SPEED__CHASSIS[1])
        angular_speed[0] = self.clampValue__float(msg.angular.x, MAX_SPEED__GIMBAL[0], MIN_SPEED__GIMBAL[0])
        angular_speed[1] = self.clampValue__float(msg.angular.y, MAX_SPEED__GIMBAL[1], MIN_SPEED__GIMBAL[1])
        # Instruct the S1's gimbal to turn according to the angular commands
        self.robot.gimbal.drive_speed(angular_speed[0], angular_speed[1])
        # Instruct the S1's chassis to drive according to the linear commands


def main(args=None):
    """All function calls."""
    # """This function is the main and runs the node."""
    # Initialize rclpy
    rclpy.init(args=args)
    # Define serial number of robot
    sn = "159CGAC0050R76"
    print("Main called!")
    # Create S1 driver instance
    S1 = S1_driver(sn)
    # Spin the s1 driver instance to use subscriber
    rclpy.spin(S1)
    # Destroy the node explicitly
    S1.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
