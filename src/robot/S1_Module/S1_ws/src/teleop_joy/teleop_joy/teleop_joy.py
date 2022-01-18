import rclpy
import geometry_msgs.msg 

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header


#A variable determining wether verbose prints should be used
VERBOSE_PRINT = False


class TeleopJoy(Node):
    """This class inherits from the ROS2 node class and converts messages
       from the /joy topic to twist messages on the /cmd_vel topic
    """

    def __init__(self):
        super().__init__("teleop_joy")


        #Declare node parameters for linear and angular speeds
        #These parameters should only be changed using a launch file
        self.declare_parameters(
            namespace='',
            parameters=[
                ('linear_speed__x', 40.0),
                ('linear_speed__y', 40.0),
                ('angular_speed__pitch', 40.0),
                ('angular_speed__yaw', 40.0)
            ]
        )

        #Create a publisher for twist messages on the /cmd_vel topic
        self.twist_publisher = self.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', 10)

        #create subscriber for the /joy topic
        self.joy_subscriber = self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        #Prevent unused variable warning
        self.joy_subscriber


    def joy_callback(self, joy_msg):
        """This function is called by the node every time a joy message is posted on the '/joy' topic. It is converted to a twist message and send over the publisher"""

        #Extract the joy message data
        joy_axes = joy_msg.axes
        joy_buttons = joy_msg.buttons
       
        #Extract the linear velocity in x and y direction

        linear_x = joy_axes[0] * self.get_parameter('linear_speed__x').value
        linear_y = joy_axes[1] * self.get_parameter('linear_speed__y').value
        angular_pitch= joy_axes[4] * self.get_parameter('angular_speed__pitch').value
        angular_yaw = joy_axes[3] * self.get_parameter('angular_speed__yaw').value
        #Publish the twist commands
        self.teleop_publish(linear_x, linear_y, angular_pitch, angular_yaw)


        #self.get_logger().info("params: {}, {}, {}, {}".format(self.get_parameter('linear_speed__x').value, self.get_parameter('linear_speed__y').value, self.get_parameter('angular_speed__pitch').value, self.get_parameter('angular_speed__yaw').value))



    def teleop_publish(self, linear_x, linear_y, angular_pitch, angular_yaw):
        #Create a twist message instance
        twist = geometry_msgs.msg.Twist()

        #Multiply the linear joy inputs with the specified speeds
        twist.linear.x = linear_x #* LINEAR_SPEED[0]
        twist.linear.y = linear_y #* LINEAR_SPEED[1]
        twist.linear.z = 0.0

        
        twist.angular.x = angular_pitch #* ANGULAR_SPEED[0]
        twist.angular.y = angular_yaw #* ANGULAR_SPEED[1]
        twist.angular.z = 0.0

        #Print if running in verbose mode
        if VERBOSE_PRINT:
            print("\n============================\nPublishing:\n")
            print("Linear:\n\tx: {}\n\ty: {}\n\tz:{}".format(twist.linear.x, twist.linear.y, twist.linear.z)) 
            print("Angular:\n\tx: {}\n\ty: {}\n\tz:{}".format(twist.angular.x, twist.angular.y, twist.angular.z)) 

        #Publish the message
        self.twist_publisher.publish(twist)
        
        

def main(args=None):
    #Initialize the ros node
    rclpy.init(args=args)

    teleop_joy = TeleopJoy()

    rclpy.spin(teleop_joy)
    
    #Destroy the node explicitly
    teleop_joy.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
