import rclpy
import geometry_msgs.msg 

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

#Define the linear speed multipliers to be used when sending twist commands, denoted as [x, y]
#(z is skipped for the robomaster S1 cannot fly
LINEAR_SPEED = [40.0, 40.0] 

#Define the angular speed multipliers to be used when sending twist commands, denoted as [x, y]
#(z is skipped for the S1's gimbal does not turn around the z-axis
ANGULAR_SPEED = [40.0, 40.0]



#A variable determining wether verbose prints should be used
VERBOSE_PRINT = True


class TeleopJoy(Node):
    """This class inherits from the ROS2 node class and converts messages
       from the /joy topic to twist messages on the /cmd_vel topic
    """

    def __init__(self):
        super().__init__("teleop_joy")

        #These should be changed to ROS parameters
        self.linear_speed_x = 1.0
        self.linear_speed_y = 1.0
        self.angular_speed_pitch = 1.0
        self.angular_speed_yaw = 1.0
    

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
        linear_x = joy_axes[0] * self.linear_speed_x
        linear_y = joy_axes[1] * self.linear_speed_y
        angular_pitch = joy_axes[4] * self.angular_speed_pitch
        angular_yaw = joy_axes[3] * self.angular_speed_yaw

    
        #Publish the twist commands
        self.teleop_publish(linear_x, linear_y, angular_pitch, angular_yaw)



    def teleop_publish(self, linear_x, linear_y, angular_pitch, angular_yaw):
        #Create a twist message instance
        twist = geometry_msgs.msg.Twist()

        #Multiply the linear joy inputs with the specified speeds
        twist.linear.x = linear_x * LINEAR_SPEED[0]
        twist.linear.y = linear_y * LINEAR_SPEED[1]
        twist.linear.z = 0.0

        
        twist.angular.x = angular_pitch * ANGULAR_SPEED[0]
        twist.angular.y = angular_yaw * ANGULAR_SPEED[1]
        twist.angular.z = 0.0

        #Print if running in verbose mode
        if VERBOSE_PRINTING:
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
