import rclpy
import geometry_msgs.msg 

from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Header




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

        #ALL PARTS BELOW NEED TO BE FINETUNED ONCE RUN WITH THE ACTUAL TOPIC
        """
        print("\n============================\nNEW MESSAGE:\n")
        print("Axes data:")
        for axis_index in range(0, len(joy_axes)):
            print("\tAxis {}:\t{}".format(axis_index, joy_axes[axis_index]))
        print("Button data:")
        for button_index in range(0, len(joy_buttons)):
            print("\tButton {}:\t{}".format(button_index, joy_buttons[button_index]))
        """
        
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
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.linear.z = 0.0
        twist.angular.x = angular_pitch
        twist.angular.y = angular_yaw
        twist.angular.z = 0.0
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
