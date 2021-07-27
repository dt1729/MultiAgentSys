import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist

def main():
    rclpy.init()
    node = rclpy.create_node('vel_cmd')
    publisher = node.create_publisher(Twist, '/cmd_vel', 10)
    while True:
        vel_msg = Twist()
        speed = 1.0
        isForward = True
        #defining the message values
        if(isForward):
            vel_msg.linear.x = abs(speed)
        else:
            vel_msg.linear.x = -abs(speed)
        #Since we are moving just in x-axis
        vel_msg.linear.y = 0.0
        vel_msg.linear.z = 0.0
        vel_msg.angular.x = 0.0
        vel_msg.angular.y = 0.0
        vel_msg.angular.z = 0.0
        publisher.publish(vel_msg)
        rclpy.spin_once(node,timeout_sec=0.1)

    #Destroying the node once the task is complete
    node.destroy_node()
    rclpy.shutdown()



if __name__ == "__main__":
    main()
