import rclpy
from std_msgs.msg import String


def listener_callback(msg):
    print('I heard: "%s"' % msg.data)

def main():
    rclpy.init()
    node = rclpy.create_node('subscriber')
    subscriber = node.create_subscription(String,'/topic',listener_callback,10)
    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
