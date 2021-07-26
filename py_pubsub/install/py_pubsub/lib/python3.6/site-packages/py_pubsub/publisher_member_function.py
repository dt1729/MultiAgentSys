import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('talker')
    publisher = node.create_publisher(String, 'topic', 10)
    while True:
        print("First Publisher")
        msg = String()
        msg.data = "First Publisher"
        publisher.publish(msg)
        rclpy.spin_once(node,timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
