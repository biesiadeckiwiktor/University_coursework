import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class Prey(Node):
    def __init__(self):
        super().__init__('prey_node')
        self.create_subscription(Pose, '/prey/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Pose, '/prey_location', 10)

    def pose_callback(self, msg):
        self.publisher.publish(msg)

def main():
    rclpy.init()
    node = Prey()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()