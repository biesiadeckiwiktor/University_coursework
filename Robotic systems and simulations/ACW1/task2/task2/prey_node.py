import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Prey(Node):
    def __init__(self):
        super().__init__('prey_node')
        self.create_subscription(Pose, '/prey/pose', self.pose_callback, 10)
        self.create_subscription(Pose, '/predator_location', self.predator_callback, 10)
        self.publisher = self.create_publisher(Pose, '/prey_location', 10)
        self.prey_publisher = self.create_publisher(Twist, '/prey/cmd_vel', 10)
        
        self.current_pose = Pose()
        self.predator_x = None
        self.predator_y = None

        self.start = self.get_clock().now().seconds_nanoseconds()[0]

    def pose_callback(self, msg):
        self.current_pose = msg
        self.publisher.publish(msg)

    def predator_callback(self, msg):
        self.predator_x = msg.x
        self.predator_y = msg.y

    def move_turtle(self):
        if self.predator_x is None or self.predator_y is None:
            return
        
        move_cmd = Twist()
        distance = math.sqrt((self.predator_x - self.current_pose.x) ** 2 + (self.predator_y - self.current_pose.y) ** 2)

        if distance < 0.3:
            print("Prey caught!")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.prey_publisher.publish(move_cmd)
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.start >= 60:
            print("Prey escaped!")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.prey_publisher.publish(move_cmd)
            return
        
        
        
        dx = self.current_pose.x - self.predator_x
        dy = self.current_pose.y - self.predator_y

        if self.current_pose.x <= 1.0 or self.current_pose.x >= 10.0:
            dx = -dx
        if self.current_pose.y <= 1.0 or self.current_pose.y >= 10.0:
            dy = -dy

        escape_angle = math.atan2(dy, dx)
        angle_diff = escape_angle - self.current_pose.theta
        
        # Normalize angle
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 4.0 * angle_diff
        self.prey_publisher.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)
    prey_node = Prey()
    try:
        while rclpy.ok():
            rclpy.spin_once(prey_node)
            prey_node.move_turtle()
    finally:    
        prey_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()