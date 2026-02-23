import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist


class Prey(Node):
    def __init__(self):
        super().__init__('prey_node')
        self.create_subscription(Pose, '/prey/pose', self.pose_callback, 10)
        
        # Subscribe to all 3 predator locations
        self.create_subscription(Pose, 'predator1_location', self.predator1_callback, 10)
        self.create_subscription(Pose, 'predator2_location', self.predator2_callback, 10)
        self.create_subscription(Pose, 'predator3_location', self.predator3_callback, 10)

        self.publisher = self.create_publisher(Pose, '/prey_location', 10)
        self.prey_publisher = self.create_publisher(Twist, '/prey/cmd_vel', 10)
        
        self.current_pose = Pose()
        self.predators = {}  # Dictionary to store all predator positions

        self.start = self.get_clock().now().seconds_nanoseconds()[0]

    def pose_callback(self, msg):
        self.current_pose = msg
        self.publisher.publish(msg)

    def predator1_callback(self, msg):
        self.predators['predator1'] = (msg.x, msg.y)

    def predator2_callback(self, msg):
        self.predators['predator2'] = (msg.x, msg.y)

    def predator3_callback(self, msg):
        self.predators['predator3'] = (msg.x, msg.y)

    def move_turtle(self):
        if not self.predators:
            return
        
        move_cmd = Twist()
        
        # Check if any predator caught the prey
        for predator_name, (px, py) in self.predators.items():
            distance = math.sqrt((px - self.current_pose.x) ** 2 + (py - self.current_pose.y) ** 2)
            if distance < 0.5:
                print(f"Prey caught by {predator_name}!")
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
        
        # Calculate escape direction as sum of vectors away from all predators
        
        # Predator 1 vector
        p1 = self.predators.get('predator1')
        v1x = self.current_pose.x - p1[0] if p1 else 0.0
        v1y = self.current_pose.y - p1[1] if p1 else 0.0

        # Predator 2 vector
        p2 = self.predators.get('predator2')
        v2x = self.current_pose.x - p2[0] if p2 else 0.0
        v2y = self.current_pose.y - p2[1] if p2 else 0.0

        # Predator 3 vector
        p3 = self.predators.get('predator3')
        v3x = self.current_pose.x - p3[0] if p3 else 0.0
        v3y = self.current_pose.y - p3[1] if p3 else 0.0

        #sum of all vectors
        dx = v1x + v2x + v3x
        dy = v1y + v2y + v3y


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


def main():
    rclpy.init()
    prey_node = Prey()
    
    while rclpy.ok():
        rclpy.spin_once(prey_node)
        prey_node.move_turtle()
    
    prey_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()