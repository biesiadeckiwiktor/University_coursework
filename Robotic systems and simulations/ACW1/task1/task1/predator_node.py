import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import math

class Predator(Node):
    def __init__(self):
        super().__init__('predator_node')

        
        self.publisher_ = self.create_publisher(Twist, '/predator/cmd_vel', 10)
        
        self.pose_subscriber = self.create_subscription(Pose, '/predator/pose', self.pose_callback, 10)
        self.prey_subscriber = self.create_subscription(Pose, '/prey_location', self.prey_callback, 10)
        
        self.current_pose = Pose()

        self.target_x = None
        self.target_y = None


        print ('Node initialised')

    def pose_callback(self, msg: Pose):
        self.current_pose = msg

    def prey_callback(self, msg: Pose):
        self.target_x = msg.x
        self.target_y = msg.y

    def move_turtle(self):

        
        
        if self.target_x is None or self.target_y is None:
            return
        
        move_cmd = Twist()

        distance = math.sqrt((self.target_x - self.current_pose.x)**2 + (self.target_y - self.current_pose.y)**2)

        if distance < 0.5:
            print(f"Turtle reached the goal at x: {self.target_x}, y: {self.target_y}")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.publisher_.publish(move_cmd)
            return
        
        angle_to_goal = math.atan2(self.target_y - self.current_pose.y, self.target_x - self.current_pose.x)
        angle_diff = angle_to_goal - self.current_pose.theta

        if angle_diff > math.pi:
            angle_diff -= 2*math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2*math.pi

        move_cmd.linear.x = 1.0 * distance 
        move_cmd.angular.z = 4.0 * angle_diff

        self.publisher_.publish(move_cmd)


def main(args=None):
    rclpy.init(args=args)

    predator_node = Predator()

    while rclpy.ok():
        rclpy.spin_once(predator_node)
        predator_node.move_turtle()

    predator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
