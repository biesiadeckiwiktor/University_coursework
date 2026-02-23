import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from xml.dom import minidom
import os


class Predator(Node):
    def __init__(self):
        super().__init__('predator_node')

        self.predator_publisher = self.create_publisher(Twist, '/predator/cmd_vel', 10)
        self.location_pub = self.create_publisher(Pose, '/predator_location', 10)

        self.pose_subscriber = self.create_subscription(Pose, '/predator/pose', self.pose_callback, 10)
        self.prey_subscriber = self.create_subscription(Pose, '/prey_location', self.prey_callback, 10)

        self.current_pose = Pose()
        self.target_x = None
        self.target_y = None

        self.start = self.get_clock().now().seconds_nanoseconds()[0]

        self.virtual_x = None
        self.virtual_y = None

        self.virtual_reached = False

        # reading from file
        if os.path.exists('predator-kb.xml'):
            xmldoc = minidom.parse('predator-kb.xml')
            move = xmldoc.getElementsByTagName("move")
            prey = move[200].getElementsByTagName("prey")[0]
            virtual_coordinates = prey.firstChild.data.split(',')
            # saving coordinates from previous run into variables
            self.virtual_x = float(virtual_coordinates[0])
            self.virtual_y = float(virtual_coordinates[1])
        

    def pose_callback(self, msg: Pose):
        self.current_pose = msg
        self.location_pub.publish(msg)

    def prey_callback(self, msg: Pose):
        self.target_x = msg.x
        self.target_y = msg.y

    def move_turtle(self):
        # if virtual_reached is false assign virtual target coordinates to current target for predator
        if not self.virtual_reached and self.virtual_x is not None and self.virtual_y is not None:
            target_x = self.virtual_x
            target_y = self.virtual_y
        # when virtual target was reached predator comes back to following prey
        elif self.target_x is not None and self.target_y is not None:
            target_x = self.target_x
            target_y = self.target_y
        else:
            return

        move_cmd = Twist()
        distance = math.sqrt((target_x - self.current_pose.x) ** 2 + (target_y - self.current_pose.y) ** 2)

        # check if virtual target was reached
        if not self.virtual_reached and distance < 0.3:
            self.virtual_reached = True
            return

        # check if prey was caught
        if distance < 0.3:
            print("Prey caught!")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.predator_publisher.publish(move_cmd)
            return
        
        # check if 60s passed
        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.start >= 60:
            print("Prey escaped!")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.predator_publisher.publish(move_cmd)
            return

        angle_to_goal = math.atan2(target_y - self.current_pose.y , target_x - self.current_pose.x)
        angle_diff = angle_to_goal - self.current_pose.theta

        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi

        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 4.0 * angle_diff

        self.predator_publisher.publish(move_cmd)

        


def main(args=None):
    rclpy.init(args=args)
    predator_node = Predator()
    try:
        while rclpy.ok():
            rclpy.spin_once(predator_node)
            predator_node.move_turtle()
    finally:
        predator_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()