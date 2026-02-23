import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
import xml.etree.cElementTree as ET


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

        # list to store coordinates of each turtle and variable to store outcome
        self.moves = []
        self.result = None

    # function to record coordinates
    def record_move(self):
        self.moves.append({
            'predator': (self.predator_x, self.predator_y),
            'prey': (self.current_pose.x, self.current_pose.y)
        })

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
            self.result = 'capture' # setting result
            return

        now = self.get_clock().now().seconds_nanoseconds()[0]
        if now - self.start >= 60:
            print("Prey escaped!")
            move_cmd.linear.x = 0.0
            move_cmd.angular.z = 0.0
            self.prey_publisher.publish(move_cmd)
            self.result = 'escape' # setting result
            return
        
        
        
        dx = self.current_pose.x - self.predator_x
        dy = self.current_pose.y - self.predator_y

        # save current position to list
        if self.current_pose.x <= 1.0 or self.current_pose.x >= 10.0:
            dx = -dx
            self.record_move() 
        if self.current_pose.y <= 1.0 or self.current_pose.y >= 10.0:
            dy = -dy
            self.record_move() 

        escape_angle = math.atan2(dy, dx)
        angle_diff = escape_angle - self.current_pose.theta
        
        if angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        elif angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        move_cmd.linear.x = 2.0
        move_cmd.angular.z = 4.0 * angle_diff
        self.prey_publisher.publish(move_cmd)

# function to save xml file in desired format
def saveXML(moves, result):
    root = ET.Element("xml")
    trial = ET.SubElement(root, "trial")
    ET.SubElement(trial, "number").text = "1"

    for i, move in enumerate(moves):
        move_num = ET.SubElement(trial, "move", number=str(i+1)) # enumerating each move
        ET.SubElement(move_num,"predator").text = f"{move['predator'][0]},{move['predator'][1]}" # predator coordinates
        ET.SubElement(move_num, "prey").text = f"{move['prey'][0]},{move['prey'][1]}" # prey coordinates
    if result:
        ET.SubElement(trial, "result").text = result # result of simulation caught/escaped


    tree = ET.ElementTree(root)
    tree.write('predator-kb.xml')

def main(args=None):
    rclpy.init(args=args)
    prey_node = Prey()
    try:
        while rclpy.ok():
            rclpy.spin_once(prey_node)
            prey_node.move_turtle()
    finally:
        saveXML(prey_node.moves, prey_node.result)  # calling function when simulation finishes  
        prey_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()