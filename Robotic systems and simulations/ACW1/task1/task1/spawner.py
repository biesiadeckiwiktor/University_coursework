import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
import random

class Spawner(Node):
    def __init__(self):
        super().__init__('spawner')

        self.kill_client = self.create_client(Kill, '/kill')
        self.spawn_client = self.create_client(Spawn, '/spawn')

        self.kill_client.wait_for_service()
        self.spawn_client.wait_for_service()

        # Kill default turtle for clean slate
        kill_request = Kill.Request()
        kill_request.name = 'turtle1' # Automatically spawned turtle is named turtle1
        future = self.kill_client.call_async(kill_request)
        rclpy.spin_until_future_complete(self, future)

 

        # Spawning predator and prey on clean slate in random positions

        self.spawn_turtle('prey', random.uniform(1, 10), random.uniform(1, 10))
        self.spawn_turtle('predator', random.uniform(1, 10), random.uniform(1, 10))

    def spawn_turtle(self, name, x, y):
        request = Spawn.Request()
        request.name = name
        request.x = x
        request.y = y
        request.theta = 0.0
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
         
        if future.result() is not None:
            self.get_logger().info(f"Successfully spawned {future.result().name}")
        else:
            self.get_logger().error(f"Failed to spawn {future.result().name}.")

def main():
    rclpy.init()
    node = Spawner()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except KeyboardInterrupt:
        pass