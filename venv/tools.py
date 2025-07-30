# tools.py
from langchain.tools import tool
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

@tool
def teleport_turtle(x: float, y: float):
    """Teleports the turtle to (x, y) in turtlesim."""
    rclpy.init()
    node = Node('teleport_turtle_client')
    client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
    req = TeleportAbsolute.Request()
    req.x = x
    req.y = y
    req.theta = 0.0
    while not client.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    node.destroy_node()
    rclpy.shutdown()
    return f"Turtle teleported to ({x}, {y})"