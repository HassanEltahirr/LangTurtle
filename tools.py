from langchain.tools import tool
import rclpy
from rclpy.node import Node
from turtlesim.srv import TeleportAbsolute

@tool
def teleport_turtle(input_str: str):
    """
    Teleports the turtle to (x, y) in turtlesim.
    Expects input in the format: "x,y" (e.g., "5.0,3.0")
    """
    try:
        x_str, y_str = input_str.split(",")
        x = float(x_str.strip())
        y = float(y_str.strip())
    except Exception as e:
        return f"Error: input should be 'x,y' (e.g., '5.0,3.0'). {e}"

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

@tool
def return_to_start(input_str: str):
    """
    Returns the turtle to the original starting point at (5.5, 5.5).
    Input is ignored.
    """
    rclpy.init()
    node = Node('return_to_start_client')
    client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
    req = TeleportAbsolute.Request()
    req.x = 5.5
    req.y = 5.5
    req.theta = 0.0
    while not client.wait_for_service(timeout_sec=1.0):
        print('service not available, waiting...')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    node.destroy_node()
    rclpy.shutdown()
    return "You're welcome!"

from turtlesim.srv import SetPen

@tool
def thank_you_turtle_tool(input_str: str):
    """
    Turns off the turtle's pen, teleports it to the center, and says 'You're welcome!'.
    Triggered when the user says 'thank you mr turtle'.
    """
    rclpy.init()
    node = Node('thank_you_turtle_client')

    # 1. Disable pen
    pen_client = node.create_client(SetPen, '/turtle1/set_pen')
    while not pen_client.wait_for_service(timeout_sec=1.0):
        print("Waiting for /turtle1/set_pen service...")

    pen_req = SetPen.Request()
    pen_req.r = 0
    pen_req.g = 0
    pen_req.b = 0
    pen_req.width = 1
    pen_req.off = True
    pen_future = pen_client.call_async(pen_req)
    rclpy.spin_until_future_complete(node, pen_future)

    # 2. Teleport to (5.5, 5.5)
    teleport_client = node.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
    while not teleport_client.wait_for_service(timeout_sec=1.0):
        print("Waiting for /turtle1/teleport_absolute...")

    teleport_req = TeleportAbsolute.Request()
    teleport_req.x = 5.5
    teleport_req.y = 5.5
    teleport_req.theta = 0.0
    teleport_future = teleport_client.call_async(teleport_req)
    rclpy.spin_until_future_complete(node, teleport_future)

    node.destroy_node()
    rclpy.shutdown()

    return "You're welcome!"
