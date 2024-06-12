import math
import threading
from pathlib import Path

import rclpy
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from nicegui import Client, app, ui, ui_run


class NiceGuiNode(Node):

    def __init__(self) -> None:
        super().__init__('nicegui')
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.subscription = self.create_subscription(Odometry, '/odometry/global', self.handle_pose, 1)

        with Client.auto_index_client:
            with ui.row().classes('items-stretch'):
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Control').classes('text-2xl')
                    ui.joystick(color='blue', size=50,
                                on_move=lambda e: self.send_speed(float(e.x),float(e.y)),
                                on_end=lambda _: self.send_speed(0.0, 0.0))
                    ui.label('Publish steering commands by dragging your mouse around in the blue field').classes('mt-6')
                with ui.card().classes('w-44 text-center items-center'):
                    ui.label('Data').classes('text-2xl')
                    ui.label('linear velocity').classes('text-xs mb-[-1.8em]')
                    slider_props = 'readonly selection-color=transparent'
                    self.linear = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('angular velocity').classes('text-xs mb-[-1.8em]')
                    self.angular = ui.slider(min=-1, max=1, step=0.05, value=0).props(slider_props)
                    ui.label('position').classes('text-xs mb-[-1.4em]')
                    self.position = ui.label('---')
                with ui.card().classes('w-96 h-96 items-center'):
                    ui.label('Visualization').classes('text-2xl')
                    with ui.scene(350, 300) as scene:
                        with scene.group() as self.robot_3d:
                            prism = [[-0.5, -0.5], [0.5, -0.5], [0.75, 0], [0.5, 0.5], [-0.5, 0.5]]
                            self.robot_object = scene.extrusion(prism, 0.4).material('#4488ff', 0.5)

    def send_speed(self, y, x):
        cmd_message = Twist()
        cmd_message.linear.x, cmd_message.linear.y, cmd_message.linear.z = 0.5 * float(x), 0.0, 0.0
        cmd_message.angular.x, cmd_message.angular.y, cmd_message.angular.z = 0.0, 0.0, 0.5 * float(-y)
        self.linear.value = x
        self.angular.value = y
        self.cmd_vel_publisher.publish(cmd_message)


    def handle_pose(self, msg: Odometry) -> None:
        self.position.text = f'x: {msg.pose.pose.position.x:.2f}, y: {msg.pose.pose.position.y:.2f}'
        self.robot_3d.move(msg.pose.pose.position.x, msg.pose.pose.position.y)
        self.robot_3d.rotate(0, 0, 2 * math.atan2(msg.pose.pose.orientation.z, msg.pose.pose.orientation.w))


def main() -> None:
    # NOTE: This function is defined as the ROS entry point in setup.py, but it's empty to enable NiceGUI auto-reloading
    pass


def ros_main() -> None:
    rclpy.init()
    node = NiceGuiNode()
    try:
        rclpy.spin(node)
    except ExternalShutdownException:
        pass


app.on_startup(lambda: threading.Thread(target=ros_main).start())
ui_run.APP_IMPORT_STRING = f'{__name__}:app'  # ROS2 uses a non-standard module name, so we need to specify it here
ui.run(uvicorn_reload_dirs=str(Path(__file__).parent.resolve()), favicon='🤖')