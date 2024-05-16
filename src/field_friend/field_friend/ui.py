#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
from geometry_msgs.msg import Twist, Vector3
from nicegui import ui
import json

class UINode(Node):
    def __init__(self):
        super().__init__('ui')
        self.publish_twist = self.create_publisher(Twist, '/steer', 1)
        self.publish_configure = self.create_publisher(Empty, '/configure', 1)
        self.subscription_status = self.create_subscription(String, '/status', self.handle_status, 1)
        self.subscription_odometry = self.create_subscription(Twist, '/odometry', self.handle_odometry, 1)

        with ui.row().classes('items-stretch'):
            with ui.card():
                ui.markdown('### Setup')
                ui.button('Configure', on_click=lambda: self.publish_configure.publish(Empty()))
            with ui.card() as status:
                ui.markdown('### Status')
                self.time = ui.label()
            with ui.card():
                ui.markdown('### Control')
                ui.joystick(
                    color='blue',
                    size=50,
                    on_move=lambda msg: self.send(msg.data.vector.x, msg.data.vector.y),
                    on_end=lambda _: self.send(0, 0),
                )
            with ui.card() as odometry:
                ui.markdown('### Odometry')
                ui.label('linear')
                self.linear = ui.slider(min=-1.0, max=1.0, step=0.01).props('label-always readonly')
                ui.label('angular')
                self.angular = ui.slider(min=-1.0, max=1.0, step=0.01).props('label-always readonly')

            ui.timer(0.1, lambda: None)  # NOTE: update ui

        ui.run(title='ROS Example', reload=False, show=False)

    def send(self, x, y):
        linear = Vector3(y, 0, 0)
        angular = Vector3(0, 0, -x)
        self.publish_twist.publish(Twist(linear, angular))

    def handle_status(self, data):
        msg = json.loads(data.data)
        self.time.text = f'Timestamp: {msg["time"]} ms'

    def handle_odometry(self, data):
        self.linear.value = data.linear.x
        self.angular.value = data.angular.z

def main(args=None):
    rclpy.init(args=args)
    node = UINode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()