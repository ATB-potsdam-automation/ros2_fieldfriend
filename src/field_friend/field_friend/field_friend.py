#!/usr/bin/env python3
import json
import os.path
from functools import reduce
from operator import ixor

import rclpy
from rclpy.node import Node
import serial
from geometry_msgs.msg import Twist, Vector3
from std_msgs.msg import Empty, String

            
class FieldFriendControl(Node):

    def __init__(self):
        super().__init__('field_friend_controller')
        self.odometry_publisher = self.create_publisher(String, 'odometry', 10)
        self.status_publisher = self.create_publisher(String, 'status', 10)
        self.cmd_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.config_subscription = self.create_subscription(
            String,
            'configure',
            self.config_callback,
            10)
        self.cmd_subscription  # prevent unused variable warning
        self.config_subscription  # prevent unused variable warning
        self.port = None

    def send(self, line):
        if self.port is not None:
            checksum = reduce(ixor, map(ord, line))
            line = f'{line}@{checksum:02x}\n'
            self.port.write(line.encode())

    def config_callback(self, config_msg):
        pass
        
    def cmd_callback(self, cmd_msg):
        self.send(f'wheels.speed({cmd_msg.linear.x:3f}, {cmd_msg.angular.z:.3f})')
        self.get_logger().info('received drive command')
    
    def handle_configure(self, data):
        with open(os.path.dirname(__file__) + '/../startup.liz') as f:
            self.send('!-')
            for line in f.read().splitlines():
                self.send('!+' + line)
            self.send('!.')
            self.send('core.restart()')
            
    def read_serial(self):
        with serial.Serial('/dev/esp', 115200) as self.port:
            while self.context.ok():
                try:
                    line = self.port.readline().decode().strip()
                except UnicodeDecodeError:
                    continue
                if line[-3:-2] == '@':
                    line, checksum = line.split('@')
                    if reduce(ixor, map(ord, line)) != int(checksum, 16):
                        continue

                words = line.split()
                if not any(words):
                    continue
                if words.pop(0) != '!"core':
                    continue
                time = int(words.pop(0))
                linear_speed = float(words.pop(0))
                angular_speed = float(words.pop(0))

                self.odometry_publisher.publish(Twist(
                    Vector3(linear_speed, 0, 0),
                    Vector3(0, 0, angular_speed),
                ))
                self.status_publisher.publish(json.dumps({
                    'time': time,
                }))



def main(args=None):
    rclpy.init(args=args)

    field_friend = FieldFriendControl()

    rclpy.spin(field_friend)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    field_friend.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
