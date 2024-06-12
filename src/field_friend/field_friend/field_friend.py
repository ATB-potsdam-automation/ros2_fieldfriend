#!/usr/bin/env python3
import json
import os.path
from math import cos, sin, atan2
from functools import reduce
from operator import ixor

import rclpy
from rclpy.node import Node
from rclpy.time import Time
import serial
from geometry_msgs.msg import Twist, Vector3, TransformStamped, PoseStamped, Quaternion
from std_msgs.msg import Empty, String
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler, euler_from_quaternion
from nav_msgs.msg import Odometry


from launch_ros.substitutions import FindPackageShare

package_name = "field_friend"

class FieldFriendControl(Node):

    def __init__(self):
        super().__init__('field_friend_controller')
        self.odometry_publisher = self.create_publisher(Odometry, 'odometry', 10)
        self.status_publisher = self.create_publisher(String, 'status', 10)
        self.cmd_subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.config_subscription = self.create_subscription(
            String,
            'configure',
            self.handle_configure,
            10)
        self.cmd_subscription  # prevent unused variable warning
        self.config_subscription  # prevent unused variable warning
        self.port = None
        self.publish_tf = False
        self.base_frame_id = 'base_link'
        self.odom_frame = 'odom'
        self.current_pose = PoseStamped()
        self.reset_odometry()

        # tf broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        self.open_port()

        # Create a timer to send odometry messages at 10 Hz
        self.odom_timer = self.create_timer(0.001, self.read_serial)


    def reset_odometry(self):
        self.current_pose.header.frame_id = self.odom_frame
        self.current_pose.header.stamp = self.get_clock().now().to_msg()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        self.current_pose.pose.orientation.x = 0.0
        self.current_pose.pose.orientation.y = 0.0
        self.current_pose.pose.orientation.z = 0.0
        self.current_pose.pose.orientation.w = 1.0

    def open_port(self):
        try:
            self.port = serial.Serial('/dev/esp', 115200)
        except serial.SerialException:
            self.port = None


    def send(self, line):
        if self.port is not None:
            checksum = reduce(ixor, map(ord, line))
            line = f'{line}@{checksum:02x}\n'
            self.port.write(line.encode())
        else:
            self.get_logger().warning('no Port open')

    def config_callback(self, config_msg):
        pass
        
    def cmd_callback(self, cmd_msg):
        self.send(f'wheels.speed({cmd_msg.linear.x:3f}, {cmd_msg.angular.z:.3f})')
        # self.get_logger().info('received drive command')
    
    def handle_configure(self, data):
        self.get_logger().info('received data: ', data)
        with open(FindPackageShare(package_name) + '/startup.liz') as f:
            self.send('!-')
            for line in f.read().splitlines():
                self.send('!+' + line)
            self.send('!.')
            self.send('core.restart()')
            
    def read_serial(self):
        if self.port is not None:
            try:
                line = self.port.readline().decode().strip()
                self.port.flushOutput()
            except UnicodeDecodeError:
                return
            if line[-3:-2] == '@':
                line, checksum = line.split('@')
                if reduce(ixor, map(ord, line)) != int(checksum, 16):
                    return

            # print(line)

            words = line.split()
            if not any(words):
                return
            if words.pop(0) != 'core':
                return  
            time = int(words.pop(0))
            linear_speed = float(words.pop(0))
            angular_speed = float(words.pop(0))

            self.compute_odometry(float(linear_speed), 0, float(angular_speed), self.get_clock().now())
            st = String()
            st.data = json.dumps({
                'time': time,
            })
            self.status_publisher.publish(st)
        else:
            self.get_logger().warn('unable to read from serial')

    def compute_odometry(self, linear_x : float, linear_y : float, angular_z : float, ros_time : rclpy.time.Time):

        # self.get_logger().info('current position x , y: 'linear_x, linear_y))
        
        (_, _, yaw) = euler_from_quaternion([self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])
    
        delta_t = (ros_time.nanoseconds - Time.from_msg(self.current_pose.header.stamp).nanoseconds) / 1e9
        
        # compute new position from linear and angular speed
        # compute new orientation from angular speed and current orientation
        
        current_yaw = yaw + angular_z * delta_t         # compute yaw from quaternion and add angular speed * delta_t
        self.current_pose.pose.position.x += linear_x * cos(current_yaw) * delta_t
        self.current_pose.pose.position.y += linear_x * sin(current_yaw) * delta_t
        quat_tf = quaternion_from_euler(0, 0, current_yaw)
        self.current_pose.pose.orientation = Quaternion(x=quat_tf[0], y=quat_tf[1], z=quat_tf[2], w=quat_tf[3])
        self.current_pose.header.stamp = ros_time.to_msg()

        odom_msg = Odometry()
        odom_msg.header = self.current_pose.header
        odom_msg.child_frame_id = self.base_frame_id
        odom_msg.pose.pose = self.current_pose.pose
        odom_msg.twist.twist.linear.x = float(linear_x)
        odom_msg.twist.twist.linear.y = float(linear_y)
        odom_msg.twist.twist.angular.z = angular_z

        # TODO add covariances

        self.odometry_publisher.publish(odom_msg)



        if self.publish_tf:
            t = TransformStamped()
            # set frame parameters
            t.header.stamp = ros_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame_id
            # Create the transformation
            t.transform.translation.x = self.current_pose.pose.position.x
            t.transform.translation.y = self.current_pose.pose.position.y
            t.transform.translation.z = 0.0

            t.transform.rotation.x = self.current_pose.pose.orientation.x
            t.transform.rotation.y = self.current_pose.pose.orientation.y
            t.transform.rotation.z = self.current_pose.pose.orientation.z
            t.transform.rotation.w = self.current_pose.pose.orientation.w
            # Send the transformation
            self.tf_broadcaster.sendTransform(t)

    # make sure to close the serial port when the node is destroyed
    def destroy_node(self):
        if self.port is not None:
            self.port.close()
        super().destroy_node()



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
