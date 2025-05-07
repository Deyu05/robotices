#!/usr/bin/env python3
# A simple ROS2 Publisher

import math
import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist 
from nav_msgs.msg import Odometry 

from com2009_team69_2025_modules.tb3_tools import quaternion_to_euler 
from math import sqrt, pow, pi 

class Eight(Node):
    def __init__(self):
        # Initialize node and variables
        super().__init__("move_eight_shape")
        self.first_message = False
        self.second_loop_flag = False 
        self.counter = 0
        self.terminal = 0

        # Twist message for velocity commands
        self.vel_msg = Twist() 
        
        # Current and reference positions/orientations
        self.x = 0.0; self.y = 0.0; self.theta_z = 0.0
        self.xref = 0.0; self.yref = 0.0; self.theta_zref = 0.0

        # Publisher for velocity commands
        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )

        # Subscriber for odometry data
        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic="odom",
            callback=self.odom_callback,
            qos_profile=10,
        )

        # Timer for regular control updates (10 Hz)
        ctrl_rate = 10 
        self.timer = self.create_timer(1/ctrl_rate, self.timer_callback)
        self.shutdown = False

        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

    def on_shutdown(self):
        # Stop the robot on shutdown
        print("Stopping the robot...")
        self.vel_pub.publish(Twist())
        self.shutdown = True

    def odom_callback(self, msg_data: Odometry):
        # Update current position and orientation from odometry
        pose = msg_data.pose.pose
        (roll, pitch, yaw) = quaternion_to_euler(pose.orientation)
        self.x = pose.position.x 
        self.y = pose.position.y
        self.theta_z = abs(yaw)

        # Set reference on first message
        if not self.first_message: 
            self.first_message = True
            self.xref, self.yref, self.theta_zref = self.x, self.y, self.theta_z

        # If more than 20 messages have been received (~1 second), log the state and reset the counter;
        # otherwise, increment the counter.
        if self.terminal > 20: 
            self.terminal = 0
            self.get_logger().info(
                f"x = {self.x:.2f} [m], y = {self.y:.2f} [m], theta_z = {math.degrees(yaw):.1f} [degrees]."
            )
        else:
            self.terminal += 1
    
    def timer_callback(self):
        # Compute displacement and angular difference from the reference
        fwd_vel = 0.11
        radius = 0.5
        total_displacement = sqrt(pow(self.xref - self.x, 2) + pow(self.yref - self.y, 2))
        total_spin_movement = abs(self.theta_zref - self.theta_z)
        
        if not self.second_loop_flag:
            # First motion segment: move forward and turn
            self.vel_msg.linear.x = fwd_vel
            self.vel_msg.angular.z = fwd_vel / radius
            self.counter += 1
            if total_displacement <= 0.08 and total_spin_movement <= 0.08 and self.counter > 10:
                self.second_loop_flag = True
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0
                self.xref, self.yref, self.theta_zref = self.x, self.y, self.theta_z
                self.counter = 0
        else:
            # Second motion segment: move forward and turn in the opposite direction
            self.vel_msg.linear.x = fwd_vel
            self.vel_msg.angular.z = -(fwd_vel / radius)
            self.counter += 1
            if total_displacement <= 0.08 and total_spin_movement <= 0.08 and self.counter > 10:
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = 0.0

        self.vel_pub.publish(self.vel_msg)

def main(args=None):
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    move_eight_shape = Eight()
    try:
        rclpy.spin(move_eight_shape)
    except KeyboardInterrupt:
        print(f"{move_eight_shape.get_name()} received a shutdown request (Ctrl+C).")
    finally:
        move_eight_shape.on_shutdown()
        while not move_eight_shape.shutdown:
            continue
        move_eight_shape.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
