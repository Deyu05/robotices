#!/usr/bin/env python3

from math import degrees, sqrt

from com2009_team69_2025.action import ExploreForward

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tuos_examples.tb3_tools import quaternion_to_euler

from sensor_msgs.msg import LaserScan 

import numpy as np 

class AdvancedExplorationServer(Node):

    def __init__(self):
        super().__init__('explore_server')

        self.start_time = self.get_clock().now().seconds_nanoseconds
        self.first_message = False
        self.front_dist = float("nan") 
        self.lidar_reading = 0.0
        self.displacement = 0
        self.ref_displacement = 0
        self.initial_turn = True
        self.left_turn_completed = False

        self.shutdown = False 

        self.await_odom = True 
        self.await_lidar = True

        self.loop_rate = self.create_rate(
            frequency=20, 
            clock=self.get_clock()
        ) 

        self.vel_pub = self.create_publisher(
            msg_type=Twist,
            topic="cmd_vel",
            qos_profile=10,
        )
        self.vel_pub.publish(Twist())

        self.odom_sub = self.create_subscription(
            msg_type=Odometry,
            topic='odom',
            callback=self.odom_callback,
            qos_profile=10
        )
        
        self.lidar_sub = self.create_subscription(
            msg_type=LaserScan,
            topic="/scan",
            callback=self.lidar_callback,
            qos_profile=10,
        ) 

        self.get_logger().info(f"The '{self.get_name()}' node is initialised.")

        self._action_server = ActionServer(
            node=self,
            action_type = ExploreForward,
            action_name = 'explore_forward',
            execute_callback=self.server_execution_callback,
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)
        
        
    def odom_callback(self, odom_msg):
        pose = odom_msg.pose.pose

        _, _, yaw = quaternion_to_euler(
            pose.orientation
        )
        self.yaw = abs(degrees(yaw))

        self.x = pose.position.x 
        self.y = pose.position.y
        self.theta_z = abs(yaw) 

        if not self.first_message: 
            self.first_message = True
            self.xref = self.x
            self.yref = self.y
            self.theta_zref = self.theta_z
        
        self.await_odom = False
    
    def lidar_callback(self, scan_data: LaserScan): 
        left_20_deg = scan_data.ranges[0:11]
        right_20_deg = scan_data.ranges[-10:] 
        f_left = scan_data.ranges[11:31]
        f_right = scan_data.ranges[-30:-10] 
        right_side_deg = scan_data.ranges[260:280] 
        left_side_deg = scan_data.ranges[80:100] 
        desire = scan_data.ranges[75:105] 
        front = np.array(left_20_deg + right_20_deg) 

        left = np.array(f_left)
        right = np.array(f_right)
        right_side = np.array(right_side_deg)
        left_side = np.array(left_side_deg)
        desire_left = np.array(desire)
        valid_data = front[front != float("inf")] 
        valid_left = left[left != float("inf")] 
        valid_right = right[right != float("inf")] 
        valid_right_side = right_side[right_side != float("inf")] 
        valid_left_side = left_side[left_side != float("inf")] 
        valid_desire = desire_left[desire_left != float("inf")] 
        self.front_dist = []
        
        if np.shape(valid_data)[0] > 0: 
            front_average = valid_data.mean() 
        else:
            front_average = float("nan") 

        if np.shape(valid_left)[0] > 0: 
            front_left = valid_left.mean()
        else:
            front_left = float("nan") 
        
        if np.shape(valid_right)[0] > 0: 
            front_right = valid_right.mean()
        else:
            front_right = float("nan") 
        
        if np.shape(valid_right_side)[0] > 0: 
            right_average = valid_right_side.mean()
        else:
            right_average = float("nan") 

        if np.shape(valid_left_side)[0] > 0: 
            left_average = valid_left_side.mean()
        else:
            left_average = float("nan") 

        if np.shape(valid_desire)[0] > 0: 
            desire_average = valid_desire.mean()
        else:
            desire_average = float("nan") 

        self.front_dist.append(front_average)
        self.front_dist.append(front_left)
        self.front_dist.append(front_right)
        self.front_dist.append(left_average)
        self.front_dist.append(right_average)
        self.front_dist.append(desire_average)
        

        self.get_logger().info(
            f"LiDAR Reading (front): {front_average:.3f} meters."
            f"LiDAR Reading (front-left): {front_left:.3f} meters."
            f"LiDAR Reading (front-right): {front_right:.3f} meters."
            f"LiDAR Reading (left): {left_average:.3f} meters."
            f"LiDAR Reading (right): {right_average:.3f} meters."
             f"LiDAR Reading (desire): {desire_average:.3f} meters.",
            throttle_duration_sec = 1,
        ) 

        self.await_lidar = False

    def on_shutdown(self):
        """
        A method to stop the robot on shutdown
        """
        for i in range(5):
            self.vel_pub.publish(Twist())
        self.shutdown = True

    def goal_callback(self, request):
        """Accept or reject a client request to begin an action."""
    
        goal_ok = True
        if request.fwd_velocity < 0 or request.fwd_velocity > 0.26:
            self.get_logger().warn(
                "Invalid fwd_velocity! Select a value between 0 and 0.26 degrees."
            )
            goal_ok = False
        
        if request.stopping_distance <= 0.2:
            self.get_logger().warn(
                "Not enough distance (stopping_distance must be greater than 0.2)."
            )
            goal_ok = False
        elif request.stopping_distance > 3.5:
            self.get_logger().warn(
                "Too much distance (maximum of 3.5 meters)."
            )
            goal_ok = False
        return GoalResponse.ACCEPT if goal_ok else GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    async def server_execution_callback(self, goal_handle):
        """Execute a goal."""
        self.get_logger().info('Executing goal...')

        result = ExploreForward.Result()
        feedback = ExploreForward.Feedback()

        fwd_vel = goal_handle.request.fwd_velocity
        stop_dist = goal_handle.request.stopping_distance

        self.get_logger().info(
            f"\n#####\n"
            f"The '{self.get_name()}' has been called.\n"
            f"Goal:\n"
            f"  - explore at {fwd_vel:.2f} m/s\n"
            f"  - stop {stop_dist:.2f} m in front of something\n" 
            f"Here we go..."
            f"\n#####\n")
        
        vel_cmd = Twist()

        turn_vel = -0.7 # rad/s

        while self.await_odom or self.await_lidar:
            continue
        dist_travelled = 0.0

        first_turn = False

        while not(goal_handle.is_cancel_requested):

            # checks overall front obstacle distance and also checks (front) left and right distances to ensure robot has turned enough away from object
            # if self.front_dist[0] > stop_dist + 0.05 and (abs(self.front_dist[1] - self.front_dist[2]) < self.front_dist[0] / 2.5):
            check = self.front_dist[0] > stop_dist + 0.3 and self.front_dist[1] > stop_dist + 0.3 and self.front_dist[2] > stop_dist + 0.3
            rat=1

            if self.front_dist[0] > stop_dist +0.1 and self.front_dist[1] > stop_dist -0.1 and self.front_dist[2] > stop_dist -0.1:

                # advance forward
                vel_cmd.linear.x = fwd_vel
                vel_cmd.angular.z = 0.02
                self.vel_pub.publish(vel_cmd)
                self.get_logger().info(
                            f"forward."
                        )

                # check if there has been a request to cancel the action:
                if goal_handle.is_cancel_requested:
                    # stop the robot:
                    for i in range(5):
                        self.vel_pub.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info(
                        f"Cancelled."
                    )
                    result.total_distance_travelled = dist_travelled
                    result.closest_obstacle = float(self.front_dist)
                    return result

                # calculate distance travelled so far
                xcoord = abs(self.x - self.xref)
                ycoord = abs(self.y - self.yref)
                self.displacement = sqrt(pow(xcoord, 2) + pow(ycoord, 2))

                # updating reference and adding to get distance rather than displacement
                self.xref = self.x
                self.yref = self.y
                dist_travelled += self.displacement

                feedback.current_distance_travelled = float(dist_travelled)
                goal_handle.publish_feedback(feedback)

            if (self.front_dist[3] > stop_dist + 0.4 and self.front_dist[3] < stop_dist + 0.9 and not self.left_turn_completed): # or (self.front_dist[1] > stop_dist + 0.2 and self.front_dist[1] < stop_dist + 0.4): 
                vel_cmd.linear.x = 0.0
                vel_cmd.angular.z = -turn_vel

                # Stop turning after 90 degrees
                initial_yaw = self.theta_z
                target_yaw = (initial_yaw - 1.5) % 360  # Turn left by 0.5 radians (~28.6 degrees)

                while abs(self.theta_z - target_yaw) > 0.05:  # Allow a small margin of error
                    self.vel_pub.publish(vel_cmd)
                    self.loop_rate.sleep()

                self.left_turn_completed = True

                # self.vel_pub.publish(vel_cmd)
                self.get_logger().info(
                        f"left spacious"
                    )

                # check if there has been a request to cancel the action:
                if goal_handle.is_cancel_requested:
                    # stop the robot:
                    for i in range(5):
                        self.vel_pub.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info(
                        f"Cancelled."
                    )
                    result.total_distance_travelled = dist_travelled
                    result.closest_obstacle = float(self.front_dist)
                    return result
                
            # turning robot left when object in front or the (front) right sensor is detecting an object close
            # elif self.front_dist[0] <= stop_dist + 0.05 or self.front_dist[2] < self.front_dist[0] / 2.5:
            elif self.front_dist[0] < stop_dist or self.front_dist[0] == float("nan"):
                
                self.left_turn_completed = False

                vel_cmd.linear.x = 0.02
                vel_cmd.angular.z = -turn_vel    

                self.vel_pub.publish(vel_cmd)
                self.get_logger().info(
                        f"turning."
                    )

                # check if there has been a request to cancel the action:
                if goal_handle.is_cancel_requested:
                    # stop the robot:
                    for i in range(5):
                        self.vel_pub.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info(
                        f"Cancelled."
                    )
                    result.total_distance_travelled = dist_travelled
                    result.closest_obstacle = float(self.front_dist)
                    return result
                
            #turning right if its too close to an object on the left
            elif self.front_dist[1] == float("nan") or self.front_dist[3] == float("nan") or self.front_dist[1] < stop_dist/rat or self.front_dist[3] < stop_dist/rat:
                vel_cmd.linear.x = 0.02
                vel_cmd.angular.z = turn_vel

                self.vel_pub.publish(vel_cmd)
                self.get_logger().info(
                        f"left too close."
                    )

                # check if there has been a request to cancel the action:
                if goal_handle.is_cancel_requested:
                    # stop the robot:
                    for i in range(5):
                        self.vel_pub.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info(
                        f"Cancelled."
                    )
                    result.total_distance_travelled = dist_travelled
                    result.closest_obstacle = float(self.front_dist)
                    return result
            
            elif self.front_dist[2] == float("nan") or self.front_dist[4] == float("nan") or self.front_dist[2] < stop_dist/rat or self.front_dist[4] < stop_dist/rat:
                vel_cmd.linear.x = 0.02
                vel_cmd.angular.z = -turn_vel

                self.vel_pub.publish(vel_cmd)
                self.get_logger().info(
                        f"right too close"
                    )

                # check if there has been a request to cancel the action:
                if goal_handle.is_cancel_requested:
                    # stop the robot:
                    for i in range(5):
                        self.vel_pub.publish(Twist())
                    goal_handle.canceled()
                    self.get_logger().info(
                        f"Cancelled."
                    )
                    result.total_distance_travelled = dist_travelled
                    result.closest_obstacle = float(self.front_dist)
                    return result
                
            feedback.current_distance_travelled = float(dist_travelled)
            goal_handle.publish_feedback(feedback)

            self.loop_rate.sleep()
            
        if goal_handle.is_cancel_requested:
                # stop the robot:
                for i in range(5):
                    self.vel_pub.publish(Twist())
                goal_handle.canceled()
                self.get_logger().info(
                    f"Cancelled."
                )
                result.total_distance_travelled = dist_travelled
                result.closest_obstacle = float(self.front_dist[0])
                return result

        for i in range(5):
            self.vel_pub.publish(Twist())

        goal_handle.succeed()

        result.total_distance_travelled = dist_travelled
        result.closest_obstacle = float(self.front_dist)
        return result


def main(args=None):
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = AdvancedExplorationServer()

    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info(
            "Starting the Server (shut down with Ctrl+C)"
        )
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "Server shut down with Ctrl+C"
        )
    finally:
        node.on_shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()