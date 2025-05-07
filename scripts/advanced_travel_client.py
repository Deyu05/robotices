#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient 

from com2009_team69_2025.action import ExploreForward
from rclpy.signals import SignalHandlerOptions

class ExploreActionClient(Node):

    def __init__(self):
        super().__init__("explore_client") 
        self.actionclient = ActionClient(
            node=self, 
            action_type=ExploreForward, 
            action_name="explore_forward"
        ) 
        self.goal_succeeded = False
        self.goal_cancelled = False
        self.stop = False

    # default values of 0.1 m/s and 1.0 metres
    def send_goal(self, fwd_vel=0.25, dist_stop=0.4): 
        goal = ExploreForward.Goal()
        goal.fwd_velocity = fwd_vel
        goal.stopping_distance = dist_stop

        self.actionclient.wait_for_server()

        # send the goal to the action server:
        self.send_goal_future = self.actionclient.send_goal_async(
            goal=goal, 
            feedback_callback=self.feedback_callback
        )
        self.send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("The goal was rejected by the server.")
            rclpy.shutdown()
            return

        self.get_logger().info("The goal was accepted by the server.")

        self.get_result_future = goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self.get_result_callback)
        self._goal_handle = goal_handle
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"The action has completed.\n"
            f"Result:\n"
            f"  - Total distance = {result.total_distance_travelled}"
            f"  - Closest object = {result.closest_obstacle}"
        )
        self.goal_succeeded = True
        rclpy.shutdown()
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        fdbk_current_distance = feedback.current_distance_travelled
        self.get_logger().info(
            f"\nFEEDBACK:\n"
            f"  - Current distance = {fdbk_current_distance:.1f} metres."
        )
        if self.stop:
            future = self._goal_handle.cancel_goal_async()
            future.add_done_callback(self.cancel_goal)
        


    def cancel_goal(self, future):
        cancel_response = future.result()   
        if len(cancel_response.goals_canceling) > 0:
            self.get_logger().info('Goal successfully canceled')
            self.goal_cancelled = True
        else:
            self.get_logger().info('Goal failed to cancel')

def main(args=None): 
    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )
    action_client = ExploreActionClient()
    future = action_client.send_goal()
    while not action_client.goal_succeeded:
        try:
            rclpy.spin_once(action_client)
            if action_client.goal_cancelled:
                break
        except KeyboardInterrupt:
            print("Ctrl+C")
            action_client.stop = True

if __name__ == '__main__':
    main()