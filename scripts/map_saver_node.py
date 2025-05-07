#!/usr/bin/env python3
import os

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap
from ament_index_python.packages import get_package_share_directory

class MapSaverNode(Node):
    def __init__(self):
        super().__init__('map_saver_node')

        # create <pkg_share>/maps
        pkg_share = get_package_share_directory('com2009_team69_2025')
        maps_dir = os.path.join(pkg_share, 'maps')
        os.makedirs(maps_dir, exist_ok=True)
        self.get_logger().info(f"Maps directory: {maps_dir}")

        # Create client for the SaveMap service
        service_name = '/map_saver/save_map'
        self.client = self.create_client(SaveMap, service_name)
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"Waiting for service {service_name}...")

        # Build and send the request
        req = SaveMap.Request()
        req.map_topic = 'map'                              
        req.map_url = os.path.join(maps_dir, 'arena_map')  
        req.image_format = 'png'   
        req.map_mode      = 'trinary'                         

        self.get_logger().info('Requesting map save...')
        self.client.call_async(req).add_done_callback(self._on_response)

    def _on_response(self, future):
        try:
            result = future.result().result
            if result:
                self.get_logger().info('Map saved to maps/arena_map.{yaml,png}')
            else:
                self.get_logger().error('Map saving failed (server returned false)')
        except Exception as e:
            self.get_logger().error(f'Exception during SaveMap call: {e}')
        finally:
            # Gracefully exit once we’ve received the reply
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = MapSaverNode()
    rclpy.spin(node)


if __name__ == '__main__':
    main()
