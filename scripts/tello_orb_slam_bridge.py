#!/usr/bin/env python3
"""
Bridge node to connect Tello camera stream to ORB-SLAM3.
Takes real-time images from Tello and forwards them to ORB-SLAM3 in the expected format.

Author: System Integration
Date: 2025-12-21
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
import time


class TelloOrbSlamBridge(Node):
    def __init__(self):
        super().__init__('tello_orb_slam_bridge')
        
        # Publishers to ORB-SLAM3 (expected topics from mono_driver_node.py example)
        self.img_pub = self.create_publisher(Image, '/mono_py_driver/img_msg', 1)
        self.timestamp_pub = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', 1)
        self.config_pub = self.create_publisher(String, '/mono_py_driver/experiment_settings', 1)
        
        # Subscriber to acknowledgement from ORB-SLAM3 C++ node
        self.ack_sub = self.create_subscription(
            String,
            '/mono_py_driver/exp_settings_ack',
            self.ack_callback,
            10
        )
        
        # Subscriber to Tello camera
        self.image_sub = self.create_subscription(
            Image,
            '/tello/camera/image_rect_color',
            self.image_callback,
            10
        )
        
        # State variables
        self.config_sent = False
        self.ack_received = False
        self.image_count = 0
        
        # Send configuration once on startup
        self.config_timer = self.create_timer(0.1, self.send_config_callback)
        
        self.get_logger().info('Tello-ORB-SLAM3 bridge initialized')
        self.get_logger().info('Waiting for configuration acknowledgement...')
    
    def send_config_callback(self):
        """Send configuration to ORB-SLAM3 until acknowledged"""
        if not self.ack_received:
            config_msg = String()
            config_msg.data = "TelloMono"  # Configuration name for Tello mono camera
            self.config_pub.publish(config_msg)
            
            if not self.config_sent:
                self.get_logger().info('Sending configuration: TelloMono')
                self.config_sent = True
        else:
            # Stop sending once acknowledged
            self.config_timer.cancel()
    
    def ack_callback(self, msg):
        """Handle acknowledgement from ORB-SLAM3"""
        if msg.data == "ACK" and not self.ack_received:
            self.ack_received = True
            self.get_logger().info('Configuration acknowledged by ORB-SLAM3')
            self.get_logger().info('Starting to forward camera images...')
    
    def image_callback(self, msg):
        """Forward images from Tello to ORB-SLAM3"""
        if not self.ack_received:
            return
        
        # Use message timestamp as the timestep (in seconds)
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Publish timestamp first
        timestep_msg = Float64()
        timestep_msg.data = timestamp
        self.timestamp_pub.publish(timestep_msg)
        
        # Then publish image
        self.img_pub.publish(msg)
        
        self.image_count += 1
        if self.image_count % 50 == 0:
            self.get_logger().info(f'Forwarded {self.image_count} images to ORB-SLAM3')


def main(args=None):
    rclpy.init(args=args)
    bridge = TelloOrbSlamBridge()
    
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
