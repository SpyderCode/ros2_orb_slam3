#!/usr/bin/env python3
"""
Bridge node to connect Tello camera stream to ORB-SLAM3.
Takes real-time images from Tello and forwards them to ORB-SLAM3 in the expected format.

Only the latest frame is forwarded — older frames are dropped so ORB-SLAM3
always processes the most recent image without building up a backlog.

Author: System Integration
Date: 2025-12-21
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String
import time


class TelloOrbSlamBridge(Node):
    def __init__(self):
        super().__init__('tello_orb_slam_bridge')

        # ORB-SLAM3 typically processes at 1-3 FPS on most hardware.
        # Sending faster just builds a DDS backlog that shows up as lag.
        self.declare_parameter('target_fps', 2.0)
        self.target_fps = self.get_parameter('target_fps').value
        self.min_interval = 1.0 / self.target_fps

        # Publishers to ORB-SLAM3 — KEEP_LAST/1 so DDS never queues
        # more than one pending message (drops older undelivered frames).
        pub_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.img_pub = self.create_publisher(Image, '/mono_py_driver/img_msg', pub_qos)
        self.timestamp_pub = self.create_publisher(Float64, '/mono_py_driver/timestep_msg', pub_qos)
        self.config_pub = self.create_publisher(String, '/mono_py_driver/experiment_settings', 1)
        
        # Subscriber to acknowledgement from ORB-SLAM3 C++ node
        self.ack_sub = self.create_subscription(
            String,
            '/mono_py_driver/exp_settings_ack',
            self.ack_callback,
            10
        )
        
        # Subscriber to Tello camera — keep only 1 latest message
        # so we always process the most recent frame
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.image_sub = self.create_subscription(
            Image,
            '/tello/camera/image_rect_color',
            self.image_callback,
            img_qos
        )
        
        # State variables
        self.config_sent = False
        self.ack_received = False
        self.image_count = 0
        self.last_forward_time = 0.0
        self.last_timestamp = 0.0
        
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
        """Forward images from Tello to ORB-SLAM3, rate-limited and drop-stale."""
        if not self.ack_received:
            return

        # Rate-limit: skip if we forwarded too recently
        now = time.monotonic()
        if now - self.last_forward_time < self.min_interval:
            return

        # Compute timestamp from header
        timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        # Guard against non-monotonic timestamps (ORB-SLAM3 rejects them)
        if timestamp <= self.last_timestamp:
            timestamp = self.last_timestamp + 0.001  # nudge forward
        self.last_timestamp = timestamp
        
        # Publish timestamp first, then image
        timestep_msg = Float64()
        timestep_msg.data = timestamp
        self.timestamp_pub.publish(timestep_msg)
        
        self.img_pub.publish(msg)
        self.last_forward_time = now
        
        self.image_count += 1
        if self.image_count == 1:
            # Log diagnostics on first forward
            img_subs = self.img_pub.get_subscription_count()
            ts_subs = self.timestamp_pub.get_subscription_count()
            self.get_logger().info(
                f'First image forwarded — {msg.width}x{msg.height} {msg.encoding}, '
                f'img_sub_count={img_subs}, ts_sub_count={ts_subs}')
            if img_subs == 0:
                self.get_logger().warn(
                    'No subscribers on /mono_py_driver/img_msg! '
                    'Is ORB-SLAM3 (mono_node_cpp) running?')
        if self.image_count % 50 == 0:
            img_subs = self.img_pub.get_subscription_count()
            self.get_logger().info(
                f'Forwarded {self.image_count} images to ORB-SLAM3 '
                f'(img_sub_count={img_subs})')


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
