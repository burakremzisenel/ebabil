#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist

class SafetyStop(Node):
    def __init__(self):
        super().__init__('safety_stop')

        # Topics / Parameter
        self.declare_parameter('topic_depth', '/cam_front/camera/depth/image_rect_raw')
        self.declare_parameter('topic_info',  '/cam_front/camera/depth/camera_info')
        self.declare_parameter('input_cmd',   '/cmd_vel_raw')
        self.declare_parameter('output_cmd',  '/cmd_vel')

        self.declare_parameter('stop_distance_m', 1.0)
        self.declare_parameter('release_distance_m', 1.2)  # Hysterese: erst >1.2 m wieder frei
        self.declare_parameter('roi_ratio', 0.25)
        self.declare_parameter('percentile', 30.0)

        # Entprellung / Hold
        self.declare_parameter('min_block_frames', 3)   # N Frames < stop bis BLOCKED
        self.declare_parameter('min_clear_frames', 3)   # N Frames > release bis CLEAR
        self.declare_parameter('hold_ms', 300)          # nach STOP mind. so lange 0 schicken

        # Lesen
        self.topic_depth = self.get_parameter('topic_depth').value
        self.topic_info  = self.get_parameter('topic_info').value
        self.input_cmd   = self.get_parameter('input_cmd').value
        self.output_cmd  = self.get_parameter('output_cmd').value

        self.stop_d     = float(self.get_parameter('stop_distance_m').value)
        self.rel_d      = float(self.get_parameter('release_distance_m').value)
        self.roi_ratio  = float(self.get_parameter('roi_ratio').value)
        self.percentile = float(self.get_parameter('percentile').value)

        self.min_block  = int(self.get_parameter('min_block_frames').value)
        self.min_clear  = int(self.get_parameter('min_clear_frames').value)
        self.hold_ms    = int(self.get_parameter('hold_ms').value)

        # I/O
        self.create_subscription(CameraInfo, self.topic_info,  self._on_info, 10)
        self.create_subscription(Image,      self.topic_depth, self._on_depth,10)
        self.create_subscription(Twist,      self.input_cmd,   self._on_cmd,  10)
        self.pub_cmd = self.create_publisher(Twist, self.output_cmd, 10)

        # State
        self.w = self.h = None
        self.state = 'CLEAR'      # 'CLEAR' | 'BLOCKING' | 'BLOCKED' | 'CLEARING'
        self.block_count = 0
        self.clear_count = 0
        self.last_stop_time = 0.0
        self.last_cmd = Twist()
        self.last_dmin = None
        self.last_log = 0.0

    def _on_info(self, msg: CameraInfo):
        self.w, self.h = msg.width, msg.height

    def _depth_to_m(self, msg: Image) -> np.ndarray:
        if msg.encoding in ('16UC1', 'mono16'):
            arr = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) / 1000.0
        else:  # e.g. 32FC1
            arr = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        return arr

    def _on_depth(self, msg: Image):
        if self.w is None or self.h is None:
            return
        arr = self._depth_to_m(msg)

        # Zentrales ROI
        rw = max(8, int(self.w * self.roi_ratio))
        rh = max(8, int(self.h * self.roi_ratio))
        x0 = (self.w - rw) // 2
        y0 = (self.h - rh) // 2
        roi = arr[y0:y0+rh, x0:x0+rw]

        # gültige Tiefen
        roi = roi[np.isfinite(roi)]
        roi = roi[(roi > 0.1) & (roi < 15.0)]
        if roi.size == 0:
            # Keine Info -> nicht freigeben, aber auch nicht aggressiv blocken
            self.last_dmin = None
            return

        dmin = float(np.percentile(roi, self.percentile))
        self.last_dmin = dmin

        # State-Maschine mit Hysterese + Entprellung
        if dmin < self.stop_d:
            self.block_count += 1
            self.clear_count = 0
            if self.block_count >= self.min_block:
                if self.state != 'BLOCKED':
                    self.state = 'BLOCKED'
                    self.last_stop_time = time.time()
        elif dmin > self.rel_d:
            self.clear_count += 1
            self.block_count = 0
            if self.clear_count >= self.min_clear:
                if self.state != 'CLEAR':
                    self.state = 'CLEAR'
        else:
            # im Hysterese-Bereich: Zähler nicht vorantreiben
            self.block_count = 0
            self.clear_count = 0

        # Ausgabe: wenn BLOCKED oder innerhalb Hold -> STOP
        now = time.time()
        in_hold = (now - self.last_stop_time) * 1000.0 < self.hold_ms
        if self.state == 'BLOCKED' or in_hold:
            self.pub_cmd.publish(Twist())
        else:
            # Wenn gerade CLEAR und es kam schon ein Upstream-Cmd, reiche durch
            if self.last_cmd is not None:
                self.pub_cmd.publish(self.last_cmd)

        # gedrosseltes Debug
        if now - self.last_log > 0.5:
            self.get_logger().info(
                f"dmin={dmin:.2f}m state={self.state} hold={in_hold} "
                f"blk={self.block_count}/{self.min_block} clr={self.clear_count}/{self.min_clear}"
            )
            self.last_log = now

    def _on_cmd(self, msg: Twist):
        self.last_cmd = msg
        # sofortige Weitergabe nur wenn CLEAR (Depth-Callback hält STOP ansonsten)
        if self.state == 'CLEAR':
            self.pub_cmd.publish(msg)

def main():
    rclpy.init()
    node = SafetyStop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
