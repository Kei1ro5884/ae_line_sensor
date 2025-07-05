#!/usr/bin/env python3
"""
8×1-pixel カメラの輝度を 0/1 にして
/ae_line/raw (std_msgs/UInt8MultiArray, 要素8) で配信
"""

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge

THRESH = 80        # 黒ラインより大きく、白床より小さく調整

class LineSensorNode(Node):
    def __init__(self):
        super().__init__('ae_line_sensor')
        self.bridge = CvBridge()

        # 8 ビット配列を配信
        self.pub = self.create_publisher(UInt8MultiArray,
                                         '/ae_line/raw', 10)

        self.create_subscription(Image,
                                 '/line_cam/image_raw',
                                 self.cb_image, 10)

    def cb_image(self, msg: Image):
        # 1) 画像を 8×1 (mono8) へ
        pixels = self.bridge.imgmsg_to_cv2(msg, 'mono8')[0, :]  # shape (1,8)

        # 2) 閾値判定 → 0/1 配列
        binary = (pixels < THRESH).astype(np.uint8)             # 黒=1, 白=0

        # 3) 配信
        self.pub.publish(UInt8MultiArray(data=binary.tolist()))

def main():
    rclpy.init()
    node = LineSensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

