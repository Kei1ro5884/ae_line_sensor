#!/usr/bin/env python3
"""
line_sensor_node.py

Gazebo で 8×1 px カメラ（topic: /line_cam/image_raw）から取得した
モノクロ画像を、フレームごとに自動計算したしきい値で 0/1 の
ビット列へ変換して `/ae_line/raw` (std_msgs/UInt8MultiArray) に公開する
ROS 2 ノード。

■ しきい値の計算
    thresh = (min(pixels) + max(pixels)) // 2
    bits   = (pixels < thresh) ? 1 : 0
→ 環境光や床材質のムラに即時に適応する。
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray
from cv_bridge import CvBridge
import numpy as np


class LineSensorNode(Node):
    def __init__(self) -> None:
        super().__init__('line_sensor_node')

        self._bridge = CvBridge()

        # 出力: 8 bit の 0/1 配列
        self._pub = self.create_publisher(
            UInt8MultiArray, '/ae_line/raw', 10)

        # 入力: Gazebo カメラ画像 (1×8, mono8)
        self.create_subscription(
            Image, '/line_cam/image_raw', self._on_image, 10)

        self.get_logger().info('ae_line_sensor node started')

    # ────────────────────────────────────────────────
    # 画像 → ビット列
    # ────────────────────────────────────────────────
    def _on_image(self, msg: Image) -> None:
        # Gazebo Image → NumPy uint8, shape = (8,)
        pixels = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')[0]

        # フレームごとに自動しきい値
        thresh = (int(pixels.min()) + int(pixels.max())) // 2
        bits   = (pixels < thresh).astype(np.uint8)

        # Publish
        self._pub.publish(UInt8MultiArray(data=bits.tolist()))


# ──────────────────────────────────────────────────
def main() -> None:
    rclpy.init()
    node = LineSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

