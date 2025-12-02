#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import base64
import requests
import time


class VLMClientNode(Node):
    def __init__(self):
        super().__init__("vlm_client_node")

        self.bridge = CvBridge()
        self.latest_image = None

        # ✔ STATIC PROMPT (YOUR CHOICE)
        self.static_prompt = (
            "Describe the scene clearly. Identify obstacles, walkable areas, "
            "and hazards in JSON: {\"obstacles\": [], \"walkable\": [], \"notes\": \"\" }"
        )

        # ✔ LOCAL vLLM server URL
        self.api_url = "http://localhost:8000/v1/chat/completions"

        # ✔ Throttle: 1 request/sec
        self.last_request = 0
        self.min_interval = 1.0

        # ✔ ROS subscriptions + publishers
        self.image_sub = self.create_subscription(
            Image, "/camera/rgb/image_raw", self.image_callback, 10
        )

        self.response_pub = self.create_publisher(String, "/vlm_response", 10)

        self.get_logger().info("🚀 Image VLM Node Started (Static Prompt Mode)")

    def image_callback(self, msg):
        """Convert ROS image -> CV2 + trigger VLM call"""
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            now = time.time()
            if now - self.last_request >= self.min_interval:
                self.last_request = now
                self.process_vlm()

        except Exception as e:
            self.get_logger().error(f"Image conversion error: {e}")

    def process_vlm(self):
        if self.latest_image is None:
            return

        try:
            # Encode to base64
            _, buf = cv2.imencode(".jpg", self.latest_image)
            img_b64 = base64.b64encode(buf).decode("utf-8")

            payload = {
                "model": "Qwen/Qwen2.5-VL-7B-Instruct",
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": self.static_prompt},
                            {"type": "image_url",
                             "image_url": {"url": f"data:image/jpeg;base64,{img_b64}"}},
                        ],
                    }
                ],
                "temperature": 0.2,
                "max_tokens": 200,
            }

            headers = {"Content-Type": "application/json"}

            response = requests.post(self.api_url, json=payload, headers=headers, timeout=60)

            if response.status_code != 200:
                self.get_logger().error(f"API error: {response.text}")
                return

            output = response.json()["choices"][0]["message"]["content"]

            ros_msg = String()
            ros_msg.data = output
            self.response_pub.publish(ros_msg)

            self.get_logger().info(f"🤖 VLM Output Published: {output}")

        except Exception as e:
            self.get_logger().error(f"VLM request error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = VLMClientNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutdown requested.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
