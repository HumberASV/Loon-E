"""
Copyright (c) 2026 Carson Fujita. All rights reserved.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
"""

import json
import socket
from io import BytesIO
from pathlib import Path

import paho.mqtt.client as mqtt
import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Log
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import UInt8MultiArray

try:
    from PIL import Image as PILImage
    IMAGE_BRIDGE_AVAILABLE = True
    IMAGE_BRIDGE_IMPORT_ERROR = None
except Exception as exc:
    PILImage = None
    IMAGE_BRIDGE_AVAILABLE = False
    IMAGE_BRIDGE_IMPORT_ERROR = str(exc)


class Communications(Node):
    """
    Communications node for Loon-E project.
    This node handles all inter-node communication within the ROS ecosystem and also interfaces with external systems via MQTT.
    It subscribes to various topics published by other nodes (e.g., task planning, path planning, mapping, sensor data) and 
    republishes this data to corresponding MQTT topics for external monitoring and integration.
    It also subscribes to ZED camera image streams and forwards them via UDP to the Base-Station.
    It listens to the ROS log stream from /rosout and publishes log messages to MQTT for centralized logging.
    """

    def __init__(self) -> None:
        """
        Initialize the Communications node.
        Reads from the configuration file to set up MQTT connection and topics, then subscribes to all relevant ROS topics.
        """

        super().__init__("Communications")

        self.get_logger().info(f"[coms] Initializing Communications node...")

        # Load YAML configuration
        # Try to find config in installed location first, then fall back to source
        try:
            config_dir = get_package_share_directory('loon_e_coms')
            config_path = Path(config_dir) / "config" / "coms.yaml"
        except:
            # Fall back to this package's source directory
            config_path = Path(__file__).resolve().parent.parent / "config" / "coms.yaml"
        
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.get_logger().info(f"[coms] Loaded configuration from {config_path}")
        
        mqtt_config = self.config['mqtt']
        
        # Initialize MQTT client
        self.mqtt_client = mqtt.Client(mqtt_config['client']['id'])
        self.mqtt_client.on_connect = self.on_mqtt_connect
        self.mqtt_client.on_disconnect = self.on_mqtt_disconnect
        
        # Connect to MQTT broker
        try:
            self.mqtt_client.connect(
                mqtt_config['broker']['host'],
                mqtt_config['broker']['port'],
                mqtt_config['client']['keepalive']
            )
            self.mqtt_client.loop_start()
            self.get_logger().info(f"[coms] MQTT client initialized and connecting to {mqtt_config['broker']['host']}:{mqtt_config['broker']['port']}")
        except Exception as e:
            self.get_logger().error(f"[coms] Failed to connect to MQTT broker: {e}")
            # TODO: should we make a status topic?
            raise
        
        # Store MQTT topics and QoS
        self.mqtt_topics = mqtt_config['topics']
        self.mqtt_qos = mqtt_config['client']['qos']

        # UDP video streaming configuration
        video_cfg = self.config.get('video', {})
        self.video_udp_enabled = bool(video_cfg.get('enabled', True))
        self.video_udp_host = video_cfg.get('udp_host', '127.0.0.1')
        self.video_udp_port = int(video_cfg.get('udp_port', 5005))
        self.video_udp_socket = None
        self.video_jpeg_quality = int(video_cfg.get('jpeg_quality', 70))

        if self.video_udp_enabled and not IMAGE_BRIDGE_AVAILABLE:
            self.get_logger().error(
                f"[coms] Video UDP is enabled, but Pillow is unavailable: {IMAGE_BRIDGE_IMPORT_ERROR}. Disabling video forwarding."
            )
            self.video_udp_enabled = False

        if self.video_udp_enabled:
            try:
                self.video_udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                self.get_logger().info(
                    f"[coms] UDP video socket initialized: sending to {self.video_udp_host}:{self.video_udp_port}"
                )
            except Exception as e:
                self.get_logger().error(f"[coms] Failed to initialize UDP video socket: {e}")
                self.video_udp_enabled = False
        
        # Subscribe to all data from other nodes
        # From task_ROS.py
        self.create_subscription(UInt8MultiArray, "destination", self.destination_callback, 10)
        self.create_subscription(UInt8MultiArray, "PWM", self.pwm_callback, 10)
        
        # From path_planning_ROS.py
        self.create_subscription(UInt8MultiArray, "path", self.path_callback, 10)

        # Odometry stream (compatible with ZED default QoS: reliable + volatile)
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Odometry, "odom", self.odom_callback, odom_qos)
        
        # From mapping_ROS.py
        self.create_subscription(UInt8MultiArray, "map", self.map_callback, 10)
        
        # From sensor/vision systems
        self.create_subscription(UInt8MultiArray, "objects", self.objects_callback, 10)
        self.create_subscription(UInt8MultiArray, "locations", self.locations_callback, 10)
        self.create_subscription(UInt8MultiArray, "obstacles", self.obstacles_callback, 10)

        # ROS logs from all nodes
        self.create_subscription(Log, "/rosout", self.rosout_callback, 100)

        # ZED image subscriptions with best-effort QoS (compatible with ZED wrapper)
        zed_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(Image, "left_image", self.left_image_callback, zed_qos)
        self.create_subscription(Image, "right_image", self.right_image_callback, zed_qos)
        
        # Store received data
        self.destination = [] # TODO: should we store this as a dict with timestamps?
        self.pwm = []
        self.path = []
        self.odom = {}
        self.map = []
        self.objects = []
        self.locations = []
        self.obstacles = []
        self.rosout = {}
        
        self.get_logger().info("Communications node initialized")

    def _bridge_ros_image_to_jpeg(self, ros_image_msg: Image):
        """
        Convert a ROS Image message to JPEG bytes.
        Handles different encoding formats (e.g., rgb8, bgr8, mono8, etc.)
        """
        if not IMAGE_BRIDGE_AVAILABLE:
            return None

        try:
            size = (ros_image_msg.width, ros_image_msg.height)
            raw_data = bytes(ros_image_msg.data)

            if ros_image_msg.encoding == 'rgb8':
                pil_image = PILImage.frombytes('RGB', size, raw_data)
            elif ros_image_msg.encoding == 'bgr8':
                # PIL can decode BGR byte order directly using the raw decoder.
                pil_image = PILImage.frombuffer('RGB', size, raw_data, 'raw', 'BGR', 0, 1)
            elif ros_image_msg.encoding == 'mono8':
                pil_image = PILImage.frombytes('L', size, raw_data)
            else:
                self.get_logger().warning(
                    f"[coms] Unsupported image encoding '{ros_image_msg.encoding}' for JPEG bridge"
                )
                return None

            buf = BytesIO()
            pil_image.save(buf, format='JPEG', quality=self.video_jpeg_quality)
            return buf.getvalue()
        except Exception as e:
            self.get_logger().warning(f"[coms] Failed to convert ROS image to JPEG: {e}")
            return None

    def send_video_frame_udp(self, jpeg_bytes: bytes, source: str) -> None:
        """Send JPEG frame bytes via UDP to Base-Station."""
        if not self.video_udp_enabled or self.video_udp_socket is None or jpeg_bytes is None:
            return
        
        try:
            self.video_udp_socket.sendto(jpeg_bytes, (self.video_udp_host, self.video_udp_port))
            self.get_logger().debug(
                f"[coms] Sent {source} frame ({len(jpeg_bytes)} bytes) via UDP to {self.video_udp_host}:{self.video_udp_port}"
            )
        except Exception as e:
            self.get_logger().error(f"[coms] Failed to send {source} frame via UDP: {e}")
    
    def on_mqtt_connect(self, client, userdata, flags, rc) -> None:
        """Callback when MQTT client connects"""
        if rc == 0:
            self.get_logger().info("[coms] MQTT client connected successfully")
        else:
            self.get_logger().error(f"[coms] MQTT connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc) -> None:
        """Callback when MQTT client disconnects"""
        if rc != 0:
            self.get_logger().warning(f"[coms] Unexpected MQTT disconnection with code {rc}")
        else:
            self.get_logger().info("[coms] MQTT client disconnected")
    
    def publish_mqtt(self, topic_key: str, data) -> None:
        """
        Publish data to MQTT topic
        In format:
            {
            "data": <data>,
            "timestamp": <timestamp in nanoseconds>
            }
        Parameters:
            topic_key (str): Key to look up the MQTT topic from configuration
            data: The data to publish (will be JSON-encoded)
        """
        try:
            if topic_key not in self.mqtt_topics:
                self.get_logger().error(f"[coms] Topic key '{topic_key}' not found in configuration")
                return
            
            topic = self.mqtt_topics[topic_key]
            payload = json.dumps({"data": data, "timestamp": self.get_clock().now().nanoseconds})
            self.mqtt_client.publish(topic, payload, qos=self.mqtt_qos)
            self.get_logger().debug(f"[coms] Published to MQTT topic '{topic}': {payload}")
        except Exception as e:
            self.get_logger().error(f"[coms] Failed to publish to MQTT: {e}")
    
    def destination_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for destination data from task planning"""
        self.get_logger().info(f"[coms] Received destination: {msg.data}")
        self.destination = msg.data
        self.publish_mqtt("destination", list(msg.data))
    
    def pwm_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for PWM control data"""
        self.get_logger().info(f"[coms] Received PWM: {msg.data}")
        self.pwm = msg.data
        self.publish_mqtt("pwm", list(msg.data))
    
    def path_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for path planning data"""
        self.get_logger().info(f"[coms] Received path: {msg.data}")
        self.path = msg.data
        self.publish_mqtt("path", list(msg.data))
    
    def map_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for map/environment data"""
        self.get_logger().info(f"[coms] Received map: {msg.data}")
        self.map = msg.data
        self.publish_mqtt("map", list(msg.data))

    def odom_callback(self, msg: Odometry) -> None:
        """Callback for odometry data"""
        odom_payload = {
            "frame_id": msg.header.frame_id,
            "stamp": {
                "sec": msg.header.stamp.sec,
                "nanosec": msg.header.stamp.nanosec,
            },
            "position": {
                "x": msg.pose.pose.position.x,
                "y": msg.pose.pose.position.y,
                "z": msg.pose.pose.position.z,
            },
            "orientation": {
                "x": msg.pose.pose.orientation.x,
                "y": msg.pose.pose.orientation.y,
                "z": msg.pose.pose.orientation.z,
                "w": msg.pose.pose.orientation.w,
            },
        }
        self.get_logger().info(
            "[coms] Received odometry in '%s' frame: X: %.2f Y: %.2f Z: %.2f - Ts: %u.%u sec"
            % (
                odom_payload["frame_id"],
                odom_payload["position"]["x"],
                odom_payload["position"]["y"],
                odom_payload["position"]["z"],
                odom_payload["stamp"]["sec"],
                odom_payload["stamp"]["nanosec"],
            )
        )
        self.odom = odom_payload
        self.publish_mqtt("odom", odom_payload)
    
    def objects_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for detected objects from vision system"""
        self.get_logger().info(f"[coms] Received objects: {msg.data}")
        self.objects = msg.data
        self.publish_mqtt("objects", list(msg.data))
    
    def locations_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for object locations"""
        self.get_logger().info(f"[coms] Received locations: {msg.data}")
        self.locations = msg.data
        self.publish_mqtt("locations", list(msg.data))
    
    def obstacles_callback(self, msg: UInt8MultiArray) -> None:
        """Callback for obstacle data"""
        self.get_logger().info(f"[coms] Received obstacles: {msg.data}")
        self.obstacles = msg.data
        self.publish_mqtt("obstacles", list(msg.data))

    def rosout_callback(self, msg: Log) -> None:
        """Callback for ROS log stream from /rosout"""
        log_payload = {
            "stamp": {
                "sec": msg.stamp.sec,
                "nanosec": msg.stamp.nanosec,
            },
            "level": msg.level,
            "name": msg.name,
            "msg": msg.msg,
            "file": msg.file,
            "function": msg.function,
            "line": msg.line,
        }
        self.rosout = log_payload
        self.publish_mqtt("rosout", log_payload)

    def left_image_callback(self, msg: Image) -> None:
        """Callback for left rectified image from ZED"""
        self.get_logger().info(
            f"[coms] Received left image from ZED\tSize: {msg.width}x{msg.height} - Ts: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec"
        )
        jpeg_bytes = self._bridge_ros_image_to_jpeg(msg)
        self.send_video_frame_udp(jpeg_bytes, "left")

    def right_image_callback(self, msg: Image) -> None:
        """Callback for right rectified image from ZED"""
        self.get_logger().info(
            f"[coms] Received right image from ZED\tSize: {msg.width}x{msg.height} - Ts: {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec"
        )
        jpeg_bytes = self._bridge_ros_image_to_jpeg(msg)
        self.send_video_frame_udp(jpeg_bytes, "right")
    
    def destroy_node(self) -> None:
        """Cleanup on node destruction"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        if self.video_udp_socket is not None:
            self.video_udp_socket.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    coms_node = Communications()
    rclpy.spin(coms_node)
    coms_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()