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

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8MultiArray
from rcl_interfaces.msg import Log
import json
import yaml
import paho.mqtt.client as mqtt
from pathlib import Path
from ament_index_python.packages import get_package_share_directory


class Communications(Node):
    """
    Communications node for Loon-E project.
    This node handles all inter-node communication within the ROS ecosystem and also interfaces with external systems via MQTT.
    It subscribes to various topics published by other nodes (e.g., task planning, path planning, mapping, sensor data) and 
    republishes this data to corresponding MQTT topics for external monitoring and integration. 
    It also listens to the ROS log stream from /rosout and publishes log messages to MQTT for centralized logging.
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
        
        # Subscribe to all data from other nodes
        # From task_ROS.py
        self.create_subscription(UInt8MultiArray, "destination", self.destination_callback, 10)
        self.create_subscription(UInt8MultiArray, "PWM", self.pwm_callback, 10)
        
        # From path_planning_ROS.py
        self.create_subscription(UInt8MultiArray, "path", self.path_callback, 10)
        
        # From mapping_ROS.py
        self.create_subscription(UInt8MultiArray, "map", self.map_callback, 10)
        
        # From sensor/vision systems
        self.create_subscription(UInt8MultiArray, "objects", self.objects_callback, 10)
        self.create_subscription(UInt8MultiArray, "locations", self.locations_callback, 10)
        self.create_subscription(UInt8MultiArray, "obstacles", self.obstacles_callback, 10)

        # ROS logs from all nodes
        self.create_subscription(Log, "/rosout", self.rosout_callback, 100)
        
        # Store received data
        self.destination = [] # TODO: should we store this as a dict with timestamps?
        self.pwm = []
        self.path = []
        self.map = []
        self.objects = []
        self.locations = []
        self.obstacles = []
        self.rosout = {}
        
        self.get_logger().info("Communications node initialized")
    
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
    
    def destroy_node(self) -> None:
        """Cleanup on node destruction"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    coms_node = Communications()
    rclpy.spin(coms_node)
    coms_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()