import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs import UInt8MultiArray
import json
import yaml
import paho.mqtt.client as mqtt
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class Communications(Node):
    def __init__(self):
        super().__init__("Communications")
        
        # Load YAML configuration
        # Try to find config in installed location first, then fall back to source
        try:
            config_dir = get_package_share_directory('loon-e-coms')
            config_path = Path(config_dir) / "config" / "coms.yaml"
        except:
            # Fall back to source directory
            config_path = Path(__file__).parent.parent.parent / "config" / "coms.yaml"
        
        with open(config_path, 'r') as config_file:
            self.config = yaml.safe_load(config_file)
        
        self.get_logger().info(f"Loaded configuration from {config_path}")
        
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
            self.get_logger().info(f"MQTT client initialized and connecting to {mqtt_config['broker']['host']}:{mqtt_config['broker']['port']}")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to MQTT broker: {e}")
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
        
        # Store received data
        self.destination = []
        self.pwm = []
        self.path = []
        self.map = []
        self.objects = []
        self.locations = []
        self.obstacles = []
        
        self.get_logger().info("Communications node initialized")
    
    def on_mqtt_connect(self, client, userdata, flags, rc):
        """Callback when MQTT client connects"""
        if rc == 0:
            self.get_logger().info("MQTT client connected successfully")
        else:
            self.get_logger().error(f"MQTT connection failed with code {rc}")
    
    def on_mqtt_disconnect(self, client, userdata, rc):
        """Callback when MQTT client disconnects"""
        if rc != 0:
            self.get_logger().warn(f"Unexpected MQTT disconnection with code {rc}")
        else:
            self.get_logger().info("MQTT client disconnected")
    
    def publish_mqtt(self, topic_key: str, data):
        """Publish data to MQTT topic"""
        try:
            if topic_key not in self.mqtt_topics:
                self.get_logger().error(f"Topic key '{topic_key}' not found in configuration")
                return
            
            topic = self.mqtt_topics[topic_key]
            payload = json.dumps({"data": list(data), "timestamp": self.get_clock().now().nanoseconds})
            self.mqtt_client.publish(topic, payload, qos=self.mqtt_qos)
            self.get_logger().debug(f"Published to MQTT topic '{topic}': {payload}")
        except Exception as e:
            self.get_logger().error(f"Failed to publish to MQTT: {e}")
    
    def destination_callback(self, msg: UInt8MultiArray):
        """Callback for destination data from task planning"""
        self.get_logger().info(f"Received destination: {msg.data}")
        self.destination = msg.data
        self.publish_mqtt("destination", msg.data)
    
    def pwm_callback(self, msg: UInt8MultiArray):
        """Callback for PWM control data"""
        self.get_logger().info(f"Received PWM: {msg.data}")
        self.pwm = msg.data
        self.publish_mqtt("pwm", msg.data)
    
    def path_callback(self, msg: UInt8MultiArray):
        """Callback for path planning data"""
        self.get_logger().info(f"Received path: {msg.data}")
        self.path = msg.data
        self.publish_mqtt("path", msg.data)
    
    def map_callback(self, msg: UInt8MultiArray):
        """Callback for map/environment data"""
        self.get_logger().info(f"Received map: {msg.data}")
        self.map = msg.data
        self.publish_mqtt("map", msg.data)
    
    def objects_callback(self, msg: UInt8MultiArray):
        """Callback for detected objects from vision system"""
        self.get_logger().info(f"Received objects: {msg.data}")
        self.objects = msg.data
        self.publish_mqtt("objects", msg.data)
    
    def locations_callback(self, msg: UInt8MultiArray):
        """Callback for object locations"""
        self.get_logger().info(f"Received locations: {msg.data}")
        self.locations = msg.data
        self.publish_mqtt("locations", msg.data)
    
    def obstacles_callback(self, msg: UInt8MultiArray):
        """Callback for obstacle data"""
        self.get_logger().info(f"Received obstacles: {msg.data}")
        self.obstacles = msg.data
        self.publish_mqtt("obstacles", msg.data)
    
    def destroy_node(self):
        """Cleanup on node destruction"""
        self.mqtt_client.loop_stop()
        self.mqtt_client.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    coms_node = Communications()
    rclpy.spin(coms_node)
    coms_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()