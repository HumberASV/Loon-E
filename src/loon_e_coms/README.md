# Communications Node

Communications node for Loon-E project.
This node handles all inter-node communication within the ROS ecosystem and also interfaces with external systems via MQTT.
It subscribes to various topics published by other nodes (e.g., task planning, path planning, mapping, sensor data) and 
republishes this data to corresponding MQTT topics for external monitoring and integration. 
It also listens to the ROS log stream from /rosout and publishes log messages to MQTT for centralized logging.

Data is published in the following format:

```json
{
"data": <data>,
"timestamp": <timestamp in nanoseconds>
}
```

## Requirements

This communications node requires that you modify the [config](./config/coms.yaml) to the proper MQTT broker. You'll need to set up a MQTT broker like [Mosquitto](https://mosquitto.org/)

### Example Install and Run

```
sudo apt install mosquitto mosquitto-clients -y
echo -e "listener 1883\nallow_anonymous true" >> mosquitto.conf
mosquitto -c ~/mosquitto.conf
```