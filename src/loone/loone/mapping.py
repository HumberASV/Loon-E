import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from zed_msgs.msg import ObjectsStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import threading

class Mapping(Node):
    """ A ROS2 node that manages the mapping of the environment based on data from various sources.
    The node subscribes to topics providing information about detected objects and the phone's position,
    and publishes the global map and the current position of the robot.
    """

    def __init__(self):
        """ Initialize the Mapping node and set up publishers and subscribers. """
        super().__init__('Map_PubSub')

        # Event used to block startup until the first phone update arrives.
        self.phone_data_ready_event = threading.Event()
        
        #Publishers and Subscribers
        self.global_pub = self.create_publisher(OccupancyGrid, 'global', 10)
        self.current_position_pub = self.create_publisher(PoseStamped, 'position', 10)
        self.objects_sub = self.create_subscription(ObjectsStamped, 'objects', self.object_callback, 10)
        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback, 10)

        # Declare parameters with fallback/default values
        self.declare_parameter('local_w', 20)
        self.declare_parameter('local_l', 20)
        self.declare_parameter('global_w', 50)
        self.declare_parameter('global_l', 50)
        self.declare_parameter('res', 0.5)
        self.declare_parameter('start_position', 0)

        # Retrieve parameters
        self.local_w = self.get_parameter('local_w').value
        self.local_l = self.get_parameter('local_l').value
        self.global_w = self.get_parameter('global_w').value
        self.global_l = self.get_parameter('global_l').value
        self.res      = self.get_parameter('res').value
        start_position = self.get_parameter('start_position').value

        #Other internal variables
        self.local_rows = self.get_cell(self.local_w)
        self.local_cols = self.get_cell(self.local_l)
        self.global_rows = self.get_cell(self.global_w)
        self.global_cols = self.get_cell(self.global_l)
        self.local_map = np.zeros((self.local_rows, self.local_cols), dtype = np.int8)
        self.global_map = np.zeros((self.global_rows, self.global_cols), dtype = np.int8)

        match start_position:
            case 0: # Origin
                self.global_position = [round(self.global_rows*0.5), round(self.global_cols*0.5)]
            case 0.5: # Positive Y axis
                self.global_position = [round(self.global_rows*0.25), round(self.global_cols*0.5)]
            case 1: # Quadrant 1
                self.global_position = [round(self.global_rows*0.25), round(self.global_cols*0.75)]
            case 1.5: # Positive X axis
                self.global_position = [round(self.global_rows*0.5), round(self.global_cols*0.75)]
            case 2: # Quadrant 2
                self.global_position = [round(self.global_rows*0.75), round(self.global_cols*0.75)]
            case 2.5: # Negative Y axis
                self.global_position = [round(self.global_rows*0.75), round(self.global_cols*0.5)]
            case 3: # Quadrant 3
                self.global_position = [round(self.global_rows*0.75), round(self.global_cols*0.25)]
            case 3.5: # Negative X axis
                self.global_position = [round(self.global_rows*0.5), round(self.global_cols*0.25)]
            case 4: # Quadrant 4
                self.global_position = [round(self.global_rows*0.25), round(self.global_cols*0.25)]

        #Other variables from topics
        # Keep the ZED object list and the derived local positions separate so
        # the map-building code can consume stable per-object coordinates.
        self.objects = []
        self.locations = []
        self.current_position = self.global_position
        self.previous_position = [np.nan, np.nan]
        self.heading = np.nan

        # Spin until data is received
        self.get_logger().info('waiting for phone data...')
        while not self.phone_data_ready_event.is_set():
            rclpy.spin_once(self, timeout_sec = 0.1)
        self.get_logger().info('phone data received, starting mapping node.')

    #ROS - Publish
    def publish_map(self) -> None:
        """ Publish the global map as an Int8MultiArray message. """
        msg = OccupancyGrid()
        msg.info.resolution = self.res
        msg.info.height, msg.info.width = self.global_map.shape
        msg.data = self.global_map.flatten().tolist()

        self.global_pub.publish(msg)
        self.get_logger().info(f"Map: {msg.data}")

    def publish_position(self) -> None:
        """ Publish the current position as a PoseStamped message. """
        # The position publisher is typed as PoseStamped, so publish the
        # current coordinates in that matching message type.
        # We only have heading, so we publish a yaw-only quaternion and assume
        # roll and pitch are zero.
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(self.global_position[1])
        msg.pose.position.y = float(self.global_position[0])
        msg.pose.position.z = 0.0 # assumed
        if np.isnan(self.heading):
            # If heading is not yet available, fall back to the identity quaternion.
            msg.pose.orientation.w = 1.0
        else:
            yaw = math.radians(self.heading)
            msg.pose.orientation.x = 0.0
            msg.pose.orientation.y = 0.0
            msg.pose.orientation.z = math.sin(yaw / 2.0)
            msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.current_position_pub.publish(msg)
        self.get_logger().info(f"Position: ({msg.pose.position.x}, {msg.pose.position.y})")
    
    #General Code
    def get_cell(self, meter: float) -> int: #convert units in meters to units in cells
        """
        Convert a distance in meters to the corresponding number of grid cells based on the map resolution.

        Args:
            meter (float): The distance in meters to be converted.
        returns:
            int: The equivalent number of grid cells, rounded to the nearest whole number.
        """
        cell = round(meter / self.res)

        return cell
    
    def get_map_cell(self, start: list, end: list) -> None: #Get heading from two coordinates
        """
        Calculate the change in global position based on two coordinates, converting the distance to grid cells.

        Args:
            start (list): The starting coordinate as a list [y, x].
            end (list): The ending coordinate as a list [y, x].
        """
        #Possible edit: Use haversine formula instead
        lat = math.radians((start[1] + end[1]) / 2)
        # NOTE: Why is lat calculated as the average of start[1] and end[1]? 
        # It seems like it should be the average of the latitudes (start[0] and end[0]) instead. 
        # This might be a bug. ~ Carson
        y = self.get_cell((start[0] - end[0]) * 111000)
        x = self.get_cell((start[1] - end[1]) * 111000 * math.cos(lat))
        self.global_position[0] += y
        self.global_position[1] += x
    
    def get_global_map(self) -> None:
        """ Update the global map based on the local map and the current position and heading. """
        self.get_map_cell(self.previous_position, self.current_position)
        mid = round((self.global_cols - 1) / 2)
        for i in range(self.global_rows):
            for j in range(self.global_cols):
                if mid - i <= j <= mid + i: #Do not include excess (Camera has 90 degree FOV)
                    #Convert cartesian to polar
                    local_x = mid - j
                    local_y = i
                    r = math.sqrt(local_y**2 + local_x**2)
                    theta = math.atan(local_x, local_y)
                    if theta < 0: #Quadrant IV
                        theta += 2*math.pi()

                    #Find matching point in local map
                    theta += math.radians(self.heading)
                    global_x = round(r*math.sin(theta)) + self.global_position[0]
                    global_y = round(r*math.cos(theta)) + self.global_position[1]
                    self.global_map[global_y][global_x] = self.local_map[i][j]
        
        self.publish_map()

    def get_local_map(self) -> None:
        """ Update the local map based on detected obstacles and their locations. """
        self.local_map = np.zeros((self.local_rows, self.local_cols), dtype = np.int8)

        # Each ZED detection is approximated as a small occupied patch in the local grid.
        for i, obstacle in enumerate(self.objects):
            location = self.locations[i]

            match obstacle: #Values rounded up to nearest whole cell length, assuming resolution of 0.5 m
                case _:
                    # ZED object detections do not guarantee a footprint here,
                    # so we treat each object as a small occupied cell by default.
                    objL = 0.5
                    objW = 0.5

            # Convert world units to grid cells
            objL = self.get_cell(objL)
            objW = self.get_cell(objW)

            # X and Y coordinates of the object in the local map
            objStartY = self.get_cell(location[0])
            objStartX = self.get_cell(location[1]) 

            # Write into the matrix
            for i in range(objL):
                for j in range(objW):
                    self.local_map[objStartY + j, objStartX + i] = 1
        
        if (self.current_position is not None and not np.isnan(self.current_position[0])) and (self.heading is not None and not np.isnan(self.heading)):
            self.get_global_map()
    
    #ROS - Subscribe
    def object_callback(self, msg: ObjectsStamped) -> None:
        """ 
        Callback function for the objects subscription. Updates the detected objects. 
        
        Args:
            msg (ObjectsStamped): The ZED message received from the objects topic, containing detected objects.
        """
        # Store the full ZED detections and derive local map coordinates from
        # each object's reported 3D position.
        self.get_logger().info(f"Objects: {msg.objects}")
        self.objects = msg.objects
        self.locations = [[obj.position[1], obj.position[0]] for obj in self.objects]
        self.get_local_map()

    def phone_callback(self, msg: Float32MultiArray) -> None:
        """Handle incoming phone telemetry and update current position, speed, and heading.

        Args:
            msg: Float32MultiArray where
                index 0 is latitude
                index 1 is longitude
                index 2 is speed
                index 3 is heading.
        """
        data = msg.data
        self.get_logger().info(f"Phone: {msg.data}")
        if data[0] != self.previous_position[0] or data[1] != self.previous_position[1]:
            self.previous_position = self.current_position
            self.current_position = [data[0], data[1]]
        self.heading = data[3]
        self.phone_data_ready_event.set() # Unblocks the init sequence

    
def main(args = None):
    """ Main function to initialize the ROS2 node and start spinning. """
    rclpy.init(args = args)
    mapping = Mapping()
    try:
        rclpy.spin(mapping)
    except KeyboardInterrupt:
        mapping.get_logger().info("Mapping node interrupted by user.")
    finally:
        mapping.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
