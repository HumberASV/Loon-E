import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray, MultiArrayDimension
from geometry_msgs.msg import Polygon
import numpy as np
import math

class Mapping(Node):
    """ A ROS2 node that manages the mapping of the environment based on data from various sources.
    The node subscribes to topics providing information about detected objects and the phone's position,
    and publishes the global map and the current position of the robot.
    """

    def __init__(self):
        """ Initialize the Mapping node and set up publishers and subscribers. """
        super().__init__('Map_PubSub')

        #Publishers and Subscribers
        self.global_pub = self.create_publisher(Int8MultiArray, 'global', 10)
        self.current_position_pub = self.create_publisher(Float32MultiArray, 'position', 10)
        self.objects_sub = self.create_subscription(Int8MultiArray, 'objects', self.object_callback, 10)
        self.locations_sub = self.create_subscription(Polygon, 'locations', self.location_callback, 10)
        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback, 10)

        # Declare parameters with fallback/default values
        self.declare_parameter('local_w', 20)
        self.declare_parameter('local_l', 20)
        self.declare_parameter('global_w', 50)
        self.declare_parameter('global_l', 50)
        self.declare_parameter('res', 0.5)
        self.declare_parameter('start_position', 4)

        # Retrieve parameters
        self.local_w = self.get_parameter('local_w').value
        self.local_l = self.get_parameter('local_l').value
        self.global_w = self.get_parameter('global_w').value
        self.global_l = self.get_parameter('global_l').value
        self.res      = self.get_parameter('res').value
        start_position = self.get_parameter('start_position').value

        self.local_rows = self.get_cell(self.local_w)
        self.local_cols = self.get_cell(self.local_l)
        self.global_rows = self.get_cell(self.global_w)
        self.global_cols = self.get_cell(self.global_l)
        self.local_map = np.zeros((0))
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
        self.objects = None
        self.locations = None
        self.current_position = self.global_position
        self.previous_position = [None, None]
        self.heading = None

    #ROS - Publish
    def publish_map(self) -> None:
        """ Publish the global map as an Int8MultiArray message. """
        rows, cols, objects = self.global_map.shape
        
        msg = Int8MultiArray()
        msg.layout.dim = [
            MultiArrayDimension(label="rows", size = rows, stride = rows * cols * objects),
            MultiArrayDimension(label="columns", size = cols, stride = cols * objects),
            MultiArrayDimension(label="objects", size = objects, stride = objects)
        ]
        msg.data = self.global_map.flatten().tolist()

        self.global_pub.publish(msg)
        self.get_logger().info(f"Map: {msg.data}")

    def publish_position(self) -> None:
        """ Publish the current position as a Int8MultiArray message. """
        msg = Int8MultiArray()
        msg.data = self.global_position
        self.current_position_pub.publish(msg)
        self.get_logger().info(f"Position: {msg.data}")
    
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

        for i in range(len(self.obstacles)):
            obstacle = self.obstacles[i]
            location = self.locations[i]

            match obstacle: #Values rounded up to nearest whole cell length, assuming resolution of 0.5 m
                case _:
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
                    self.map[objStartY + j, objStartX + i] = obstacle
        
        if self.current_position is not None and self.heading is not None:
            self.get_global_map()
    
    #ROS - Subscribe
    def object_callback(self, msg: Int8MultiArray) -> None:
        """ 
        Callback function for the objects subscription. Updates the detected objects. 
        
        Args:
            msg (Int8MultiArray): The message received from the objects topic, containing detected objects.
        """
        self.get_logger().info(f"Objects: {msg.data}")
        self.objects = msg.data
        self.get_local_map()
    
    def location_callback(self, msg: Polygon) -> None:
        """Callback function for the locations subscription. Updates the list of detected objects' locations.
        
        Args:
            msg (Polygon): The message received from the locations topic, containing detected objects' locations.
        """
        self.get_logger().info(f"Locations: {msg.data}")
        self.locations = [[p.y, p.x] for p in msg.points]
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
