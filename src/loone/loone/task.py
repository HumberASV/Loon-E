import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray
from geometry_msgs.msg import Polygon, Point32, PoseStamped
from nav_msgs.msg import OccupancyGrid
from zed_msgs.msg import ObjectsStamped
import threading
import numpy as np
import time
import math
import board
import digitalio

class Task(Node):
    """
    A ROS2 node that acts as the central control node for the Loon-E.
    
    Receives:
        List of visible objects from Camera Node
        Telemetry data from Phone Node
        Position in global map from Mapping Node
        List of waypoints from Path Planning Node
    
    Sends:
        Current and target global map positions to Path Planning Node
        Motor commands (type of action, target heading, target speed, direction) to Motor Node
    """

    def __init__(self):
        """ Initialize the Task node and set up the publisher and timer. """
        super().__init__('Task_PubSub')

        #Events
        self.object_data_ready_event = threading.Event()
        self.phone_data_ready_event = threading.Event()
        self.position_data_ready_event = threading.Event()
        self.map_data_ready_event = threading.Event()
        self.path_data_ready_event = threading.Event()

        #Publishers and Subscribers
        self.path_pub = self.create_publisher(Polygon, 'task_path', 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'task_motor', 10)
        # ZED now provides both object IDs and object positions, so the task
        # node no longer needs the separate locations topic.
        self.objects_sub = self.create_subscription(ObjectsStamped, '/zedx/zed_node/obj_det/objects', self.object_callback, 10)
        self.phone_sub = self.create_subscription(Float32MultiArray, 'phone', self.phone_callback, 10)
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.position_sub = self.create_subscription(PoseStamped, '/pose', self.position_callback, 10)
        self.path_sub = self.create_subscription(Polygon, 'path', self.path_callback, 10)
        
        #Declare parameters with fallback/default values
        self.declare_parameter('task', [1])
        self.declare_parameter('latitude', [63.4420522, 63.442353, 63.4424279])
        self.declare_parameter('longitude', [10.4239289, 10.4237452, 10.4243540])
        self.declare_parameter('camera_fov', 20)
        
        # Retrieve Parameters
        self.task = self.get_parameter('task').value
        latitude = self.get_parameter('latitude').value
        longitude = self.get_parameter('longitude').value
        self.get_path(latitude, longitude) # Merge lists
        self.camera_fov = self.get_parameter('camera_fov').value

        self.auto_pin = digitalio.DigitalInOut(board.D12)
        self.auto_pin.direction = digitalio.Direction.INPUT
        self.estop_pin = digitalio.DigitalInOut(board.D13)
        self.estop_pin.direcftion = digitalio.Direction.INPUT

        #Other internal variables
        self.i = 0 # Current target GPS coordinate
        self.stage = 0 # Current stage in task

        #Other variables from topics
        self.position = np.array([[-999, -999]]) # Current GPS position
        self.heading = -999 # Current Heading
        self.objects = [-999] # Current objects in view
        self.locations = [[-999, -999]] # Current position of objects in [x, y, z]
        self.res = -999
        self.origin = np.array([-999, -999])
        self.global_position = np.array([0, 0]) # Current position in global map
        self.waypoints = np.array([[-999, -999]]) # Path to destination
        self.buoy_list = []
        self.target_heading = -999

        #Thread
        self.receiver = threading.Thread(target = self.run_task, daemon = True)
        self.receiver.start()

    def get_path(self, latitude, longitude):
        """ Setup function which converts list of latitude and list of longitude into list of [latitude, longitude].
        
        Args:
            latitude: List of latitudes
            longitude: List of longitudes
        
        Raises:
            ValueError if latitude and longitude lists are not of same size
        """
        self.path = []
        
        if len(latitude) != len(longitude):
            self.get_logger().error("Latitude and longitude lists are not of same size")
            raise

        else:
            for i in range (len(latitude)):
                self.path.append([latitude[i], longitude[i]])

    #ROS - Publish
    def publish_path(self, data):
        """ Publish the path command as a Int8MultiArray message.
        
        Args:
            data: Message content in form [global position, target position]
        """
        msg = Polygon()

        for position in data:
            point = Point32()
            point.y = position[0]
            point.x = position[1]
            msg.points.append(point)
        
        self.path_pub.publish(msg)
        self.get_logger().info(f"Path: {msg.data}")
    
    def publish_motor(self, data):
        """ Publish the task command as a Float32MultiArray message.
        
        Args:
            data: Message content in form [command, target heading, target speed, direction].
            Where not applicable the value will be replaced with np.nan.
        """
        msg = Float32MultiArray()
        msg.data = data
        self.motor_pub.publish(msg)
        self.get_logger().info(f"Motor: {msg.data}")
    
    #General Code - Subprograms
    def object_found(self, target_objects):
        """ Checks if an object is in the current image frame.
        
        Args:
            target_objects: Object to identify

        Returns True if object found and False otherwise.
        """
        found = True
        for object in target_objects:
            if object not in self.objects: #if object not in image
                found = False
                break

        return found

    def check_change(self, target_objects, found):
        """ Checks if the Loon-E should move to the next sub-task based on current image data.
        
        Args:
            target_objects: Object to identify
            found: If the object should be in the image (True) or not (False).

        Returns True if condition met and False otherwise.
        """
        change_stage = False
        if self.object_found(target_objects) == found: #If change condition met
            change_stage = True
        
        return change_stage

    def arrived(self, i):
        """ Checks if the Loon-E is at the desired GPS point.
        
        Args:
            i: Desired index for GPS point in list provided by self.path.

        Returns True if Loon-E at point and False otherwise.
        """
        arrived = False
        if self.position[0] == self.latitude[i] and self.position[1] == self.longitude[i]:
            arrived = True

        return arrived

    def check_heading(self):
        """Checks if Loon-E is within 45 degrees of target angle.
        Turns toward target angle if this is not the case.
        
        Args:
            target_heading: Desired heading.
        
        Returns True if Loon-E within 45 degrees of target angle and False otherwise.
        """
        good_heading = True
        d_heading = self.convert(self.convert(self.heading) - self.target_heading)

        if d_heading > 180: #Wrap to [-180, 180]
            d_heading -= 360
        elif d_heading < -180:
            d_heading += 360

        if abs(d_heading) > 45:
            data = [2.0, -999.0, np.sign(d_heading)]
            self.publish_motor(data)
            good_heading = False
            
        return good_heading
    
    def get_map_cell(self, start, end):
        """Get heading from two coordinates.

        Args:
            start: Current GPS coordinate.
            end: Target GPS coordinate.
        
        Returns:
            target_position: Target coordinate in global map
            heading: Heading of the coordinate relative to North
        Possible edit: Use haversine formula instead
        """
        lat = math.radians((start[1] + end[1]) / 2)
        y = round((start[0] - end[0]) * 111000 / self.res)
        x = round((start[1] - end[1]) * 111000 * math.cos(lat) / self.res)
        target_position = [self.global_position[0] + y, self.global_position[1] + x]
        
        return target_position
    
    def get_coordinate(self, current_position, waypoints):
        """Get position and heading from coordinate and relative position in map.
        Args:
            current_position: Current GPS coordinate
            waypoints: Global map points representing path to destination
        
        Returns:
            target_position: GPS coordinate of target position
            heading: Heading of coordinate relative to North
        
        Possible edit: Use haversine formula instead
        """
        lat = current_position[0]
        long = current_position[1]
        x = (waypoints[1][0] - waypoints[0][0]) * self.res
        y = (waypoints[1][1] - waypoints[0][1]) * self.res
        heading = math.atan2(x,y)

        d = math.sqrt(x**2 + y**2)
        d_lat = d * math.cos(heading) / 111000
        d_long = d * math.sin(heading) / (111000 * math.cos(math.radians(lat)))

        target_position = [lat + d_lat, long + d_long]
        heading = math.degrees(heading)

        return target_position, heading

    def buoy_in_list(self, coord):
        check = False
        for buoy in self.buoy_list:
            if buoy[0] - 0.00001 < coord < buoy[0] + 0.00001 and buoy[1] - 0.00002 < coord[1] < buoy[1] + 0.00002:
                check = True
                break

        return check

    def drive(self):
        """Self-driving control loop"""
        #Calculate target position in global map
        target_position = self.get_map_cell(self.position, self.path[self.i])
        data = [self.global_position, target_position]
        self.publish_path(data)
        
        #Wait for path planning to be completed
        while not self.path_data_ready_event.is_set():
            rclpy.spin_once(self, timeout_sec = 0.1)
        self.path_data_ready_event.clear()

        #Send command to Motor Node
        if self.waypoints != []:
            target_heading = self.get_coordinate(self.position, self.waypoints)[1]
            data = [1.0, target_heading, -999.0]
            self.publish_motor(data)    

    def shutdown(self):
        """Stops motor at end of task or if KeyboardInterrupt"""
        data = [0.0, -999.0, -999.0]
        self.publish_motor(data)
    
    #General Code - Tasks
    def task_0(self):
        """For PID testing"""
        data = [1.0, 170.0, -999.0]
        self.publish_motor(data)
    
    def task_1(self):
        """Maneuvering and Path Finding"""
        match self.stage:
            case 0: #Drive
                buoy_type = 0
                buoy_loc = (0, 0, len(self.map))
                self.drive()

                for i in range(len(self.image_objects)): #Search for closest object of each type
                    if self.image_objects[i] == 'red' and self.locations[i][0] < buoy_loc[0]:
                        buoy_type = self.image_objects[i]
                        buoy_loc = self.locations[i]

                    elif self.image_objects[i] == 'green' and self.locations[i][0] < buoy_loc[0]:
                        buoy_type = self.image_objects[i]
                        buoy_loc = self.locations[i]
                
                if buoy_type != 0: #If marker was detected
                    waypoints = [self.global_position, (buoy_loc[0], buoy_loc[1])]
                    coord = self.get_coordinate(self.position, waypoints)[0]
                    if not self.buoy_in_list(coord):
                        if buoy_type == 'red':
                            coord[1] += 0.00008
                        else:
                            coord[1] -= 0.00008
                        self.path[self.i] = coord
                        self.buoy_list.append(coord)
                
                if self.arrived(self.i):
                    self.i += 1
                    self.stage = 0
                    if self.i == len(self.path): #Final waypoint
                        self.shutdown()

    def task_2(self):
        """Collision Avoidance"""
        margin = 0.4
        match self.stage:
            case 0: #Go to destination, checking for Otter
                self.drive()

                if self.check_change('otter', True):
                    for i in self.objects: #Find otter in objects to get its position
                        if i == 'otter':
                            index = i
                            break

                    #check x position of otter
                    if self.locations[index][1] > self.camera_fov * margin:
                        self.stage = 20
                    elif self.location[index][1] < self.camera_fov * (1 - margin):
                        self.stage = 12
                    else:
                        buoy_loc = (0, len(self.map))

                        for i in range(len(self.image_objects)): #Search for closest object of each type
                            if self.image_objects[i] == 'red' and self.locations[i][0] < buoy_loc[0]:
                                buoy_loc = self.locations[i]

                        waypoints = [self.global_position, (buoy_loc[0], buoy_loc[1] + 3)]
                        coord = self.get_coordinate(self.position, waypoints)[0]
                        self.path.insert(coord, 1)

                        self.stage = 11

            case 11: #If head on, turn to avoid
                self.drive()
                
                if self.arrived(self.i):
                    self.i += 1
                    self.stage = 20

            case 12: #If right of boat, wait until Otter passes
                data = [0.0, -999.0, -999.0]
                self.publish_motor(data)
                
                if self.check_change('otter', False):
                    self.stage = 20
            
            case 20: #Go to destination
                self.drive()
                
                if self.arrived(self.i):
                    self.shutdown()

    def task_3a(self):
        """Forward Docking"""
        match self.stage:
            case 0:
                self.drive()

                if self.arrived(self.i):
                    self.i += 1
                    #Scan AR tag and place two new GPS points
                    self.stage = 10
            
            case 10:
                self.drive()

                if self.arrived(self.i):
                    self.i += 1
                    time.sleep(10)
                    data = [-1.0, -999.0, -999.0]
                    self.publish_motor(data)
                    time.sleep(10)

                    self.stage = 20

            case 20:
                self.drive()

                if self.arrived(self.i):
                    self.shutdown()
                
        return

    def task_3b(self):
        """Parallel Docking"""
        match self.stage:
            case 0:
                self.drive()

                if self.arrived(self.i):
                    #Scan AR tag and place GPS point
                    self.stage = 10
        
            case 10:
                self.drive()

                if self.arrived(self.i):
                    self.target_heading = self.current_heading + 90
                    self.stage = 20

            case 20:
                if self.check_heading():
                    self.target_heading = self.current_heading + 90
                    time.sleep(5)
                    self.stage = 30

            case 30:
                if self.check_heading():
                    self.stage = 40

            case 40:
                self.drive()

                if self.arrived(self.i):
                    self.shutdown()

        return
    
    def task_4(self):
        """Surprise Task"""

        return

    def run_task(self):
        """Wait to receive data, execute task when data received, and clear event status when task executed"""
        while True:
            #Spin until data is received
            while not (self.object_data_ready_event.is_set() # True if new data received
                       and self.phone_data_ready_event.is_set() # True if new data received
                       and self.position_data_ready_event.is_set() # True if new data received
                       and not self.auto_pin.value # True if boat in RC mode
                       and not self.estop_pin.value): # True if estop on
                rclpy.spin_once(self, timeout_sec = 0.1)
            self.get_logger().info("Ready!")

            match self.task:
                case 0: self.task_0()
                case 1: self.task_1()
                case 2: self.task_2()
                case 3.1: self.task_3a()
                case 3.2: self.task_3b()
                case 4: self.task_4()
            
            #Clear to wait until next set of data is received - May cause problem
            self.object_data_ready_event.clear()
            self.phone_data_ready_event.clear()
            self.position_data_ready_event.clear()
    
    #ROS - Subscribe
    def object_callback(self, msg: ObjectsStamped) -> None:
        """ 
        Callback function for the objects subscription. Updates the detected objects. 
        
        Args:
            msg (ObjectsStamped): The ZED message received from the objects topic, containing detected objects.
        """
        # Store ZED label IDs for task logic and keep each object's reported
        # position alongside it for the stage logic that reasons about location.
        self.objects = [obj.label for obj in msg.objects]
        self.locations = [[obj.position[0], obj.position[1]] for obj in msg.objects]
        self.get_logger().info(f"Objects: {self.objects}")
        self.get_logger().info(f"Locations: {self.locations}")
        self.object_data_ready_event.set()

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
        self.get_logger().info(f"Phone: {data}")
        self.position = [data[0], data[1]]
        self.speed = data[2]
        self.heading = data[3]
        self.phone_data_ready_event.set()
    
    def map_callback(self, msg: OccupancyGrid) -> None:
        self.get_logger().info("Map data received")
        self.res = msg.info.resolution
        self.origin[0] = msg.info.origin.position.y
        self.origin[1] = msg.info.origin.position.x
        self.map_data_ready_event.set()
    
    def position_callback(self, msg: PoseStamped) -> None:
        """Handle incoming data and update current position in global map.

        Args:
            msg: Int8MultiArray where
                index 0 is vertical position in global map
                index 1 is horizontal position in global map
        """
        data = msg.pose
        self.get_logger().info(f"Pose data received")
        if self.map_data_ready_event:
            self.global_position[0] = (data.position.y - self.origin[0]) / self.res
            self.global_position[1] = (data.position.x - self.origin[1]) / self.res
            self.get_logger().info(f"Position: {self.global_position}")
        
        self.position_data_ready_event.set()
    
    def path_callback(self, msg: Polygon) -> None:
        """Handle incoming data and update current waypoint list.

        Args:
            msg: Polygon of waypoints.
        """
        # Polygon paths carry waypoints in points, not data.
        self.get_logger().info(f"Path: {msg.points}")
        self.waypoints = [[p.y, p.x] for p in msg.points]
        self.path_data_ready_event.set()

def main(args = None):
    """ Main function to initialize the ROS2 node and start spinning. """
    rclpy.init(args = args)
    task = Task()
    try:
        rclpy.spin(task)
    except KeyboardInterrupt:
        task.get_logger().info("Task node interrupted by user.")
    finally:
        task.shutdown()
        task.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()