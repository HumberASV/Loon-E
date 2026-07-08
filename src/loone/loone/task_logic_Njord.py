import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8MultiArray, MultiArrayDimension
import numpy as np
import time
import math

#Fill with object IDs from model
GREEN = 0
RED = 1
NORTH = 2
EAST = 3
SOUTH = 4
WEST = 5
OTTER = 6

class Task(Node):
    """
    A ROS2 node that publishes task commands to various topics.
    """

    def __init__(self):
        """ Initialize the Task node and set up the publisher and timer. """
        super().__init__('Task_PubSub')
        self.path_pub = self.create_publisher(Int8MultiArray, 'Task_Path', 10)
        self.motor_pub = self.create_publisher(Float32MultiArray, 'Task_Motor', 10)
        self.objects_sub = self.create_subscription(Float32MultiArray, 'Objects', self.object_callback(), 10)
        self.phone_sub = self.create_subscription(Float32MultiArray, 'Phone', self.phone_callback(), 10)
        self.position_sub = self.create_subscription(Int8MultiArray, 'Position', self.position_callback(), 10)
        self.path_sub = self.create_subscription(Int8MultiArray, 'Path', self.path_callback(), 10)
        
        #Declare parameters with fallback/default values
        self.declare_parameter('local_l', 20)
        self.declare_parameter('res', 0.5)
        self.declare_parameter('task', [])
        self.declare_parameter('latitude', [])
        self.declare_parameter('longitude', [])

        # Retrieve Parameters
        self.local_w = self.get_parameter('local_w').value
        self.res = self.get_parameter('res').value
        self.task = self.get_parameter('task').integer_array_value
        latitude = self.get_parameter('latitude').double_array_value
        longitude = self.get_parameter('longitude').double_array_value

        self.i = 0 # Current target GPS coordinate
        self.stage = 0 # Current stage in task
        self.get_path(latitude, longitude)

        self.position = [np.nan, np.nan] # Current GPS position
        self.heading = np.nan # Current Heading
        self.objects = [np.nan] # Current objects in view
        self.locations = [np.nan] # Current position of objects
        self.global_position = [np.nan, np.nan] # Current position in global map
        self.waypoints = [np.nan] # Path to destination

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
                self.path[i] = [latitude[i], longitude[i]]

    #ROS - Publish
    def publish_path(self, data):
        """ Publish the path command as a Int8MultiArray message.
        
        Args:
            data: Message content in form [global position, target position]
        """
        msg = Int8MultiArray()
        msg.data = data
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

        Returns True if object found and False if not found.
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

        Returns True if condition met and False if not.
        """
        change_stage = False
        if self.object_found(target_objects) == found: #If change condition met
            change_stage = True
        
        return change_stage

    def arrived(self, i):
        """ Checks if the Loon-E is at the desired GPS point.
        
        Args:
            i: Desired index for GPS point in list provided by self.path.

        Returns True if Loon-E at point and False if not.
        """
        arrived = False
        if self.position[0] == self.latitude[i] and self.position[1] == self.longitude[i]:
            arrived = True

        return arrived

    def check_heading(self, target_heading):
        """Checks if Loon-E is within 45 degrees of target angle.
        Turns toward target angle if this is not the case.
        
        Args:
            target_heading: Desired heading.
        
        Returns True if Loon-E within 45 degrees of target angle and False if not.
        """
        good_heading = True
        d_heading = self.heading - target_heading

        if d_heading > 180: #Wrap to [-180, 180]
            d_heading -= 360
        elif d_heading < -180:
            d_heading += 360

        if abs(d_heading) > 45:
            data = [2, np.nan, np.nan, np.sign(d_heading)]
            self.publish_motor(data)
            good_heading = False
            
        return good_heading
    
    def get_map_cell(self, start, end): #Get heading from two coordinates
        #Possible edit: Use haversine formula instead
        lat = math.radians((start[1] + end[1]) / 2)
        y = round((start[0] - end[0]) * 111000 / self.res)
        x = round((start[1] - end[1]) * 111000 * math.cos(lat) / self.res)
        target_position = [self.global_position[0] + y, self.global_position[1] + x]
        heading = math.degrees(math.atan2(x, y))
        
        return target_position, heading
    
    def get_coordinate(self, current_position, waypoints): #Get position and heading from coordinate and relative position in map
        #Possible edit: Use haversine formula instead
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

    def drive(self):
        target_position = self.get_map_cell(self.position, self.path[self.i])[0]
        data = [self.global_position, target_position]
        self.publish_path(data)
        
        if self.waypoints is not None:
            target_heading = self.get_coordinate(self.position, self.waypoints)[1]
            data = [1.0, target_heading, 2.0, np.nan]
            self.publish_motor(data)

    def shutdown(self):
        """Stops motor at end of task or if KeyboardInterrupt"""
        data = [0.0, np.nan, np.nan, np.nan]
        self.publish_motor(data)
    
    #General Code - Tasks
    def task_0(self): #For PID testing
        data = [1.0, 0.0, self.speed, 1.0, np.nan]
        self.publish_motor(data)
    
    def task_1(self): #Maneuvering and Path Finding
        match self.stage:
            case 0: #Turn to face nearest waypoint
                dist = 0

                if self.path[self.i] != [0, 0]: #Task 1b, if marker not found
                    target_heading = self.get_map_cell(self.path[self.i - 1], self.path[self.i])[1]
                else:
                    target_heading = self.get_map_cell(self.path[self.i - 1], self.path[self.i + 1])[1]

                if self.check_heading(target_heading):
                    if self.path[self.i] == [0, 0]: #Task 1b, if marker not found
                        loc_buoy = (0, len(self.map))
                        loc_marker = (0, len(self.map))

                        for i in range(len(self.image_objects)): #Search for closest object of each type
                            if self.image_objects[i] == (RED or GREEN) and self.locations[i][1] < loc_buoy[1]:
                                loc_buoy = self.locations[i]
                            elif self.image_objects[i] == EAST and self.locations[i][1] < loc_marker[1]:
                                loc_marker = self.locations[i]
                                dist = 3
                            elif self.image_objects[i] == WEST and self.locations[i][1] < loc_marker[1]:
                                loc_marker = self.locations[i]
                                dist = -3
                        
                        if dist != 0: #If marker was detected
                            coord = self.get_coordinate(loc_buoy[0], loc_buoy[1] + dist)[0]
                            self.path[self.i] = coord

                    self.stage = 10
            
            case 10: #Path plan and drive
                self.drive()
                
                if self.arrived(self.i):
                    self.i += 1
                    self.stage = 0
                    if self.i == len(self.path): #Final waypoint
                        self.shutdown()

    def task_2(self): #Collision Avoidance
        margin = 0.4
        match self.stage:
            case 0: #Go to destination, checking for Otter
                self.drive()

                if self.check_change(OTTER, True):
                    for i in self.objects: #Find otter in objects to get its position
                        if i == OTTER:
                            index = i
                            break

                    #check x position of otter
                    if self.locations[index][0] < self.local_w * margin:
                        self.stage = 20
                    elif self.location[index][0] > self.local_w * (1 - margin):
                        self.stage = 12
                    else:
                        loc_buoy = (0, len(self.map))

                        for i in range(len(self.image_objects)): #Search for closest object of each type
                            if self.image_objects[i] == RED and self.locations[i][1] < loc_buoy[1]:
                                loc_buoy = self.locations[i]
                        
                        coord = self.get_coordinate(loc_buoy[0], loc_buoy[1] + 3)[0]
                        self.path.insert(coord, 1)

                        self.stage = 11

            case 11: #If head on, turn to avoid
                self.drive()
                
                if self.arrived(self.i):
                    self.i += 1
                    self.stage = 20

            case 12: #If right of boat, wait until Otter passes
                data = [0.0, np.nan, np.nan, np.nan]
                self.publish_motor(data)
                
                if self.check_change(OTTER, False):
                    self.stage = 20
            
            case 20: #Go to destination
                self.drive()
                
                if self.arrived(self.i):
                    self.shutdown()

    def task_3(self): #Docking
        
        return
    
    def task_4(self): #Surprise Task

        return

    def run_task(self):
        match self.task:
            case 0: self.task_0()
            case 1: self.task_1()
            case 2: self.task_2()
            case 3: self.task_3()
            case 4: self.task_4()
    
    #ROS - Subscribe
    def object_callback(self, msg):
        data = msg.data
        self.get_logger().info(f"Objects: {msg.data}")
        self.objects = data[0]
        self.locations = data[1]
        self.get_local_map()

    def phone_callback(self, msg):
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
        self.position = [data[0], data[1]]
        self.speed = data[2]
        self.heading = data[3]
    
    def position_callback(self, msg):
        """Handle incoming data and update current position in global map.

        Args:
            msg: Int8MultiArray where
                index 0 is vertical position in global map
                index 1 is horizontal position in global map
        """
        data = msg.data
        self.get_logger().info(f"Position: {msg.data}")
        self.global_position = [data[0], data[1]]
    
    def path_callback(self, msg):
        """Handle incoming data and update current waypoint list.

        Args:
            msg: Int8MultiArray of waypoints.
        """
        data = msg.data
        self.get_logger().info(f"Path: {msg.data}")
        self.waypoints = data

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