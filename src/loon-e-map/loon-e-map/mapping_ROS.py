import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs import UInt8MultiArray

class Mapper(Node):
    def __init__(self):
        super().__init__("Path_Planner")
        self.publisher_ = self.create_publisher(UInt8MultiArray, "map", 10)
        self.create_subscription(UInt8MultiArray, "obstacles", self.obstacle_callback, 10)
        self.create_subscription(UInt8MultiArray, "locations", self.location_callback, 10)
        period = 0.5
        self.timer = self.create_timer(period, self.publish)
        
        self.map_width = 20
        self.map_length = 20
        self.resolution = 0.25
        
        self.objects = []
        self.locations = []
        self.map = []
        
    def getCell(self, meter):
        cell = int(meter/self.resolution)

        return cell

    def addObject(self, obstacle, location):
        """
        Adds a rectangular object (marked as 1s) into the given matrix.
        startX, startY, length, width are in world units.
        """
        match obstacle: #Values rounded to nearest cell value
            case 1 | 2 | 3 | 6 | 8 | 9:
                obj_length = 0.5
                obj_width = 0.5
            case 4 | 5 | 7:
                obj_length = 0.25
                obj_width = 0.25

        # Convert world units to grid cells
        obj_length = self.getCell(obj_length)
        obj_width = self.getCell(obj_width)

        objStartX = self.getCell(location[0])
        objStartY = self.getCell(location[1])

        # Write into the matrix
        for i in range(obj_length):
            for j in range(obj_width):
                self.map[objStartY + j, objStartX + i] = obstacle

    def create_map(self):
        rows = self.getCell(self.map_width)
        cols = self.getCell(self.map_length)

        self.map = np.zeros((rows, cols))
        for i in range(len(self.obstacles)):
            self.map = self.addObject(self, self.obstacles[i], self.locations[i])
    
    def publish(self):
        msg = UInt8MultiArray()
        msg.data = self.create_map(self, self.obstacles, self.locations)
        self.publisher_.publish(msg)
    
    def obstacle_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.obstacles = msg.data
    
    def location_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.locations = msg.data

def main(args=None):
    rclpy.init(args=args)
    mapperNode = Mapper()
    rclpy.spin(mapperNode)
    mapperNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()