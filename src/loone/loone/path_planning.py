import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
from geometry_msgs.msg import Polygon, Point32
import numpy as np

class Path(Node):
    """
    A ROS2 node that subscribes to a global map and task path, performs path planning
    to avoid obstacles, and publishes the resulting waypoints to a topic.
    The node listens for incoming map and task data, generates a path that avoids obstacles,
    and publishes a Int8MultiArray message containing the waypoints at a regular interval.
    """

    def __init__(self):
        """ Initialize the Path node and set up the publisher and subscriptions. """
        super().__init__('Path_PubSub')

        #Publishers and Subscribers
        self.path_pub = self.create_publisher(Polygon, 'path', 10)
        self.map_sub = self.create_subscription(Int8MultiArray, 'global', self.map_callback(), 10)
        self.task_sub = self.create_subscription(Polygon, 'task_path', self.task_callback(), 10)

        # Declare parameters with fallback/default values
        self.declare_parameter('dist', 4)
        self.declare_parameter('radius', 2)

        # Retrieve parameters and initialize attributes
        self.path = [] #straight line path
        self.path_obstacles = [] #path avoiding obstacles
        self.waypoints = [] #self.path_obstacles with reduced resolution, based on self.dist
        self.dist = self.get_parameter('dist').value #interval between waypoints, in cells
        self.radius = self.get_parameter('radius').value #min distance from obstacles, in cells

        #Other variables from topics
        self.map = []
        self.x_start = np.nan
        self.y_start = np.nan
        self.x_end = np.nan
        self.y_end = np.nan

    #ROS - Publish
    def publish_path(self):
        """ Publish the waypoints as a Polygon message. """
        msg = Polygon()

        for position in self.waypoints:
            point = Point32()
            point.y = position[0]
            point.x = position[1]
            msg.points.append(point)
        
        self.path_pub.publish(msg)
        self.get_logger().info(f"{msg.data}")

    #General Code
    def point_in_map(self, position):
        """ Check if a given position is within the bounds of the map.
        Args:
            position (tuple): The (y, x) coordinates of the point to check.
        Returns:
            bool: True if the point is outside the map, False otherwise
        """
        rows = len(self.map)
        cols = len(self.map[0])
        output = False
        
        if 0 <= position[0] < cols and 0 <= position[1] < rows: #If desired area to search is within map bounds
            output = True
        
        return output

    def find_obstacle(self, point: tuple) -> bool:
        """ 
        Check if there is an obstacle within a certain radius of a given point. 
        Args:
            point (tuple): The (y, x) coordinates of the point to check.
        Returns:
            bool: True if there is an obstacle within the radius, False otherwise.
        """
        output = False
        y = point[0]
        x = point[1]
        
        for i in range (y - self.radius, y + self.radius + 1):
            for j in range (x - self.radius, x + self.radius + 1):
                position = (i, j)
                if self.point_in_map(position) and (self.map[i][j] != 0): #If obstacle in range
                    output = True
                    break
        
        return output

    def pathfind(self, start: tuple, expanded: list) -> list:
        """
        Find the path from the end position to the start position using the expanded nodes.

        Args:
            start (tuple): The starting position (y, x).
            expanded (list): A list of expanded nodes in the form [cost, current position, previous position].
        
        Returns:
            list: A list of positions representing the path from start to end.
        """
        parent_list = []
        
        if len(expanded)!=0: #If expanded is empty
            position=expanded[-1][1] #End node
            previous=expanded[-1][2] #Parent of end node
            while (previous!=start): #If parent is not start:
                parent_list.append(position) #Add node to path
                for point in expanded: #Find parent in expanded
                    if (point[1]==previous):
                        position=expanded[expanded.index(point)][1] #Parent node = new end node
                        previous=expanded[expanded.index(point)][2] #Parent of new end node
                        break        
            parent_list.append(position) #Add start node to path
            
        return parent_list

    def check_neighbours(self, start: tuple, end: tuple, position: tuple, unexpanded: list, expanded: list, checked: list) -> None:
        """ 
        Check the neighboring cells of the current position and add them to the unexpanded list if they are valid. 

        Args:
            start (tuple): The starting position (y, x).
            end (tuple): The ending position (y, x).
            position (tuple): The current position (y, x).
            unexpanded (list): A list of unexpanded nodes in the form [cost, current position, previous position].
            expanded (list): A list of expanded nodes in the form [cost, current position, previous position].
            checked (list): A list of checked positions.
        Returns:
            None
        """
        y = position[0]
        x = position[1]

        # Define the directions to check (up, down, right, left, and diagonals)
        # TODO: Amelia, can we just make this a constant ~ Carson
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)] #Up, down, right, left, diagonal
        
        for i, j in dir:
            next = (y + i, x + j)
            if self.point_in_map(next) and not (next in checked or self.find_obstacle(next)): #If in map, not checked, and not near obstacle
                manhattan = abs(next[0] - end[0]) + abs(next[1] - end[1])
                cost = manhattan + len(self.pathfind(start, expanded))
                unexpanded.append([cost, next, position])
                checked.append(next)
            
    def update_waypoints(self, start: tuple, parent_list: list) -> None:
        """
        Update the waypoints in the path to avoid obstacles.

        Args:
            start (tuple): The starting position (y, x).
            parent_list (list): A list of positions representing the path from start to end.

        Returns:
            None
        """
        location = self.path_obstacles.index(start) + 1 #Find point in path list
        
        for point in parent_list:
            self.path_obstacles.insert(location, point) #Add waypoints from obstacle avoidance
        self.path_obstacles.remove(parent_list[0])

    def avoid_obstacle(self, point: tuple) -> None:
        """
        Avoid an obstacle by finding a new path between the previous and next waypoints.

        Args:
            point (tuple): The position (y, x) of the waypoint near the obstacle.
        Returns:
            None
        """
        unexpanded=[] #Evaluated neighbours in form [cost, current position, previous position]
        expanded=[] #Visited in form [cost, current position, previous position]
        checked=[] #Checked cells in form [position]
        
        start = self.path[self.path.index(point) - 1]
        end_index = self.path.index(point)
        end = self.path[end_index]
        position = (start[0], start[1])

        while self.find_obstacle(end) and (end_index != len(self.path_obstacles) - 1): #move destination further away so it does not intersect with an obstacle
            self.path.remove(end)
            end = self.path[end_index]
        
        while position != end:
            self.check_neighbours(start, end, position, unexpanded, expanded, checked)
            
            if(unexpanded == []): #If no more nodes to search
                break

            next = min(unexpanded) #Select node with lowest cost
            expanded.append(next) #Move node to expanded list
            unexpanded.remove(next) #Remove node from unexpanded 
            position = expanded[-1][1] #Set to expand selected node on repeat

        parent_list = self.pathfind(start, expanded)
        self.update_waypoints(start, parent_list)
        
    def make_waypoints(self) -> None:
        """
        Create waypoints along the path, checking for points that are in range of obstacles.
        """
        self.path_obstacles = self.path

        for point in self.path:
            if self.find_obstacle(point): #If waypoint near obstacle, generate new path between previous and next waypoints
                self.avoid_obstacle(point)
                
        for point in self.path_obstacles:
            y = point[0]
            x = point[1]
            if ((self.path_obstacles.index(point) % self.dist == 0) or (point == self.path_obstacles[-1])): #Add every d points to waypoint list and last waypoint
                self.waypoints.append(point)

    def get_path(self) -> None:
        """
        Generate the path from start to end, not avoiding obstacles.
        """
        while self.find_obstacle((self.y_end, self.x_end)): #Ensure that end point is not near obstacle
            self.y_end = self.y_end - 1 #UPDATE
        
        dx = self.x_end - self.x_start #change in x
        dy = self.y_end - self.y_start #change in y

        if dx == 0: #vertical line
            for y in range(self.y_start, self.y_end + 1, np.sign(self.y_end - self.y_start)):
                self.path.append((y, self.x_start))
        elif dy == 0: #horizontal line
            for x in range(self.x_start, self.x_end, np.sign(self.x_end - self.x_start)):
                self.path.append((self.y_end, x))
        elif dy <= dx or -1 < dy / dx < 0: #Typical program
            m = dy / dx #Calculate slope
            b = self.y_start - m * self.x_start #Calculate y intercept
            for x in range(self.x_start, self.x_end + np.sign(dx), np.sign(dx)):
                y = round(m * x + b) #Find nearest y value
                self.path.append((y, x)) #Add point to path
        else: #Switch x and y so that m <= 1
            m = dx / dy #Calculate slope
            b = self.x_start - m * self.y_start #calculate x intercept
            for y in range(self.y_start, self.y_end + np.sign(dy), np.sign(dy)):
                x = round(m * y + b) #Find nearest x value
                self.path.append((y, x)) #Add point to path
        
        self.make_waypoints()
        self.publish_path()
    
    #ROS - Subscribe
    def map_callback(self, msg) -> None:
        """ Callback function for the map subscription. Updates the internal map representation. """
        self.get_logger().info(f"Map: {msg.data}")
        shape = [d.size for d in msg.layout.dim]
        self.map = np.array(msg.data, dtype=np.int8).reshape(shape)

    def task_callback(self, msg: Polygon) -> None:
        """ Callback function for the task subscription. Updates the start and end positions. """
        self.get_logger().info(f"Task: {msg.data}")
        self.y_start = msg.points[0].y
        self.x_start = msg.points[0].x
        self.y_end = msg.points[1].y
        self.x_end = msg.points[1].x
        if self.map != []:
            self.get_path()
    
def main(args = None):
    """ Main function to initialize the ROS2 node and start spinning. """
    rclpy.init(args = args)
    planning = Path()
    try:
        rclpy.spin(planning)
    except KeyboardInterrupt:
        planning.get_logger().info("Planning node interrupted by user.")
    finally:
        planning.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()