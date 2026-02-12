import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs import UInt8MultiArray

class PathPlanner(Node):
    def __init__(self):
        super().__init__("Path_Planner")
        self.publisher_ = self.create_publisher(UInt8MultiArray, "path", 10)
        self.create_subscription(UInt8MultiArray, "map", self.map_callback, 10)
        self.create_subscription(UInt8MultiArray, "destination", self.destination_callback, 10)
        period = 0.5
        self.timer = self.create_timer(period, self.publish)
        
        self.r = 3
        
        self.map = []
        self.destination = []

    def point_in_map(self, position):
        rows = len(self.map)
        cols = len(self.map[0])
        found = False
        
        if 0 <= position[0] < cols and 0 <= position[1] < rows:
            found = True
        
        return found

    def find_obstacle(self, point):
        found = False
        y = point[0]
        x = point[1]
        
        for i in range (y - self.r, y + self.r + 1):
            for j in range (x - self.r, x + self.r + 1):
                position = (i, j)
                if self.point_in_map(self, position) and (self.map[i][j] != 0):
                    found = True
                    break
        
        return found

    def pathfind(self, start, expanded):
        path = []
        
        if len(expanded)!=0: #If expanded is empty
            position=expanded[-1][1] #End node
            previous=expanded[-1][2] #Parent of end node
            while (previous!=start): #If parent is not start:
                path.append(position) #Add node to path
                for i in expanded: #Find parent in expanded
                    if (i[1]==previous):
                        position=expanded[expanded.index(i)][1] #Parent node = new end node
                        previous=expanded[expanded.index(i)][2] #Parent of new end node
                        break        
            path.append(position) #Add start node to path
            
        return path

    def check_neighbours(self, start, end, position, unexpanded, expanded, checked):
        y = position[0]
        x = position[1]
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)] #Up, down, right, left, diagonal
        
        for i, j in dir:
            next = (y + i, x + j)
            if self.point_in_map(self, next) and not (next in checked or self.find_obstacle(self, next)):
                manhattan = abs(next[0] - end[0]) + abs(next[1] - end[1])
                cost = manhattan + len(self.pathfind(self, start, expanded))
                unexpanded.append([cost, next, position])
                checked.append(next)
            
    def update_waypoints(self, path, start, parent_list):
        waypoints = path
        location = waypoints.index(start)+1 #Find point in path list
        for i in parent_list:
            waypoints.insert(location, i)
        waypoints.remove(parent_list[0])

        return waypoints

    def avoid_obstacle(self, path, point):
        unexpanded=[] #Evaluated neighbours in form [cost, current position, previous position]
        expanded=[] #Visited in form [cost, current position, previous position]
        checked=[] #Checked cells in form [position]
        
        start = path[path.index(point)-1]
        end_index = path.index(point)
        end = path[end_index]
        while self.find_obstacle(self, end) and (end_index != len(path)-1):
            path.remove(end)
            end = path[end_index]
        
        position = (start[0], start[1])
        while position != end:
            self.check_neighbours(self, start, end, position, unexpanded, expanded, checked)
            
            if(unexpanded == []): #If no more nodes to search
                print("No solution found")
                break

            next = min(unexpanded) #Select node with lowest cost
            expanded.append(next) #Move node to expanded list
            unexpanded.remove(next) #Remove node from unexpanded 
            position=expanded[-1][1] #Set to expand selected node on repeat

        parent_list = self.pathfind(self, start, expanded)
        waypoints = self.update_waypoints(self, path, start, parent_list)

        return waypoints
        
    def make_waypoints(self, path, d=4):
        path_obstacles = path
        waypoints = []

        for point in path:
            if self.find_obstacle(self, point): #If waypoint near obstacle, generate new path between previous and next waypoints
                path_obstacles = self.avoid_obstacle(self, path, point)
                
        for point in path_obstacles:
            if ((path_obstacles.index(point) % d == 0) or (point == path_obstacles[-1])): #Add every d points to waypoint list
                waypoints.append(point)
        return waypoints

    def generate_path(self, x_end, y_end):
        x_start = int(len(self.map[0])/2) #y_start always 0
        path = []
        
        dx = x_end - x_start #change in x
        if dx == 0: #vertical line
            for y in range(y_end + 1):
                path.append((y, x_start))
        elif y_end <= dx or -1 < y_end/dx < 0: #Typical program
            m = y_end/dx #Calculate slope
            b = -m * x_start #Calculate y intercept
            for x in range(x_start, x_end + np.sign(dx), np.sign(dx)):
                y = round(m * x + b) #Find nearest y value
                path.append((y, x)) #Add point to path
        else: #Switch x and y so that m <= 1
            m = dx/y_end #Calculate slope
            for y in range(y_end + 1):
                x = round(m * y + x_start) #Find nearest x value
                path.append((y, x)) #Add point to path
        
        waypoints = self.make_waypoints(self, path)
        return waypoints
    
    def publish(self):
        msg = UInt8MultiArray()
        msg.data = self.generate_path(self, self.map, self.destination[0], self.destination[1])
        self.publisher_.publish(msg)
    
    def map_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.map = msg.data

    def destination_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.destination = msg.data

def main(args=None):
    rclpy.init(args=args)
    plannerNode = PathPlanner()
    rclpy.spin(plannerNode)
    plannerNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()