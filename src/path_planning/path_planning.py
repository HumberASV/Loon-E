import numpy as np

class pathPlanner():
    def __init__(self):
        self.map = []
        self.x_start = -1
        self.y_start = 0
        self.x_end = -1
        self.y_end = -1

        self.path = [] #straight line path
        self.path_obstacles = [] #path avoiding obstacles
        self.waypoints = [] #self.path_obstacles with reduced resolution, based on self.dist
        self.dist = 4 #interval between waypoints, in cells. Same as motor_control self.max
        self.radius = 3 #min distance from obstacles, in cells

    def point_in_map(self, position):
        rows = len(self.map)
        cols = len(self.map[0])
        output = False
        
        if 0 <= position[0] < cols and 0 <= position[1] < rows: #If desired area to search is within map bounds
            output = True
        
        return output

    def find_obstacle(self, point):
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

    def pathfind(self, start, expanded):
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

    def check_neighbours(self, start, end, position, unexpanded, expanded, checked):
        y = position[0]
        x = position[1]
        dir = [(1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (-1, -1), (1, -1)] #Up, down, right, left, diagonal
        
        for i, j in dir:
            next = (y + i, x + j)
            if self.point_in_map(next) and not (next in checked or self.find_obstacle(next)): #If in map, not checked, and not near obstacle
                manhattan = abs(next[0] - end[0]) + abs(next[1] - end[1])
                cost = manhattan + len(self.pathfind(start, expanded))
                unexpanded.append([cost, next, position])
                checked.append(next)
        
        return
            
    def update_waypoints(self, start, parent_list):
        location = self.path_obstacles.index(start)+1 #Find point in path list
        
        for point in parent_list:
            self.path_obstacles.insert(location, point) #Add waypoints from obstacle avoidance
        self.path_obstacles.remove(parent_list[0])

        return

    def avoid_obstacle(self, point):
        unexpanded=[] #Evaluated neighbours in form [cost, current position, previous position]
        expanded=[] #Visited in form [cost, current position, previous position]
        checked=[] #Checked cells in form [position]
        
        start = self.path[self.path.index(point)-1]
        end_index = self.path.index(point)
        end = self.path[end_index]
        position = (start[0], start[1])

        while self.find_obstacle(end) and (end_index != len(self.path_obstacles)-1): #move destination further away so it does not intersect with an obstacle
            self.path.remove(end)
            end = self.path[end_index]
        
        while position != end:
            self.check_neighbours(start, end, position, unexpanded, expanded, checked)
            
            if(unexpanded == []): #If no more nodes to search
                print("No solution found")
                break

            next = min(unexpanded) #Select node with lowest cost
            expanded.append(next) #Move node to expanded list
            unexpanded.remove(next) #Remove node from unexpanded 
            position = expanded[-1][1] #Set to expand selected node on repeat

        parent_list = self.pathfind(start, expanded)
        self.update_waypoints(start, parent_list)

        return
        
    def make_waypoints(self):
        self.path_obstacles = self.path

        for point in self.path:
            if self.find_obstacle(point): #If waypoint near obstacle, generate new path between previous and next waypoints
                print(point)
                self.avoid_obstacle(point)
                
        for point in self.path_obstacles:
            y = point[0]
            x = point[1]
            if ((self.path_obstacles.index(point) % self.dist == 0) or (point == self.path_obstacles[-1])): #Add every d points to waypoint list and last waypoint
                self.waypoints.append(point)
                self.map[y][x] = 3

        return

    def generate_path(self, map, x_end, y_end):
        self.map = map
        self.x_start = int(len(self.map[0])/2) #In this iteration, x_start and y_start constant with y_start = 0
        self.x_end = x_end
        self.y_end = y_end

        while self.find_obstacle((self.y_end, self.x_end)): #Ensure that end point is not near obstacle
            self.y_end = self.y_end - 1
        
        dx = self.x_end - self.x_start #change in x
        dy = self.y_end - self.y_start #change in y

        if dx == 0: #vertical line
            for y in range(self.y_start, self.y_end + 1, np.sign(self.x_end-self.x_start)):
                self.path.append((y, self.x_start))
        elif dy == 0: #horizontal line
            for x in range(self.x_start, self.x_end, np.sign(self.x_end-self.x_start)):
                self.path.append((self.y_end, x))
        elif dy <= dx or -1 < dy/dx < 0: #Typical program
            m = dy/dx #Calculate slope
            b = -m * self.x_start + self.y_start #Calculate y intercept
            for x in range(self.x_start, self.x_end + np.sign(dx), np.sign(dx)):
                y = round(m * x + b) #Find nearest y value
                self.path.append((y, x)) #Add point to path
        else: #Switch x and y so that m <= 1
            m = dx/dy #Calculate slope
            b = -m * self.y_start + self.x_start #calculate x intercept
            for y in range(self.y_start, self.y_end + np.sign(dy), np.sign(dy)):
                x = round(m * y + b) #Find nearest x value
                self.path.append((y, x)) #Add point to path
        
        self.make_waypoints()

        return self.waypoints