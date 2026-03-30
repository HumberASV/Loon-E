import numpy as np

class Mapper():
    def __init__(self):
        self.mapW = 10
        self.mapL = 10
        self.res = 0.25

    def getCell(self, meter): #convert units in meters to units in cells
        cell = int(meter/self.res)

        return cell

    def add_obstacles(self, obstacles, locations):
        for i in range(len(obstacles)):
            """
            Adds a rectangular object into the given matrix.
            startX, startY, length, width are in world units.
            """
            obstacle = obstacles[i]
            location = locations[i]

            match obstacle: #Values rounded up to nearest whole cell length, assuming resolution of 0.25 m
                case 1 | 2 | 3 | 6 | 8 | 9:
                    obj_length = 0.5
                    obj_width = 0.5
                case 4 | 5 | 7:
                    obj_length = 0.25
                    obj_width = 0.25

            # Convert world units to grid cells
            objL = self.getCell(obj_length)
            objW = self.getCell(obj_width)

            objStartX = self.getCell(location[0])
            objStartY = self.getCell(location[1])

            # Write into the matrix
            for i in range(objL):
                for j in range(objW):
                    self.map[objStartY + j, objStartX + i] = obstacle

        return self.map

    def create_map(self):
        rows = self.getCell(self.mapW)
        cols = self.getCell(self.mapL)
        self.map = np.zeros((rows, cols))
        
        return
