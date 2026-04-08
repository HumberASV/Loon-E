import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs import UInt8MultiArray

class TaskLogic(Node):
    def __init__(self):
        super().__init__("Task_Logic")
        self.destPublisher_ = self.create_publisher(UInt8MultiArray, "destination", 10)
        self.pwmPublisher_ = self.create_publisher(UInt8MultiArray, "PWM", 10)
        self.create_subscription(UInt8MultiArray, "map", self.map_callback, 10)
        self.create_subscription(UInt8MultiArray, "objects", self.object_callback, 10)
        self.create_subscription(UInt8MultiArray, "locations", self.location_callback, 10)
        period = 0.5
        self.timer = self.create_timer(period, self.run_task)
        
        self.task = [1]
        self.stage = 0
        self.next_step = 0

        self.image_objects = []
        self.locations = []
        self.map = []
    
    def object_found(self, target_objects):
        found = True
        for object in target_objects:
            if object not in self.image_objects:
                found = False
                break
        
        return found

    def check_change(self, target_objects, found):
        self.change_stage = False
        if self.next_step == 2:
            self.change_stage = True
            self.next_step = 0
        elif self.object_found(self, target_objects) == found:
            self.next_step = self.next_step + 1
        else:
            self.next_step = 0

    def search():
        #Rotate
        return

    def getCell(self, meter, resolution=0.25):
        cell = int(meter/resolution)

        return cell

    def follow_path(self, target_objects):
        loc_1 = (0, len(self.map))
        loc_2 = (0, len(self.map))

        for i in range(len(self.image_objects)): #Search for closest object of each type
            if self.image_objects[i] == target_objects[0] and self.locations[i][1] < loc_1[1]:
                loc_1 = self.locations[i]
            elif self.image_objects[i] == target_objects[1] and self.locations[i][1] < loc_2[1]:
                loc_2 = self.locations[i]
        
        dest_x = self.getCell((loc_1[0] + loc_2[0])/2)
        dest_y = self.getCell((loc_1[1] + loc_2[1])/2)
        self.publish_dest([dest_y, dest_x])

    def go_around_obstacle(self, target_object, dir=2):
        match self.stage:
            case 30: #Stage 30: To Buoy
                if self.next_step == 0:
                    for i in range(len(self.image_objects)):
                        if self.image_objects[i] == target_object:
                            dest = self.locations[i]
                            break
                    self.publish_dest([dest[1], dest[0] + dir])

                if self.check_change([6], False): #Change condition: Buoy not visible
                    self.stage = 31
            
            case 31: #Stage 31: Around buoy
                #Motor control if green
                #Motor control if red
                
                if self.check_change([4, 5], True): #Change condition: Start line found
                    self.stage = 32
            
            case 32: #Stage 32: Return to start line
                if self.next_step == 0:
                    self.follow_path([4, 5]) #Destination: Between red and green buoys
                    #Change condition in task function

    #For all tasks: Stage 0 = start of task, Stage 100 = stage complete, other stages are intermediary
    def task_1(self):
        match self.stage:
            case 0: #Stage 0: Search for waypoint
                if self.next_step == 0:
                    self.search()
                
                if self.check_change([1, 2], True): #Change condition: Tall Buoys found)
                    self.stage = 10
            
            case 10: #Stage 10: Follow path
                if self.next_step == 0:
                    self.follow_path([1, 2]) #Destination: Between buoys
                
                if self.check_change([1, 2], False): #Change condition: Tall Buoys not found
                    self.stage = 100

    def task_2(self):
        match self.stage:
            case 0: #Stage 0: Search for waypoint
                if self.next_step == 0:
                    self.search()
                
                if self.check_change([4, 5], True): #Change condition: Buoys found
                    self.stage = 10

            case 10: #Stage 10: Go through path
                if self.next_step == 0:
                    self.follow_path([4, 5])
                
                if self.check_change([4, 5], False): #Change condition: Buoys not found
                    self.stage = 20
            
            case 20: #Stage 20: Search for beacon
                if self.next_step == 0:
                    self.search()
                
                if self.check_change([9], True): #Change condition: Green Beacon found
                    self.stage = 30
            
            case 30, 31, 32: #Stage 30: Go around beacon
                if self.next_step == 0:
                    self.go_around_obstacle(9)
                
                if check_change([4, 5], False): #Change condition: Buoys found
                    self.stage = 100

    def task_3(self):
        match self.stage:
            case 0: #Stage 0: Search for waypoint
                if self.next_step == 0:
                    self.search()

                if self.check_change([6, 8], True): #Change condition: Yellow Buoy and Green Beacon found
                    dir = 2 #left
                    self.stage = 10
                elif self.check_change([6, 9], True): #Change condition: Yellow Buoy and Red Beacon found
                    dir = -2 #right
                    self.stage = 10
        
            case 30, 31, 32: #Stage 30: go around buoy
                if self.next_step == 0:
                    self.go_around_obstacle(6, dir)
                
                if self.check_change([4, 5], False): #Change condition: Buoys not visible
                    self.stage = 100

    def run_task(self):
        if self.task != []:
            match self.task[0]:
                case 1: self.task_1(self)
                case 2: self.task_2(self)
                case 3: self.task_3(self)

            if self.stage == 100:
                self.task.remove(self.task[0])
                self.stage = 0
    
    def publish_dest(self, dest):
        msg = UInt8MultiArray()
        msg.data = dest
        self.destPublisher_.publish(msg)

    def publish_pwm(self, pwm):
        msg = UInt8MultiArray()
        msg.data = pwm
        self.pwmPublisher_.publish(msg)
    
    def map_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.map = msg.data
    
    def object_callback(self, msg: UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.objects = msg.data

    def location_callback(self, msg:UInt8MultiArray):
        self.get_logger().info(msg.data)
        self.locations = msg.data
    
def main(args=None):
    rclpy.init(args=args)
    taskNode = TaskLogic()
    rclpy.spin(taskNode)
    taskNode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()