import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import socket
import threading

class Phone(Node):
    def __init__(self):
        self.heading = np.nan
        self.speed = np.nan
        self.latitude = np.nan
        self.longitude = np.nan
        
        super().__init__('Phone_Pub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'Phone', 10)
        self.receiver = threading.Thread(target = self.get_odometry, daemon = True)
        self.receiver.start()

    def publish(self):
        msg = Float32MultiArray()
        msg.data = [self.latitude, self.longitude, self.speed, self.heading]
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Phone: {msg.data}")
    
    def get_odometry(self):
        HOST = "127.0.0.1"
        PORT = 5000

        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((HOST, PORT))
        server.listen(1)

        while rclpy.ok():
            conn, addr = server.accept()

            with conn:
                buffer = ""

                while rclpy.ok():
                    data = conn.recv(1024)
                    if not data:
                        break

                    buffer += data.decode(errors="ignore")

                    while "\n" in buffer:
                        line, buffer = buffer.split("\n", 1)
                        line = line.strip()

                        if not line:
                            continue

                        parts = line.split(",")

                        if len(parts) == 4:

                            try:
                                self.heading = float(parts[0])
                                self.speed = float(parts[1])
                                self.latitude = float(parts[2])
                                self.longitude = float(parts[3])

                            except Exception as e:
                                continue
                            
                            self.publish()

def main(args = None):
    rclpy.init(args = args)
    phone = Phone()
    rclpy.spin(phone)
    phone.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()