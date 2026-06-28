import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import numpy as np
import socket
import subprocess
import threading

class Phone(Node):

    # Class Constants
    HOST = "127.0.0.1" # Local Host
    PORT = 5000 # Port: used for rerouting phone to computer

    def __init__(self):
        """
        Initialize the Phone node.
        Node uses Android Debug Bridge (ADB) to get the phone's GPS coordinates, speed, and heading.
        """
        self.heading = np.nan
        self.speed = np.nan
        self.latitude = np.nan
        self.longitude = np.nan

        self.get_adb_devices()
        self.route_adb()

        super().__init__('Phone_Pub')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'phone', 10)
        self.receiver = threading.Thread(target = self.get_odometry, daemon = True)
        self.receiver.start()

    def publish(self):
        msg = Float32MultiArray()
        msg.data = [self.latitude, self.longitude, self.speed, self.heading]
        self.publisher_.publish(msg)
        #self.get_logger().info(f"Phone: {msg.data}")
    
    def get_odometry(self):


        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind((self.HOST, self.PORT))
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
    
    def get_adb_devices(self):
        # Execute native 'adb devices' in terminal
        output = subprocess.check_output(["adb","devices"]).decode("utf-8")

        # Parse lines and omit header
        lines = output.strip().split("\n")[1:]

        # Isolate the serial identifiers
        devices = [line.split()[0] for line in lines if line.strip()]
        return devices

    def route_adb(self):
        # clean slate
        subprocess.run(["adb","reverse","--remove-all"])

        # Route the localhost to computer local host
        subprocess.run(["adb", "reverse", f"tcp:{self.PORT}", f"tcp:{self.PORT}"])
        self.get_logger().info(f"Routed phone localhost:{self.PORT} to computer localhost:{self.PORT}")

def main(args = None):
    rclpy.init(args = args)
    phone = Phone()
    try:
        rclpy.spin(phone)
    except KeyboardInterrupt:
        phone.get_logger().info("Phone node interrupted by user.")
    finally:
        phone.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
