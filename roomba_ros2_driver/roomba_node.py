
import rclpy
from rclpy.node import Node
import serial
import time
from geometry_msgs.msg import Twist

class RoombaDriver(Node):
    def __init__(self):
        super().__init__('roomba_driver')

        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)

        # Read parameters
        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value

        # Establish serial connection
        self.ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout
        )

        while not self.ser.is_open:
            self.get_logger().info("Waiting for serial connection...")
            rclpy.sleep(0.2)
        self.get_logger().info(f"Serial connection established: port={self._port}, baudrate={self._baudrate}, timeout={self._timeout}")


        self.get_logger().info("Initalizing Roomba...")
        self.ser.write(bytes([128]))  # Start
        time.sleep(0.5)
        self.ser.write(bytes([132]))  # Full mode
        time.sleep(0.5)

        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info("Subscribed to cmd_vel topic")
        self.get_logger().info("RoombaDriver Node Running!")
    
# ---------------------------------------------------------------------------------------

    def cmd_vel_callback(self, msg):
        # Example: log received velocities
        self.get_logger().info(f"Received cmd_vel: linear={msg.linear.x}, angular={msg.angular.z}")
        self.get_logger().info(f"Sending to Roomba: linear={int(msg.linear.x * 100)}, angular={int(msg.angular.z * 100)}")
        self.ser.write(bytes([137, 0x00, int(msg.linear.x * 100), 0x80, int(msg.angular.z * 100)]))
        
# ---------------------------------------------------------------------------------------

    def start(self):
        self.get_logger().info("RoombaDriver started")

# ---------------------------------------------------------------------------------------

    def run(self):
        rclpy.spin(self)

# ---------------------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = RoombaDriver()
    node.start()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


# import serial
# import time

# # Configure serial connection
# ser = serial.Serial(
#     port="/dev/ttyACM0",      
#     baudrate=115200,
#     timeout=0.1
# )

# print("Serial connection established")
# time.sleep(5)

# # Initialize
# print("Initializing...")
# ser.write(bytes([128]))  # Start
# time.sleep(2.5)
# # ser.write(bytes([130]))  # Control
# # time.sleep(5)
# ser.write(bytes([132]))  # Full mode
# time.sleep(2.5)


# print("Drinving forward...")
# ser.write(bytes([137, 0x00, 10, 0x80, 0x00]))
# time.sleep(5)

# # Stop
# print("Stopping...")
# ser.write(bytes([137, 0x00, 0x00, 0x00, 0x00]))

# print("Commands sent successfully.")
