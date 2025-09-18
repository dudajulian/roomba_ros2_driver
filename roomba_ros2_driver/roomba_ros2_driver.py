
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
        self.declare_parameter('mode', 130)

        # Read parameters
        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self._mode = self.get_parameter('mode').get_parameter_value().integer_value

        # Establish serial connection
        self.ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout
        )       
    
# ---------------------------------------------------------------------------------------

    def cmdVelCallback(self, msg):
        linear_velocity = int(msg.linear.x * 100)
        radius = int((msg.angular.z) * 10) # TODO compute cradius from linear and angular velocity
        # TODO check limits to be within -500 to 500 mm/s for linear velocity and and -2000 to 2000 mm for radius
        drive_command = [137] \
            + list(linear_velocity.to_bytes(2, 'big', signed=True)) \
            + list(radius.to_bytes(2, 'big', signed=True))
        
        self.get_logger().info(f"Received cmd_vel: linear.x={msg.linear.x} mm/s, angular.z={msg.angular.z}mm | Drive command: {drive_command}") 
        self.ser.write(bytes(drive_command))

# ---------------------------------------------------------------------------------------

    def start(self):
        self.get_logger().info("Starting RoombaDriver Node...")

        # Initialize SCI connetinon
        self.get_logger().info("Initalizing Roomba...")
        self.ser.write(bytes([128]))  # 128 = Start
        time.sleep(0.5)
        self.ser.write(bytes([self._mode]))
        time.sleep(0.5)

        # Wait for serial connection to be established
        while not self.ser.is_open:
            self.get_logger().info("Waiting for serial connection...")
            rclpy.sleep(0.2)
        self.get_logger().info(f"Serial connection established: port={self._port}, baudrate={self._baudrate}, timeout={self._timeout}, mode={self._mode}")
        
        # Subscribe to cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmdVelCallback,
            10
        )
        self.get_logger().info("Subscribed to cmd_vel topic")
       
        # Log node is running
        self.get_logger().info("RoombaDriver Node Running!")

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
    
# ---------------------------------------------------------------------------------------

if __name__ == "__main__":
    main()
