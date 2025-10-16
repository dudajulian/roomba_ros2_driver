import rclpy
from rclpy.node import Node
import serial
import time
from pycreate2 import Create2
import math
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf_transformations
from tf2_ros import TransformBroadcaster


class RoombaDriver(Node):
    def __init__(self):
        super().__init__('roomba_driver')
        # Declare parameters with default values
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('mode', 130)
        self.declare_parameter('debug', False)

        # Read parameters
        self._port = self.get_parameter('port').get_parameter_value().string_value
        self._baudrate = self.get_parameter('baudrate').get_parameter_value().integer_value
        self._timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self._mode = self.get_parameter('mode').get_parameter_value().integer_value
        self._debug = self.get_parameter('debug').get_parameter_value().bool_value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self._t = TransformStamped()

        # Timers
        self.timer = self.create_timer(0.1, self.computeOdom)

        # Establish serial connection
        self.ser = serial.Serial(
            port=self._port,
            baudrate=self._baudrate,
            timeout=self._timeout
        )

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)


# ---------------------------------------------------------------------------------------

    def start(self):
        """
        Start the RoombaDriver node, initialize the robot, and set up ROS interfaces.
        """
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
        self.subscription = self.create_subscription(Twist, 'cmd_vel', self.cmdVelCallback, 10)
        self.get_logger().info("Subscribed to cmd_vel topic")

        # Publisher
        self.odom_publisher = self.create_publisher(Odometry, 'roomba_ros2_driver/odom', 10)
        self.get_logger().info("Publisher to odom topic created")

        # Log node is running
        self.get_logger().info("RoombaDriver Node Running!")

# ---------------------------------------------------------------------------------------

    def cmdVelCallback(self, msg):
        """
        Callback function for cmd_vel topic. Converts Twist message to Roomba drive command.
        """
        linear_velocity = max(-500, min(500, int(msg.linear.x * 1000)))

        if msg.angular.z != 0:
            radius = int((msg.linear.x / msg.angular.z) * 1000)
            radius = max(-2000, min(2000, radius))
        else:
            radius = 0

        drive_command = [137] \
            + list(linear_velocity.to_bytes(2, 'big', signed=True)) \
            + list(radius.to_bytes(2, 'big', signed=True))

        self.ser.write(bytes(drive_command))

        if self._debug:
            self.get_logger().info(f"Drive command: {drive_command}")

# ---------------------------------------------------------------------------------------

    def computeOdom(self):
        """
        Reads the Distance and Angle sensor values from the Roomba and publishes the odometry.
        """
        distance_m = self.requestDistance()
        angle = self.requestAngle()
        if distance_m is not None and angle is not None:
            # Save previous theta for velocity calculation
            prev_theta = self.theta
            prev_x = self.x
            prev_y = self.y

            self.theta += angle
            self.theta = (self.theta + math.pi) % (2 * math.pi) - math.pi
            self.x += distance_m * math.cos(self.theta)
            self.y += distance_m * math.sin(self.theta)

            # Publish odometry message
            odom_msg = Odometry()
            odom_msg.header.stamp = self.get_clock().now().to_msg()
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0

            # Convert theta to quaternion
            q = tf_transformations.quaternion_from_euler(0, 0, self.theta)
            odom_msg.pose.pose.orientation.x = q[0]
            odom_msg.pose.pose.orientation.y = q[1]
            odom_msg.pose.pose.orientation.z = q[2]
            odom_msg.pose.pose.orientation.w = q[3]

            # Estimate velocities (simple finite difference)
            dt = 0.2  # timer period
            odom_msg.twist.twist.linear.x = (self.x - prev_x) / dt
            odom_msg.twist.twist.linear.y = (self.y - prev_y) / dt
            odom_msg.twist.twist.angular.z = (self.theta - prev_theta) / dt

            self.odom_publisher.publish(odom_msg)

            # Publish TF transform from odom to base_link
            
            self._t.header.stamp = self.get_clock().now().to_msg()
            self._t.header.frame_id = "odom"
            self._t.child_frame_id = "base_link"
            self._t.transform.translation.x = self.x
            self._t.transform.translation.y = self.y
            self._t.transform.translation.z = 0.0
            self._t.transform.rotation.x = q[0]
            self._t.transform.rotation.y = q[1]
            self._t.transform.rotation.z = q[2]
            self._t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(self._t)

# ---------------------------------------------------------------------------------------

    def requestDistance(self):
        """
        Request the distance traveled from the Roomba and return it in millimeters.
        """
        self.ser.write(bytes([142, 19]))
        data = self.ser.read(2)
        if len(data) == 2:
            dist = - int.from_bytes(data, byteorder='big', signed=True) / 100
            if self._debug: 
                self.get_logger().info(f"Distance: {dist} m")
            return dist
        else:
            self.get_logger().warn("Failed to read distance data from Roomba.")

# ---------------------------------------------------------------------------------------

    def requestAngle(self):
        """
        Requests the angle from the Roomba and returns it in radians.
        """
        self.ser.write(bytes([142, 20]))
        data = self.ser.read(2)
        if len(data) == 2:
            difference = int.from_bytes(data, byteorder='big', signed=True)
            angle_rad = (4 * difference) / 233.0
            if self._debug:
                self.get_logger().info(f"Angle: {angle_rad} radians")
            return angle_rad
        else:
            self.get_logger().warn("Failed to read angle data from Roomba.")
            return None

# ---------------------------------------------------------------------------------------

    def run(self):
        """
        Spin the ROS2 node.
        """
        rclpy.spin(self)

# ---------------------------------------------------------------------------------------

def main(args=None):
    """
    Main entry point for the RoombaDriver node.
    """
    rclpy.init(args=args)
    node = RoombaDriver()
    node.start()
    node.run()
    node.destroy_node()
    rclpy.shutdown()

# ---------------------------------------------------------------------------------------

if __name__ == "__main__":
    main()
