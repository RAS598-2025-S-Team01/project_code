import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import json
import sys
import select
import termios
import tty

class TurtleBot4BlobTracker(Node):
    def __init__(self):
        super().__init__('turtlebot4_blob_tracker')
        self.bridge = CvBridge()

        # Publisher for velocity commands
        self.publisher_ = self.create_publisher(Twist, '/rpi_04/cmd_vel', 10)

        # Subscriber for blob distances
        self.subscription = self.create_subscription(
            String,
            '/rpi_04/blob_data',  # Updated to match camera output
            self.blob_distance_callback,
            10)
        self.subscription  # Prevent unused variable warning

        # Subscriber for camera feed
        self.subscription_img = self.create_subscription(
            Image,
            '/rpi_04/oak/rgb/image_raw',  # Camera topic
            self.image_callback,
            10)
        
        self.get_logger().info("TurtleBot4 Blob Tracker Node Started. Press 'd' to start moving, 'q' to quit.")

        # Movement control
        self.move_enabled = False  # Starts in a waiting state
        self.target_reached = False  # True when we stop at the target
        self.tracking_marker = None  # Tracks the marker once detected

    def blob_distance_callback(self, msg):
        """ Callback function to process /rpi_04/blob_data topic """
        try:
            blob_data = json.loads(msg.data.replace("'", "\""))  # Fix single quotes for JSON parsing
            red_blobs = blob_data.get("Red", [])

            if red_blobs:
                largest_blob = max(red_blobs, key=lambda b: b["size"])  # Get the largest red blob
                red_blob_distance = self.calculate_distance(largest_blob["size"])
                red_blob_angle = largest_blob["angle"]

                if self.move_enabled:
                    self.tracking_marker = largest_blob  # Lock onto the largest detected red marker
                    if red_blob_distance > 0.00:
                        self.move_towards_blob(red_blob_distance, red_blob_angle)
                    else:
                        self.stop()
                        self.get_logger().info("Target reached. Stopping.")
                        self.move_enabled = False  # Reset for next move command
                        self.target_reached = True  # Prevents further movement
                else:
                    self.stop()
            else:
                self.stop()

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing JSON data: {e}")

    def image_callback(self, msg):
        """ Callback function to display camera preview """
        try:
            cv_image = cv2.resize(self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8'), (360, 240))
            cv2.imshow("TurtleBot Camera Feed", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

    def move_towards_blob(self, distance, angle):
        """ Moves the robot forward and adjusts direction """
        velocity_msg = Twist()
        velocity_msg.linear.x = min(0.2, 0.1 * distance)  # Move slower when close
        velocity_msg.angular.z = angle * 0.1  # Adjust direction dynamically
        self.publisher_.publish(velocity_msg)

    def calculate_distance(self, size_px):
        """ Convert blob size in pixels to approximate distance in meters """
        return 0.5 / (size_px / 1000.0 + 0.01)  # Simple inverse proportionality model

    def stop(self):
        """ Stops the TurtleBot4 """
        velocity_msg = Twist()
        self.publisher_.publish(velocity_msg)

    def get_keypress(self):
        """ Checks if a key is pressed """
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

def main(args=None):
    rclpy.init(args=args)
    node = TurtleBot4BlobTracker()

    # Set terminal to raw mode for non-blocking key input
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(fd)

    try:
        while rclpy.ok():
            key = node.get_keypress()

            if key == 'd':  # Start moving towards the red blob
                node.move_enabled = True
                node.target_reached = False
                node.tracking_marker = None  # Reset marker tracking
            elif key == 'q':  # Quit the program
                node.get_logger().info("Quitting TurtleBot4 Blob Tracker")
                break

            rclpy.spin_once(node, timeout_sec=0.1)

    except KeyboardInterrupt:
        node.get_logger().info("Shutting down TurtleBot4 Blob Tracker")
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        node.stop()
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

