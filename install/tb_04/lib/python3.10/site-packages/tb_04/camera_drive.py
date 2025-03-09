import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist
import json

class BlobTracker(Node):
    def __init__(self):
        super().__init__('blob_tracker_node')
        self.blob_subscriber = self.create_subscription(
            String,
            '/rpi_04/blob_data',
            self.blob_callback,
            10
        )
        self.cmd_publisher = self.create_publisher(Twist, '/rpi_04/cmd_vel', 10)
        self.largest_red_blob = None
        self.robot_stopped = False  # Flag to track if the robot has stopped
        self.timer = self.create_timer(0.1, self.drive_towards_blob)  # Runs every 0.1s

    def blob_callback(self, msg):
        blob_data = json.loads(msg.data)
        red_blobs = [blob for blob in blob_data if blob['color'] == 'red']

        if not red_blobs:
            self.largest_red_blob = None
            return

        # Find largest red blob by area
        self.largest_red_blob = max(red_blobs, key=lambda b: b['size']['width'] * b['size']['height'])

    def drive_towards_blob(self):
        """ Moves the robot towards the largest red blob, stops at <0.4m, resumes at >0.6m """
        twist = Twist()

        if self.largest_red_blob is not None:
            x_position = self.largest_red_blob['position']['x']
            distance = self.largest_red_blob.get('distance', None)  # Get distance if available

            if distance is None:
                return  # Ignore if distance is not provided

            # Stop if distance is less than 0.4 meters
            if distance < 0.4:
                self.get_logger().info(f"Blob reached! Stopping robot. Distance: {distance}m")
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.robot_stopped = True  # Mark robot as stopped

            # Resume moving only when distance becomes greater than 0.6 meters
            elif distance > 0.6:
                self.robot_stopped = False  # Allow movement again

            # If not stopped, control movement towards the blob
            if not self.robot_stopped:
                if x_position > 0.55:
                    twist.angular.z = -0.1  # Turn right
                elif x_position < 0.45:
                    twist.angular.z = 0.1   # Turn left
                else:
                    twist.angular.z = 0.0
                    twist.linear.x = 0.5  # Move forward if aligned

                self.get_logger().info(f"Driving towards red blob at x: {x_position}, distance: {distance}m")

        else:
            self.get_logger().info("No red blob detected!")
            twist.linear.x = 0.0
            twist.angular.z = 0.0  # Stop if no blob is detected

        self.cmd_publisher.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    blob_tracker = BlobTracker()

    try:
        rclpy.spin(blob_tracker)
    except KeyboardInterrupt:
        pass

    blob_tracker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

