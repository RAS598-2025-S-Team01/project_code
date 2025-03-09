import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class OakDBlobDetection(Node):
    def __init__(self):
        super().__init__('oakd_blob_detection')
        self.bridge = CvBridge()

        # Subscribe to RGB image topic
        self.subscription_rgb = self.create_subscription(
            Image,
            '/rpi_04/oak/rgb/image_raw',
            self.image_callback,
            10
        )

        # Subscribe to Depth image topic
        self.subscription_depth = self.create_subscription(
            Image,
            '/rpi_04/oak/stereo/image_raw',  # Stereo depth image topic
            self.depth_callback,
            10
        )

        # Publisher for blob size and location data
        self.publisher = self.create_publisher(String, '/rpi_04/blob_data', 10)

        self.latest_depth_frame = None  # Store latest depth frame
        self.image_width = 360  # Set display width for angle calculation
        self.get_logger().info('Blob detection node started.')

    def depth_callback(self, msg):
        try:
            self.latest_depth_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f"Failed to process depth image: {e}")

    def image_callback(self, msg):
        try:
            # Convert ROS image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Reduce image size for display
            cv_image = cv2.resize(cv_image, (self.image_width, 240))

            # Detect blobs and get their sizes and centroids
            blob_data = self.detect_blobs(cv_image)

            # Publish blob data sorted by color and then size
            if blob_data:
                sorted_blob_data = {color: sorted(blobs, key=lambda b: b["size"], reverse=True) for color, blobs in blob_data.items()}
                blob_msg = String()
                blob_msg.data = str(sorted_blob_data)
                self.publisher.publish(blob_msg)

                for color, blobs in sorted_blob_data.items():
                    for blob in blobs:
                        x, y, size_px, angle = blob["x"], blob["y"], blob["size"], blob["angle"]
                        size_m = self.calculate_size_m(size_px, y)
                        self.get_logger().info(f"{color} Blob - Size: {size_m:.3f}m, Position: ({x}, {y}), Angle: {angle:.2f} degrees")

                        # Display blob info on image
                        cv2.putText(cv_image, f"{color}: {size_m:.3f}m, {angle:.1f}°", (x, y - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)
                        cv2.circle(cv_image, (x, y), 5, (0, 255, 255), -1)

            # Show the processed image
            cv2.imshow("Blob Detection", cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

    def detect_blobs(self, img):
        """Detects red, blue, and green blobs (including shades) and returns their size and centroid."""
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        color_ranges = {
            "Red": [(np.array([0, 20, 20]), np.array([10, 255, 255])),
                     (np.array([170, 50, 50]), np.array([180, 255, 255]))],
            "Green": [(np.array([35, 20, 20]), np.array([140, 255, 255]))],
            "Blue": [(np.array([90, 20, 20]), np.array([140, 255, 255]))]
        }

        blob_data = {"Red": [], "Green": [], "Blue": []}
        for color, ranges in color_ranges.items():
            mask = None
            for lower, upper in ranges:
                partial_mask = cv2.inRange(hsv, lower, upper)
                mask = partial_mask if mask is None else mask | partial_mask

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                size_px = cv2.contourArea(contour)
                if size_px > 500:  # Minimum size threshold
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                        angle = self.calculate_angle(cx)
                        blob_data[color].append({"x": cx, "y": cy, "size": size_px, "angle": angle})

        return blob_data

    def calculate_angle(self, x):
        """Calculate the angle of the object from the center of the camera's field of view."""
        fov = 60.0  # Assume 60-degree field of view
        center_x = self.image_width // 2

        # Calculate angle using proportionality to the FOV
        angle = (x - center_x) * (fov / self.image_width)
        return angle

    def calculate_size_m(self, size_px, y):
        """Convert blob size from pixels to meters using approximate depth estimation."""
        if self.latest_depth_frame is not None:
            depth_resized = cv2.resize(self.latest_depth_frame, (self.image_width, 240))
            if 0 <= y < depth_resized.shape[0]:
                depth_m = depth_resized[y, self.image_width // 2] / 1000.0  # Convert mm to meters
                pixel_to_meter_ratio = depth_m / 500  # Approximate conversion factor
                return size_px * pixel_to_meter_ratio
        return size_px * 0.0001  # Fallback assumption

def main(args=None):
    rclpy.init(args=args)
    node = OakDBlobDetection()
    rclpy.spin(node)

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

