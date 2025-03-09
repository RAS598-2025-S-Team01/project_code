import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
import numpy as np
import json
from cv_bridge import CvBridge

class DepthBlobDetectorNode(Node):
    def __init__(self):
        super().__init__('depth_blob_detector_node')

        self.rgb_subscriber = self.create_subscription(
            Image,
            '/rpi_04/oak/rgb/image_rect',
            self.rgb_callback,
            10
        )

        self.depth_subscriber = self.create_subscription(
            Image,
            '/rpi_04/oak/stereo/image_raw',
            self.depth_callback,
            10
        )

        self.publisher = self.create_publisher(String, '/rpi_04/blob_data', 10)
        self.bridge = CvBridge()

        self.latest_depth_image = None

        self.color_ranges = {
            'red_lower': ((0, 100, 100), (10, 255, 255)),
            'red_upper': ((160, 100, 100), (179, 255, 255)),
            'green': ((40, 40, 40), (70, 255, 255)),
            'blue': ((100, 150, 0), (140, 255, 255))
        }

    def depth_callback(self, msg):
        self.latest_depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def rgb_callback(self, msg):
        if self.latest_depth_image is None:
            self.get_logger().warn('Depth image not received yet.')
            return

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        detections = []

        for color_key in ['red_lower', 'red_upper', 'green', 'blue']:
            mask = cv2.inRange(hsv, np.array(self.color_ranges[color_key][0]), np.array(self.color_ranges[color_key][1]))
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # Sort contours by area in descending order
            contours = sorted(contours, key=cv2.contourArea, reverse=True)
            valid_contours = []

            for i, contour in enumerate(contours):
                if cv2.contourArea(contour) < 100:
                    continue  # Ignore small noise

                is_inside = False
                for j in range(i):  # Check if the current contour is inside any larger contour
                    point = tuple(map(int, contour[0][0]))  # Ensure (x, y) is a tuple of integers
                    if cv2.pointPolygonTest(valid_contours[j], point, False) >= 0:
                        is_inside = True
                        break

                if not is_inside:
                    valid_contours.append(contour)

            # Process only the outermost contours
            for contour in valid_contours:
                rect = cv2.minAreaRect(contour)
                (x, y), (w, h), angle = rect
                box = cv2.boxPoints(rect)
                box = np.intp(box)

                cx, cy = int(x), int(y)
                depth_value = self.latest_depth_image[cy, cx]

                color_label = 'red' if 'red' in color_key else color_key

                detection = {
                    'color': color_label,
                    'position': {'x': cx / 1000.0, 'y': cy / 1000.0},
                    'size': {'width': w / 1000.0, 'height': h / 1000.0},
                    'angle': angle,
                    'distance': float(depth_value) / 1000.0 if depth_value > 0 else None
                }
                detections.append(detection)

                cv2.drawContours(image, [box], 0, (0, 255, 255), 2)
                cv2.putText(image, color_label, (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1.8, (255, 255, 255), 5)

        json_message = json.dumps(detections)
        self.get_logger().info(f'Publishing detections: {json_message}')
        self.publisher.publish(String(data=json_message))

        resized_image = cv2.resize(image, (360, 240))
        cv2.imshow('Blob Detections', resized_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DepthBlobDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

