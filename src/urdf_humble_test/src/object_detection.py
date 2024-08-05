#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ObjectDetection(Node):
    def __init__(self):
        super().__init__('object_detection')
        self.subscription = self.create_subscription(
            Image,
            '/camera_sensor/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        self.get_logger().info('Receiving video frame')
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        detected_frame = self.detect_objects(frame)
        cv2.imshow('Detected Objects', detected_frame)
        cv2.waitKey(1)

    def detect_objects(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        return frame

def main(args=None):
    rclpy.init(args=args)
    object_detection = ObjectDetection()
    rclpy.spin(object_detection)
    object_detection.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

