#!/usr/bin/python3

import struct
import sys
from threading import Lock, Thread

import cv2 as cv
import mediapipe as mp
import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray, UInt8MultiArray


class ToolFaceDetector(Node):
    def __init__(self):
        super().__init__("face_detector")
        # ----Timer
        self.tim_10hz = self.create_timer(0.1, self.cllbck_tim_10hz)
        # ----Subscriber
        self.sub_image_bgr = self.create_subscription(
            Image, "image_bgr", self.cllbck_sub_image_bgr, 10
        )
        # ----Publisher
        self.pub_image_face = self.create_publisher(Image, "image_face", 10)
        self.pub_face_position = self.create_publisher(
            UInt8MultiArray, "face/position", 10
        )
        self.pub_face_position_biggest = self.create_publisher(
            Float32MultiArray, "face/position_biggest", 10
        )
        self.pub_face_position_smallest = self.create_publisher(
            Float32MultiArray, "face/position_smallest", 10
        )

        # Image data
        # ----------
        self.frame = np.zeros((64, 64, 3), np.uint8)
        self.mutex = Lock()

        # MediaPipe
        # ---------
        BaseOptions = mp.tasks.BaseOptions
        FaceDetector = mp.tasks.vision.FaceDetector
        FaceDetectorOptions = mp.tasks.vision.FaceDetectorOptions
        VisionRunningMode = mp.tasks.vision.RunningMode

        model_path = (
            get_package_share_directory("pandu_ros2_kit")
            + "/assets/blaze_face_short_range.tflite"
        )
        options = FaceDetectorOptions(
            base_options=BaseOptions(model_asset_path=model_path),
            running_mode=VisionRunningMode.LIVE_STREAM,
            result_callback=self.mediapipe_print_result,
        )

        self.detector = FaceDetector.create_from_options(options)

    def cllbck_tim_10hz(self):
        self.mutex.acquire()
        mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.frame)
        self.mutex.release()

        self.detector.detect_async(
            mp_image, int(self.get_clock().now().nanoseconds / 1e6)
        )

    def cllbck_sub_image_bgr(self, msg):
        if self.mutex.acquire(blocking=False):
            self.frame = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.mutex.release()

    def mediapipe_print_result(self, result, output_image, timestamp_ms):
        frame = output_image.numpy_view().copy()

        biggest_index = -1
        biggest_area = 0
        smallest_index = -1
        smallest_area = 1e9

        for i, detection in enumerate(result.detections):
            # Get bounding box
            x = detection.bounding_box.origin_x
            y = detection.bounding_box.origin_y
            w = detection.bounding_box.width
            h = detection.bounding_box.height
            # Draw bounding box
            cv.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)

            for j, keypoint in enumerate(detection.keypoints):
                # Get keypoint
                x = int(keypoint.x * frame.shape[1])
                y = int(keypoint.y * frame.shape[0])
                # Draw keypoint
                cv.circle(frame, (x, y), 4, (0, 0, 255), -1)

            # Find biggest and smallest face
            area = w * h
            if area > biggest_area:
                biggest_index = i
                biggest_area = area
            if area < smallest_area:
                smallest_index = i
                smallest_area = area

        # Publish visualization image with bounding_box and keypoints
        msg_image_face = CvBridge().cv2_to_imgmsg(frame, "bgr8")
        self.pub_image_face.publish(msg_image_face)

        # Publish biggest face position
        msg_face_position_biggest = Float32MultiArray()
        if biggest_index != -1:
            x = (
                result.detections[biggest_index].bounding_box.origin_x
                + 0.5 * result.detections[biggest_index].bounding_box.width
            ) / frame.shape[1]
            y = (
                result.detections[biggest_index].bounding_box.origin_y
                + 0.5 * result.detections[biggest_index].bounding_box.height
            ) / frame.shape[0]
            msg_face_position_biggest.data = [x, y]
        self.pub_face_position_biggest.publish(msg_face_position_biggest)

        # Publish smallest face position
        msg_face_position_smallest = Float32MultiArray()
        if smallest_index != -1:
            x = (
                result.detections[smallest_index].bounding_box.origin_x
                + 0.5 * result.detections[smallest_index].bounding_box.width
            ) / frame.shape[1]
            y = (
                result.detections[smallest_index].bounding_box.origin_y
                + 0.5 * result.detections[smallest_index].bounding_box.height
            ) / frame.shape[0]
            msg_face_position_smallest.data = [x, y]
        self.pub_face_position_smallest.publish(msg_face_position_smallest)

        # Face position protocol
        # ---------------------------
        # Offset | Size | Description
        # -------|------|------------
        # 0      | 1    | Number of faces
        # 1      | 4    | Face 1 x (0.0 - 1.0)
        # 5      | 4    | Face 1 y (0.0 - 1.0)
        # 9      | 4    | Face 2 x (0.0 - 1.0)
        # 13     | 4    | Face 2 y (0.0 - 1.0)
        # ...    | ...  | ... (repeat for each face)

        # Publish face position
        msg_face_position = UInt8MultiArray()
        msg_face_position.data = [len(result.detections)]
        while len(result.detections) > 0:
            biggest_index = 0
            biggest_area = 0
            for i, detection in enumerate(result.detections):
                x = detection.bounding_box.origin_x
                y = detection.bounding_box.origin_y
                w = detection.bounding_box.width
                h = detection.bounding_box.height
                area = w * h
                if area > biggest_area:
                    biggest_index = i
                    biggest_area = area
            x = (
                result.detections[biggest_index].bounding_box.origin_x
                + 0.5 * result.detections[biggest_index].bounding_box.width
            ) / frame.shape[1]
            y = (
                result.detections[biggest_index].bounding_box.origin_y
                + 0.5 * result.detections[biggest_index].bounding_box.height
            ) / frame.shape[0]
            msg_face_position.data.extend([byte for byte in struct.pack("ff", x, y)])
            result.detections.pop(biggest_index)
        self.pub_face_position.publish(msg_face_position)


def main(args=None):
    rclpy.init(args=args)

    node_tool_face_detector = ToolFaceDetector()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_tool_face_detector)
    executor.spin()


if __name__ == "__main__":
    main(sys.argv)
