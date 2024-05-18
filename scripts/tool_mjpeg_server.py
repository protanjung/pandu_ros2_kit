#!/usr/bin/python3

import sys
from threading import Lock, Thread

import cv2 as cv
import numpy as np
import rclpy
import rclpy.logging
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from flask import Flask, Response, render_template_string, request, url_for
from rclpy.node import Node
from sensor_msgs.msg import Image


class ToolMJPEGServer(Node):
    def __init__(self):
        super().__init__("mjpeg_server")
        # ----Parameter
        self.declare_parameter("port", 9999)
        self.declare_parameter("topics", ["image_bgr", "image_face"])
        self.declare_parameter("scale", 0.5)
        self.declare_parameter("quality", 50)
        self.port = self.get_parameter("port").value
        self.topics = self.get_parameter("topics").value
        self.scale = self.get_parameter("scale").value
        self.quality = self.get_parameter("quality").value
        # ----Subscriber
        self.subs_image = {}

        # Image data
        # ----------
        self.frames = {}
        self.mutexes = {}

        # Flask
        # -----
        self.flask_app = Flask(__name__)

        # Convert all topics to absolute topics with respect to the namespace
        # This is to make sure that the topics are correct and consistent
        ns = self.get_namespace()
        for i, topic in enumerate(self.topics):
            if topic.startswith("/"):
                continue
            if ns == "/":
                self.topics[i] = ns + topic
            else:
                self.topics[i] = ns + "/" + topic

        # Initialize subscriber, frame, and mutex for each topic
        # A lambda function is used to pass the topic to the callback function
        for topic in self.topics:
            self.subs_image[topic] = self.create_subscription(
                Image,
                topic,
                lambda msg, topic=topic: self.cllbck_sub_image(msg, topic),
                10,
            )
            self.frames[topic] = np.zeros((64, 64, 3), np.uint8)
            self.mutexes[topic] = Lock()

        # Flask route definitions to serve the MJPEG stream
        # The route function is defined inside the class to access the class variables
        template_index = (
            get_package_share_directory("pandu_ros2_kit")
            + "/assets/template_index.html"
        )
        template_image = (
            get_package_share_directory("pandu_ros2_kit")
            + "/assets/template_image.html"
        )
        template_warning = (
            get_package_share_directory("pandu_ros2_kit")
            + "/assets/template_warning.html"
        )

        @self.flask_app.route("/")
        def flask_index():
            return render_template_string(
                open(template_index).read(), topics=self.topics
            )

        @self.flask_app.route("/image/<path:image>")
        def flask_image(image):
            scale = float(request.args.get("scale", self.scale))
            scale = max(0.1, min(10.0, scale))
            quality = int(request.args.get("quality", self.quality))
            quality = max(1, min(100, quality))
            return render_template_string(
                open(template_image).read(),
                image="/" + image,
                scale=scale,
                quality=quality,
            )

        @self.flask_app.route("/stream/<path:topic>")
        def flask_stream(topic):
            scale = float(request.args.get("scale", self.scale))
            scale = max(0.1, min(10.0, scale))
            quality = int(request.args.get("quality", self.quality))
            quality = max(1, min(100, quality))
            if "/" + topic not in self.topics:
                return render_template_string(
                    open(template_warning).read(),
                    message="Topic " + topic + " not found.",
                )
            return Response(
                self.imencode_stream("/" + topic, scale, quality),
                mimetype="multipart/x-mixed-replace; boundary=frame",
            )

        @self.flask_app.route("/snapshot/<path:topic>")
        def flask_snapshot(topic):
            scale = float(request.args.get("scale", self.scale))
            scale = max(0.1, min(10.0, scale))
            quality = int(request.args.get("quality", self.quality))
            quality = max(1, min(100, quality))
            if "/" + topic not in self.topics:
                return render_template_string(
                    open(template_warning).read(),
                    message="Topic " + topic + " not found.",
                )
            return Response(
                self.imencode_snapshot("/" + topic, scale, quality),
                mimetype="image/jpeg",
            )

        # Flask thread to run the Flask app independently
        # It use threading to run the Flask app in parallel with the ROS2 node
        self.flask_thread = Thread(target=self.flask_app_run)
        self.flask_thread.daemon = True
        self.flask_thread.start()

        self.get_logger().info("Port: " + str(self.port))
        self.get_logger().info("Topics: " + str(self.topics))
        self.get_logger().info("Scale: " + str(self.scale))
        self.get_logger().info("Quality: " + str(self.quality))

    def cllbck_sub_image(self, msg, topic):
        if self.mutexes[topic].acquire(blocking=False):
            self.frames[topic] = CvBridge().imgmsg_to_cv2(msg, "bgr8")
            self.mutexes[topic].release()

    def flask_app_run(self):
        self.flask_app.run("0.0.0.0", self.port, debug=False)

    def imencode_stream(self, topic, scale, quality):
        rate = self.create_rate(10)
        while rclpy.ok():
            rate.sleep()

            self.mutexes[topic].acquire()
            frame = self.frames[topic]
            self.mutexes[topic].release()

            frame_resized = cv.resize(
                frame, (0, 0), fx=scale, fy=scale, interpolation=cv.INTER_AREA
            )
            _, frame_encoded = cv.imencode(
                ".jpg", frame_resized, [cv.IMWRITE_JPEG_QUALITY, quality]
            )

            yield (
                b"--frame\r\n"
                b"Content-Type: image/jpeg\r\n\r\n" + frame_encoded.tobytes() + b"\r\n"
            )

    def imencode_snapshot(self, topic, scale, quality):
        self.mutexes[topic].acquire()
        frame = self.frames[topic]
        self.mutexes[topic].release()

        if frame is None:
            frame = np.zeros((64, 64, 3), np.uint8)
            cv.line(frame, (0, 0), (64, 64), (0, 0, 255), 2)
            cv.line(frame, (0, 64), (64, 0), (0, 0, 255), 2)

        frame_resized = cv.resize(
            frame, (0, 0), fx=scale, fy=scale, interpolation=cv.INTER_AREA
        )
        _, frame_encoded = cv.imencode(
            ".jpg", frame_resized, [cv.IMWRITE_JPEG_QUALITY, quality]
        )

        return frame_encoded.tobytes()


def main(args=None):
    rclpy.init(args=args)

    node_tool_mjpeg_server = ToolMJPEGServer()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_tool_mjpeg_server)
    executor.spin()


if __name__ == "__main__":
    main(sys.argv)
