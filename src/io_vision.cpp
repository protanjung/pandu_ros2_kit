#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include "pandu_ros2_kit/help_logger.hpp"
#include "rclcpp/rclcpp.hpp"

class IOVision : public rclcpp::Node {
  public:
    //----Parameter
    std::string camera_path;
    int camera_width;
    int camera_height;
    int camera_fps;
    int output_width;
    int output_height;
    //----Publisher
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_bgr;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_image_gray;
    //----Help
    HelpLogger logger;
    //----Thread
    std::thread thr_main;

    // Image data
    // ----------
    cv::VideoCapture cap;
    cv::Mat frame_raw;
    cv::Mat frame_bgr;
    cv::Mat frame_gray;

    IOVision() : Node("io_vision") {
        //----Parameter
        this->declare_parameter("camera_path", rclcpp::PARAMETER_STRING);
        this->declare_parameter("camera_width", 640);
        this->declare_parameter("camera_height", 480);
        this->declare_parameter("camera_fps", 30);
        this->declare_parameter("output_width", 640);
        this->declare_parameter("output_height", 480);
        this->get_parameter("camera_path", camera_path);
        this->get_parameter("camera_width", camera_width);
        this->get_parameter("camera_height", camera_height);
        this->get_parameter("camera_fps", camera_fps);
        this->get_parameter("output_width", output_width);
        this->get_parameter("output_height", output_height);
        //----Publisher
        pub_image_bgr = this->create_publisher<sensor_msgs::msg::Image>("image_bgr", 10);
        pub_image_gray = this->create_publisher<sensor_msgs::msg::Image>("image_gray", 10);
        //----Help
        if (!logger.init()) {
            rclcpp::shutdown();
        }
        //----Thread
        thr_main = std::thread(&IOVision::main, this);
    }

    ~IOVision() {
        if (thr_main.joinable()) {
            thr_main.join();
        }
    }

    void
    main() {
        logger.info("camera_path: %s", camera_path.c_str());
        logger.info("camera_width: %d", camera_width);
        logger.info("camera_height: %d", camera_height);
        logger.info("camera_fps: %d", camera_fps);
        logger.info("output_width: %d", output_width);
        logger.info("output_height: %d", output_height);

        if (!cap.open(camera_path, cv::CAP_V4L)) {
            logger.error("Failed to open camera.");
            rclcpp::shutdown();
        }

        int camera_fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

        cap.set(cv::CAP_PROP_FOURCC, camera_fourcc);
        cap.set(cv::CAP_PROP_FRAME_WIDTH, camera_width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, camera_height);
        cap.set(cv::CAP_PROP_FPS, camera_fps);
        logger.info("Set camera fourcc code: %d", camera_fourcc);
        logger.info("Set camera parameters: %dx%d %d fps", camera_width, camera_height, camera_fps);

        int _fourcc = cap.get(cv::CAP_PROP_FOURCC);
        int _width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
        int _height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
        int _fps = cap.get(cv::CAP_PROP_FPS);
        logger.info("Get camera fourcc code: %d", _fourcc);
        logger.info("Get camera parameters: %dx%d %d fps", _width, _height, _fps);

        if (_fourcc != camera_fourcc || _width != camera_width || _height != camera_height || _fps != camera_fps) {
            logger.error("Failed to set camera parameters.");
            rclcpp::shutdown();
        }

        while (rclcpp::ok()) {
            if (!cap.read(frame_raw)) {
                logger.error("Failed to read frame.");
                break;
            }

            cv::resize(frame_raw, frame_bgr, cv::Size(output_width, output_height));
            auto msg_bgr = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame_bgr).toImageMsg();
            pub_image_bgr->publish(*msg_bgr);

            cv::cvtColor(frame_bgr, frame_gray, cv::COLOR_BGR2GRAY);
            auto msg_gray = cv_bridge::CvImage(std_msgs::msg::Header(), "mono8", frame_gray).toImageMsg();
            pub_image_gray->publish(*msg_gray);
        }
    }
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_io_vision = std::make_shared<IOVision>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_io_vision);
    executor.spin();

    return 0;
}