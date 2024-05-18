#include "pandu_ros2_kit/help_logger.hpp"
#include "pandu_ros2_kit/help_udp.hpp"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

struct EIGEN_ALIGN16 MyPointXYZIR {
    PCL_ADD_POINT4D;
    float intensity;
    uint8_t ring;
    PCL_MAKE_ALIGNED_OPERATOR_NEW
};
POINT_CLOUD_REGISTER_POINT_STRUCT(MyPointXYZIR,                 // MyPointXYZIR
                                  (float, x, x)                 // x
                                  (float, y, y)                 // y
                                  (float, z, z)                 // z
                                  (float, intensity, intensity) // intensity
                                  (uint8_t, ring, ring)         // ring
)

class IOLSLIDARC16 : public rclcpp::Node {
  public:
    //----Parameter
    int port_msop;
    int port_difop;
    std::string frame_id;
    float distance_min;
    float distance_max;
    float azimuth_start;
    float azimuth_stop;
    //----Publisher
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_xyz;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_pointcloud_xyzir;
    //----Help
    HelpLogger logger;
    HelpUDP udp_msop;
    HelpUDP udp_difop;
    //----Thread
    std::thread thr_main;

    // Socket connection
    // -----------------
    uint8_t rx_buffer_msop[1206];
    uint8_t rx_buffer_difop[1206];

    // Lidar data
    // ----------
    float azimuth_cos[36000];
    float azimuth_sin[36000];
    float elevation_cos[16];
    float elevation_sin[16];
    uint8_t ring_index[16];
    pcl::PointCloud<pcl::PointXYZ> pointcloud_xyz;
    pcl::PointCloud<MyPointXYZIR> pointcloud_xyzir;

    typedef struct {
        uint8_t distance[2];
        uint8_t intensity;
    } msop_data_t;

    typedef struct {
        uint8_t flag[2];
        uint8_t azimuth[2];
        msop_data_t data[32];
    } msop_block_t;

    typedef struct {
        msop_block_t block[12];
        uint8_t timestamp[4];
        uint8_t factory[2];
    } msop_packet_t;

    IOLSLIDARC16() : Node("io_lslidar_c16") {
        //----Parameter
        this->declare_parameter("port_msop", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("port_difop", rclcpp::PARAMETER_INTEGER);
        this->declare_parameter("frame_id", rclcpp::PARAMETER_STRING);
        this->declare_parameter("distance_min", 0.1);
        this->declare_parameter("distance_max", 10.0);
        this->declare_parameter("azimuth_start", 0.0);
        this->declare_parameter("azimuth_stop", 360.0);
        this->declare_parameter("azimuth_step", 1.0);
        this->get_parameter("port_msop", port_msop);
        this->get_parameter("port_difop", port_difop);
        this->get_parameter("frame_id", frame_id);
        this->get_parameter("distance_min", distance_min);
        this->get_parameter("distance_max", distance_max);
        this->get_parameter("azimuth_start", azimuth_start);
        this->get_parameter("azimuth_stop", azimuth_stop);
        //----Publisher
        pub_pointcloud_xyz = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_xyz", 10);
        pub_pointcloud_xyzir = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud_xyzir", 10);
        //----Help
        if (!logger.init()) {
            rclcpp::shutdown();
        }
        if (!udp_msop.init_as_server(port_msop)) {
            rclcpp::shutdown();
        }
        if (!udp_difop.init_as_server(port_difop)) {
            rclcpp::shutdown();
        }
        //----Thread
        thr_main = std::thread(&IOLSLIDARC16::main, this);
    }

    ~IOLSLIDARC16() {
        if (thr_main.joinable()) {
            thr_main.join();
        }
    }

    void
    main() {
        logger.info("port_msop: %d", port_msop);
        logger.info("port_difop: %d", port_difop);
        logger.info("frame_id: %s", frame_id.c_str());
        logger.info("distance_min: %f", distance_min);
        logger.info("distance_max: %f", distance_max);
        logger.info("azimuth_start: %f", azimuth_start);
        logger.info("azimuth_stop: %f", azimuth_stop);

        for (int i = 0; i < 36000; i++) {
            azimuth_cos[i] = cos(i * M_PI / 18000);
            azimuth_sin[i] = sin(i * M_PI / 18000);
        }

        for (int i = 0; i < 16; i++) {
            elevation_cos[i * 2] = cos((-15 + i * 2) * M_PI / 180);
            elevation_cos[i * 2 + 1] = cos((1 + i * 2) * M_PI / 180);
            elevation_sin[i * 2] = sin((-15 + i * 2) * M_PI / 180);
            elevation_sin[i * 2 + 1] = sin((1 + i * 2) * M_PI / 180);
            ring_index[i * 2] = i;
            ring_index[i * 2 + 1] = 8 + i;
        }

        while (rclcpp::ok()) {
            if (udp_msop.recv(rx_buffer_msop, 1206) == 1206) {
                parse_msop();
            }
            if (udp_difop.recv(rx_buffer_difop, 1206) == 1206) {
                parse_difop();
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    void
    parse_msop() {
        msop_packet_t* packet = (msop_packet_t*)rx_buffer_msop;

        for (int i_block = 0; i_block < 12; i_block++) {
            // Check if packet header is correct
            static const uint8_t flag[2] = {0xFF, 0xEE};
            if (memcmp(packet->block[i_block].flag, flag, 2) != 0) {
                continue;
            }

            // Calculate azimuth difference between current and next/previous block
            static float azimuth_difference;
            if (i_block > 0) {
                uint16_t azimuth0 = *(uint16_t*)packet->block[i_block - 1].azimuth;
                uint16_t azimuth1 = *(uint16_t*)packet->block[i_block].azimuth;
                azimuth_difference = (float)((azimuth1 - azimuth0 + 36000) % 36000) / 100;
            } else {
                uint16_t azimuth0 = *(uint16_t*)packet->block[i_block].azimuth;
                uint16_t azimuth1 = *(uint16_t*)packet->block[i_block + 1].azimuth;
                azimuth_difference = (float)((azimuth1 - azimuth0 + 36000) % 36000) / 100;
            }

            // Calculate azimuth of current block
            static float azimuth_block;
            static float azimuth_block_old;
            azimuth_block_old = azimuth_block;
            azimuth_block = (float)(*(uint16_t*)packet->block[i_block].azimuth) / 100;

            for (int i_laser = 0; i_laser < 2; i_laser++) {
                for (int i_data = 0; i_data < 16; i_data++) {
                    // Calculate azimuth of current data
                    static float azimuth_data;
                    azimuth_data = azimuth_block + (azimuth_difference / 32) * (i_laser * 16 + i_data);
                    azimuth_data = azimuth_data >= 360 ? azimuth_data - 360 : azimuth_data;

                    // Calculate distance and intensity of current data
                    uint16_t distance_int = *(uint16_t*)packet->block[i_block].data[i_laser * 16 + i_data].distance;
                    uint8_t intensity_int = packet->block[i_block].data[i_laser * 16 + i_data].intensity;
                    float distance_float = (float)distance_int * 0.0025;
                    float intensity_float = (float)intensity_int / 255;

                    // Check if data is valid for publishing
                    if (distance_float < distance_min || distance_float > distance_max) {
                        continue;
                    } else if (azimuth_stop > azimuth_start
                               && (azimuth_data < azimuth_start || azimuth_data > azimuth_stop)) {
                        continue;
                    } else if (azimuth_stop < azimuth_start
                               && (azimuth_data < azimuth_start && azimuth_data > azimuth_stop)) {
                        continue;
                    }

                    // Filling point cloud data
                    int _i = azimuth_data * 100;
                    pcl::PointXYZ point_xyz;
                    MyPointXYZIR point_xyzir;
                    point_xyzir.x = point_xyz.x = distance_float * elevation_cos[i_data] * azimuth_cos[_i];
                    point_xyzir.y = point_xyz.y = distance_float * elevation_cos[i_data] * azimuth_sin[_i];
                    point_xyzir.z = point_xyz.z = distance_float * elevation_sin[i_data];
                    point_xyzir.intensity = intensity_float;
                    point_xyzir.ring = ring_index[i_data];
                    pointcloud_xyz.push_back(point_xyz);
                    pointcloud_xyzir.push_back(point_xyzir);
                }
            }

            if (azimuth_block < azimuth_block_old) {
                sensor_msgs::msg::PointCloud2 msg_pointcloud_xyz;
                sensor_msgs::msg::PointCloud2 msg_pointcloud_xyzir;
                pcl::toROSMsg(pointcloud_xyz, msg_pointcloud_xyz);
                pcl::toROSMsg(pointcloud_xyzir, msg_pointcloud_xyzir);
                msg_pointcloud_xyzir.header.frame_id = msg_pointcloud_xyz.header.frame_id = frame_id;
                msg_pointcloud_xyzir.header.stamp = msg_pointcloud_xyz.header.stamp = rclcpp::Clock().now();
                pub_pointcloud_xyz->publish(msg_pointcloud_xyz);
                pub_pointcloud_xyzir->publish(msg_pointcloud_xyzir);

                pointcloud_xyz.clear();
                pointcloud_xyzir.clear();
            }
        }
    }

    void
    parse_difop() {}
};

int
main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node_io_lslidar_c16 = std::make_shared<IOLSLIDARC16>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node_io_lslidar_c16);
    executor.spin();

    return 0;
}