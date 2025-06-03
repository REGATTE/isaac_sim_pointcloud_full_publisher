/**
 * @file lidar_ring_converter.cpp
 * @brief ROS 2 Node for converting LiDAR point cloud data to include ring and time fields.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <yaml-cpp/yaml.h>

// Global variables for topic names
std::string lidar_topic;
std::string output_topic;

/**
 * @struct PointXYZIRT
 * @brief Custom point type to store x, y, z, intensity, ring, and time fields.
 */
struct PointXYZIRT
{
    PCL_ADD_POINT4D                  ///< 3D point coordinates (x, y, z)
    PCL_ADD_INTENSITY                ///< Intensity value
    uint16_t ring;                   ///< Ring number
    // float time;                      ///< Time offset
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

// Register custom point type
POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRT,
    (float, x, x) (float, y, y) (float, z, z) (float, intensity, intensity)
    (uint16_t, ring, ring) // (float, time, time)
)

/**
 * @class LidarRingConverter
 * @brief ROS 2 Node for processing and augmenting LiDAR point cloud data.
 */
class LidarRingConverter : public rclcpp::Node
{
public:
    /**
     * @brief Constructor for the LidarRingConverter class.
     */
    LidarRingConverter() : Node("lidar_ring_converter")
    {
        // Declare Launch Parameters
        this->declare_parameter<std::string>("robot_namespace", "");
        this->declare_parameter<std::string>("frame_id", "XT_16_10hz");  // Add frame_id parameter

        // Decalare Configurable Parameters
        this->declare_parameter<int>("N_SCAN", 128);
        this->declare_parameter<int>("Horizon_SCAN", 1800);
        this->declare_parameter<float>("fov_bottom", -25.0);
        this->declare_parameter<float>("fov_top", 15.0);
        this->declare_parameter<float>("min_dist", 1.0);
        this->declare_parameter<float>("max_dist", 100.0);

        // Get parameters
        std::string robot_namespace = this->get_parameter("robot_namespace").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();  // Get frame_id parameter
        N_SCAN = this->get_parameter("N_SCAN").as_int();
        Horizon_SCAN = this->get_parameter("Horizon_SCAN").as_int();
        fov_bottom = this->get_parameter("fov_bottom").as_double();
        fov_top = this->get_parameter("fov_top").as_double();
        min_dist = this->get_parameter("min_dist").as_double();
        max_dist = this->get_parameter("max_dist").as_double();

        // Define topic names based on namespace
        lidar_topic = robot_namespace.empty() ? "/scan_3d" : "/" + robot_namespace + "/scan_3d";
        output_topic = robot_namespace.empty() ? "/scan3D_with_rings" : "/" + robot_namespace + "/scan3D_with_rings";

        // Create subscriber and publisher
        subPC_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic, 10, std::bind(&LidarRingConverter::lidarHandle, this, std::placeholders::_1));

        pubPC_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
    }

private:
    std::string frame_id_;  // Add frame_id member variable
    int N_SCAN;                      ///< Number of vertical beams
    int Horizon_SCAN;                ///< Horizontal resolution
    float fov_bottom;                ///< Bottom of vertical FoV
    float fov_top;                   ///< Top of vertical FoV
    float min_dist;                  ///< Minimum distance threshold
    float max_dist;                  ///< Maximum distance threshold
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subPC_; ///< Subscriber
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pubPC_;   ///< Publisher

    /**
     * @brief Publishes the modified point cloud data.
     * @param new_pc Processed point cloud.
     * @param old_msg Original point cloud message header.
     */
    template<typename T>
    void publishPoints(T &new_pc, const sensor_msgs::msg::PointCloud2 &old_msg) {
        new_pc->is_dense = true;

        // Convert to ROS 2 message
        sensor_msgs::msg::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg.header;
        pc_new_msg.header.frame_id = frame_id_;  // Use dynamic frame_id
        pubPC_->publish(pc_new_msg);
    }

    /**
     * @brief Callback function to process incoming point cloud data.
     * @param pc_msg Pointer to incoming point cloud message.
     */
    void lidarHandle(const sensor_msgs::msg::PointCloud2::SharedPtr pc_msg) {
        // Convert incoming data
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>());
        pcl::fromROSMsg(*pc_msg, *pc);

        if (pc->points.empty()) {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud!");
            return;
        }

        // Extract scan start time from the header
        double scan_start_time = pc_msg->header.stamp.sec + pc_msg->header.stamp.nanosec * 1e-9;

        // Debug log for scan start time
        RCLCPP_INFO(this->get_logger(), "Processing point cloud with timestamp: %f", scan_start_time);

        // LiDAR parameters for vertical FoV
        float ang_res_y = (fov_top - fov_bottom) / (N_SCAN - 1);  // Vertical resolution

        // Process each point
        for (size_t point_id = 0; point_id < pc->points.size(); ++point_id) {

            PointXYZIRT new_point;
            new_point.x = pc->points[point_id].x;
            new_point.y = pc->points[point_id].y;
            new_point.z = pc->points[point_id].z;

            // Calculate Intensity by Distance
            float distance = sqrt(new_point.x * new_point.x +
                                  new_point.y * new_point.y +
                                  new_point.z * new_point.z);

            new_point.intensity = std::min(255.0f, distance * 10.0f);

            // Calculate ring ID
            float verticalAngle = atan2(new_point.z, sqrt(new_point.x * new_point.x + new_point.y * new_point.y)) * 180.0 / M_PI;
            float rowIdn = (verticalAngle - fov_bottom) / ang_res_y;

            if (rowIdn < 0 || rowIdn >= N_SCAN) {
                continue;  // Skip points outside the valid FoV
            }

            new_point.ring = static_cast<uint16_t>(rowIdn);
            // Calculate time for each point based on scan start time
            // float point_relative_time = static_cast<float>(point_id) / static_cast<float>(pc->points.size());
            // new_point.time = scan_start_time + point_relative_time;

            pc_new->points.push_back(new_point);
        }
        publishPoints(pc_new, *pc_msg);
    }
};

/**
 * @brief Main entry point for the node.
 */
int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarRingConverter>();
    RCLCPP_INFO(node->get_logger(), "Listening to lidar topic ......");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
