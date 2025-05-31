/**
 * @file lidar_ring_converter.cpp
 * @brief ROS 1 Node for converting LiDAR point cloud data to include ring and time fields.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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
 * @brief ROS 1 Node for processing and augmenting LiDAR point cloud data.
 */
class LidarRingConverter
{
public:
    /**
     * @brief Constructor for the LidarRingConverter class.
     */
    LidarRingConverter(ros::NodeHandle& nh) : nh_(nh)
    {
        // Get parameters
        std::string robot_namespace;
        nh_.param<std::string>("robot_namespace", robot_namespace, "scout_1_1");
        nh_.param<int>("N_SCAN", N_SCAN, 16);
        nh_.param<int>("Horizon_SCAN", Horizon_SCAN, 1800);
        nh_.param<float>("fov_bottom", fov_bottom, -16.87);
        nh_.param<float>("fov_top", fov_top, 16.87);
        nh_.param<float>("min_dist", min_dist, 0.1);
        nh_.param<float>("max_dist", max_dist, 120.0);

        // Define topic names based on namespace
        lidar_topic = "/" + robot_namespace + "/PointCloud2";
        output_topic = "/" + robot_namespace + "/scan3D_with_rings";

        // Create subscriber and publisher
        subPC_ = nh_.subscribe(lidar_topic, 10, &LidarRingConverter::lidarHandle, this);
        pubPC_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic, 10);
    }

private:
    ros::NodeHandle nh_;
    int N_SCAN;                      ///< Number of vertical beams
    int Horizon_SCAN;                ///< Horizontal resolution
    float fov_bottom;                ///< Bottom of vertical FoV
    float fov_top;                   ///< Top of vertical FoV
    float min_dist;                  ///< Minimum distance threshold
    float max_dist;                  ///< Maximum distance threshold
    ros::Subscriber subPC_;          ///< Subscriber
    ros::Publisher pubPC_;           ///< Publisher

    /**
     * @brief Publishes the modified point cloud data.
     * @param new_pc Processed point cloud.
     * @param old_msg Original point cloud message header.
     */
    template<typename T>
    void publishPoints(T &new_pc, const sensor_msgs::PointCloud2 &old_msg) {
        new_pc->is_dense = true;

        // Convert to ROS message
        sensor_msgs::PointCloud2 pc_new_msg;
        pcl::toROSMsg(*new_pc, pc_new_msg);
        pc_new_msg.header = old_msg.header;
        pc_new_msg.header.frame_id = "LiDAR"; // Set frame ID
        pubPC_.publish(pc_new_msg);
    }

    /**
     * @brief Callback function to process incoming point cloud data.
     * @param pc_msg Pointer to incoming point cloud message.
     */
    void lidarHandle(const sensor_msgs::PointCloud2::ConstPtr& pc_msg) {
        // Convert incoming data
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<PointXYZIRT>::Ptr pc_new(new pcl::PointCloud<PointXYZIRT>());
        pcl::fromROSMsg(*pc_msg, *pc);

        if (pc->points.empty()) {
            ROS_WARN("Received empty point cloud!");
            return;
        }

        // Extract scan start time from the header
        double scan_start_time = pc_msg->header.stamp.toSec();

        // Debug log for scan start time
        ROS_INFO("Processing point cloud with timestamp: %f", scan_start_time);

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
            pc_new->points.push_back(new_point);
        }
        publishPoints(pc_new, *pc_msg);
    }
};

/**
 * @brief Main entry point for the node.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "lidar_ring_converter");
    ros::NodeHandle nh;
    LidarRingConverter converter(nh);
    ROS_INFO("Listening to lidar topic ......");
    ros::spin();
    return 0;
}
