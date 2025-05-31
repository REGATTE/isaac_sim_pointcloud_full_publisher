# ISAAC Sim PCD Converter ROS 1 Package

## Overview
This ROS 1 package processes LiDAR point cloud data to include additional fields such as **ring**, **intensity** and **time**, for ISAAC SIM. It dynamically subscribes to a specified topic for input data, computes the ring ID, intensity and time offset for each point, and republishes the augmented point cloud to an output topic.

The package supports dynamic configuration of parameters like the number of vertical beams (**N_SCAN**) and horizontal resolution (**Horizon_SCAN**) through launch file arguments.

---

## Installation

```bash
sudo xargs -a requirements.txt apt install -y
```

## Running the Node

To launch the node with dynamic parameters [Run this after the sim is started]: 
```bash
roslaunch isaac_sim_pointcloud_full_publisher full_pcd_pub.launch robot_namespace:=scout config_file:=helios_16p.yaml
```

---

## Output

<p align="center">
  <img src="Images/before.png" width="45%" />
  <img src="Images/after.png" width="45%" />
</p>


## Dependencies
- ROS 1 (Noetic)
- roscpp
- sensor_msgs
- std_msgs
- PCL (Point Cloud Library)
- pcl_ros
- pcl_conversions

---

## Topics
| Topic Name                                      | Type                                         | Role       |
|-------------------------------------------------|---------------------------------------------|------------|
| `/robot_namespace/PointCloud2`                  | `sensor_msgs/msg/PointCloud2`               | Subscriber |
| `/robot_namespace/scan3D_with_rings`            | `sensor_msgs/msg/PointCloud2`               | Publisher  |

Note: Replace `robot_namespace` with the desired namespace using the launch file argument.

---

## Parameters
| Launch Parameter Name      | Default Value | Description                                                     |
|---------------------|---------------|-----------------------------------------------------------------|
| `robot_namespace`   | `robot_x`     | Namespace of the robot used in the topic names.                 |

### Configuration

The package comes with pre-configured parameter files for different LiDAR models in the `Params` directory:
- `helios_16p.yaml`: Configuration for Ouster OS1-16
- `velodyne_vls_128.yaml`: Configuration for Velodyne VLS-128

---

## Supported LiDAR Configuration
This package is tested with the following LiDAR configuration:
- **Beams**: 128
- **Points per Scan**: 2048
- **Frame ID**: "LiDAR"
- **Ring Field**: Calculated dynamically based on vertical angle.

---

## Building the Package
```bash
catkin build isaac_sim_pointcloud_full_publisher
```

---

## Example Visualization
To visualize the output point cloud with the `ring` field:
```bash
rviz
```
- Add a **PointCloud2** display.
- Set the topic to `/robot_namespace/scan3D_with_rings`.
- Use **Color Transformer** -> **AxisColor** to view different ring IDs.

---

## License
This package is released under the Apache 2.0 License.
