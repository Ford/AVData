# Ford AV Dataset Tutorial
This Tutorial contains installation instructions for the packages released along with Ford Multi AV Dataset. For more details please visit the website.

## Website

To get more details about the Ford AV Dataset, please visit [avdata.ford.com](https://avdata.ford.com/)

## System Requirements

This code repository has been tested on a Laptop containing 32 gb RAM, Ubuntu 16.04 and ROS Kinetic.

## Installation

### Dependencies

These packages depend on
* python 2.x
* Standard ROS packages (roscpp, rospy, sensor_msgs, std_msgs, tf2_ros)
* rviz
* pcl_conversions
* velodyne

Clone the latest version from this repository into your catkin workspace and compile the packages using the following snippet. If you do not have a catkin workspace, please read [this](http://wiki.ros.org/catkin/Tutorials/create_a_workspace "Catkin tutorial") tutorial to create one.

```
cd catkin_ws/src
git clone https://github.com/Ford/AVData.git
cd ../
catkin_make
source devel/setup.bash
```

## Package Description

* **ford_demo** - This package contains the launch files, rviz plugins and helper scripts. The multi_lidar_convert launch file uses the velodyne package.
* **fusion_description** - This package contains the Ford fusion URDF for visualization in Rviz. The physical parameters mentioned in the URDF are just for representation and visualization and do not represent the actual properties of a Ford Fusion vehicle.
* **map_loader** - Map loader package loads the ground plane reflectivity and 3D pointcloud maps as ROS PointCloud2 messages. The package subscribes to vehicle pose to decide what section of the map to display. Various dynamic parameters include:
  * *publish_rate* - Rate at which the each PointCloud2 map is published (hz)
  * *pcd_topic* - Name of the 3d pointcloud map or ground plane reflectivity map topic
  * *pose_topic* -  Name of the pose topic to determine vehicle location (default: */pose_ground_truth*)
  * *neighbor_dist* - The radius lookup to publish map tiles (m). A value of 128 means the publisher will publish map tiles whose origin lies within 128m (euclidean distance) of the origin of the tile the vehicle is in.

  Please note that the poinclouds are computation intensive in general. If you're running a 16gb system, the visualization could lag due to large pointclouds. Tune the  *publish_rate, neighbor_dist* parameters to optimize based on your application.

## Dataset Download

To get with quickly started, download the [sample data](https://ford-multi-av-seasonal.s3-us-west-2.amazonaws.com/Sample-Data.tar.gz "SampleData").
In order to download more data, visit the [download](https://avdata.ford.com/downloads/default.aspx "Downloads") page of the website

## Usage

In order to run the demo, you will need the rosbag, maps and the calibration files. These can be downloaded [here](https://avdata.ford.com/downloads/default.aspx). Once you have these files, run the demo launch file using

```
roslaunch ford_demo demo.launch map_dir:=/path/to/map/folder/ calibration_dir:=/path/to/calibration/folder/
```

In a new terminal, run the rosbag file

```
rosbag play /path/to/your/bag/file/name.bag
```

To view the live lidar pointcloud, in a new terminal, run the launch file

```
roslaunch ford_demo multi_lidar_convert.launch
```

![picture](https://github.com/Ford/AVData/blob/master/ford_demo/doc/rviz.gif "rviz_gif")

In order to convert bag files to human readable csv files, we provide a python script ```bag_to_csv.py```. This python script uses a yaml config file to determine which topics need to be converted to csv files. A sample config.yaml file is provided in the ford_demo/scripts folder. In order to convert a bag file, run the the following command

```
python bag_to_csv /path/to/your/bag/file/name.bag /path/to/the/config/file/name.yaml
```

## Citation

If you use this dataset, please cite our paper mentioned below.

## Paper

Coming soon...
