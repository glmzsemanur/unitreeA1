#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

class GridMapToOctomap {
public:
    GridMapToOctomap() : tfListener(tfBuffer) { // Initialize TF listener
        ros::NodeHandle nh;
        ros::NodeHandle pnh("~");

        // Load Parameters
        pnh.param<std::string>("input_topic", input_topic_, "/elevation_mapping/elevation_map_raw");
        pnh.param<std::string>("elevation_layer", elevation_layer_, "elevation");
        pnh.param<double>("resolution", res_, 0.2);           
        pnh.param<double>("floor_height", floor_height_, -2.0); 
        pnh.param<std::string>("frame_id", frame_id_, "map");

        // Publishers & Subscribers
        grid_map_sub_ = nh.subscribe(input_topic_, 1, &GridMapToOctomap::callback, this);
        octomap_pub_ = nh.advertise<octomap_msgs::Octomap>("octomap", 1, true);
        projected_map_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("projected_map", 1, true);
        pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("octomap_pcl", 1, true);
        
        ROS_INFO("Node started. Converting %s to Octomap and OccupancyGrid", input_topic_.c_str());
    }

    void callback(const grid_map_msgs::GridMap& msg) {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(msg, map);

        // 1. Get Robot Z for relative height filtering
        double robot_z = 0.0;
        try {
            geometry_msgs::TransformStamped tf_stamped;
            tf_stamped = tfBuffer.lookupTransform(frame_id_, "base", ros::Time(0));
            robot_z = tf_stamped.transform.translation.z;
        } catch (tf2::TransformException &ex) {
            ROS_WARN_THROTTLE(10, "TF lookup failed: %s", ex.what());
        }

        // 2. Process Octomap (Requirement #1)
        octomap::OcTree* tree = new octomap::OcTree(res_);
        map.add("temp_occ", 0.0);

        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            float elevation = map.at(elevation_layer_, index);

            if (std::isnan(elevation)) continue;

            grid_map::Position position;
            map.getPosition(index, position);

            // Populate Octree
            for (double z = floor_height_; z <= elevation; z += res_) {
                tree->updateNode(octomap::point3d(position.x(), position.y(), z), true);
            }

            // Populate Navigation Layer (Requirement #3/4)
            // Mark as obstacle if height is above robot feet + 10cm
            if (elevation > (robot_z + 0.1) && elevation < (robot_z + 2.0)) {
                map.at("temp_occ", *it) = 100.0;
            }
        }

        // 3. Publish Octomap
        octomap_msgs::Octomap octo_msg;
        octo_msg.header.frame_id = frame_id_;
        octo_msg.header.stamp = ros::Time::now();
        if (octomap_msgs::fullMapToMsg(*tree, octo_msg)) {
            octomap_pub_.publish(octo_msg);
        }

        // 4. Publish OccupancyGrid (Requirement #3/4)
        nav_msgs::OccupancyGrid occ_msg;
        grid_map::GridMapRosConverter::toOccupancyGrid(map, "temp_occ", 0.0, 100.0, occ_msg);
        projected_map_pub_.publish(occ_msg);


        pcl::PointCloud<pcl::PointXYZ> pcl_cloud;
        for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
            if (tree->isNodeOccupied(*it)) {
                pcl_cloud.push_back(pcl::PointXYZ(it.getX(), it.getY(), it.getZ()));
            }
        }

        sensor_msgs::PointCloud2 cloud_msg;
        pcl::toROSMsg(pcl_cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id_;
        cloud_msg.header.stamp = ros::Time::now();
        pc_pub_.publish(cloud_msg);
        delete tree;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber grid_map_sub_;
    ros::Publisher octomap_pub_, projected_map_pub_, pc_pub_; // Added pc_pub_ here
    
    std::string input_topic_, elevation_layer_, frame_id_;
    double res_, floor_height_;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "elevation_to_octo");
    GridMapToOctomap node;
    ros::spin();
    return 0;
}