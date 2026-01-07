#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

class GridMapToOctomap {
public:
    GridMapToOctomap(){
        ros::NodeHandle pnh("~");
        // Load Parameters using the private handle
        pnh.param<std::string>("input_topic", input_topic_, "/elevation_mapping/elevation_map_raw");
        pnh.param<std::string>("elevation_layer", elevation_layer_, "elevation_smooth");
        pnh.param<double>("resolution", res_, 0.2);           
        pnh.param<double>("floor_height", floor_height_, -2.0); 
        pnh.param<std::string>("frame_id", frame_id_, "map");

        // Publishers & Subscribers
        grid_map_sub_ = nh_.subscribe(input_topic_, 1, &GridMapToOctomap::callback, this);
        octomap_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap", 1, true);
        
        ROS_INFO("Grid Map to Octomap node initialized. Resolution: %f", res_);
    }

    void callback(const grid_map_msgs::GridMap& msg) {
        grid_map::GridMap map;
        grid_map::GridMapRosConverter::fromMessage(msg, map);

        // Create an OcTree with the user-defined resolution
        octomap::OcTree* tree = new octomap::OcTree(res_);

        for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
            const grid_map::Index index(*it);
            float elevation = map.at(elevation_layer_, index);

            if (std::isnan(elevation)) continue;

            grid_map::Position position;
            map.getPosition(index, position);

            // Create a vertical column of occupied voxels from floor to elevation
            for (double z = floor_height_; z <= elevation; z += res_) {
                octomap::point3d endpoint(position.x(), position.y(), z);
                tree->updateNode(endpoint, true); // true = occupied
            }
        }

        // Prepare message
        octomap_msgs::Octomap output_msg;
        output_msg.header.frame_id = frame_id_;
        output_msg.header.stamp = ros::Time::now();
        
        if (octomap_msgs::fullMapToMsg(*tree, output_msg)) {
            octomap_pub_.publish(output_msg);
        }

        delete tree;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber grid_map_sub_;
    ros::Publisher octomap_pub_;
    std::string input_topic_, elevation_layer_, frame_id_;
    double res_, floor_height_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_to_octomap");
    GridMapToOctomap node;
    ros::spin();
    return 0;
}
