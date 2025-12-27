#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <string>
#include <vector>

using namespace grid_map;

// Function to calculate position from filename (e.g., "24.bag" -> x:-21, y:-3)
#include <map>

// Define a structure to hold our 3D offsets
struct MapOffset {
    double x;
    double y;
    double z;
};

bool getPositionFromFilename(std::string filename, Position& pos, float& z_offset) {
    // 1. Extract the ID (e.g., "24")
    size_t lastSlash = filename.find_last_of("/\\");
    std::string name = (lastSlash == std::string::npos) ? filename : filename.substr(lastSlash + 1);
    
    // Get first two characters (the ID)
    if (name.length() < 2) return false;
    std::string id = name.substr(0, 2);

    // 2. Define your Lookup Table (Add all 25 regions here)
    // Format: {"ID", {X, Y, Z_Offset}}
    std::map<std::string, MapOffset> registry = {
        {"11", {19.0,  57.0, 0.0}}, {"12", {19.0,  37.0, 0.0}}, {"13", {19.0,  17.0, 0.0}}, {"14", {19.0,  -3.0, -0.5}}, {"15", {21.0, -23.0, -0.8}},
        {"21", {-1.0,  57.0, 0.0}}, {"22", {-1.0,  37.0, -1.25}}, {"23", {-1.0,  17.0, -0.3}}, {"24", {-1.0, -3.0, 0.0}}, {"25", {-1.0, -23.0, 0.0}},
        {"31", {-21.0, 57.0, 0.0}}, {"32", {-21.0, 37.0, -1.2}}, {"33", {-21.0, 17.0, 0.0}}, {"34", {-21.2, -3.0, -0.15}}, {"35", {-21.0, -25.5, -0.4}},
        {"41", {-41.0, 57.0, 0.0}}, {"42", {-41.0, 37.0, -1.3}}, {"43", {-41.0, 17.0, 0.0}}, {"44", {-41.0, -3.0, -0.5}}, {"45", {-41.0, -23.0, 0.0}},
        {"51", {-61.0, 57.0, 0.0}}, {"52", {-63.0, 37.0, -1.45}}, {"53", {-63.0, 17.0, -1.2}}, {"54", {-61.0, -3.0, -1.7}}, {"55", {-61.0, -23.0, 0.0}}
    };

    // 3. Search for the ID in our registry
    if (registry.count(id)) {
        pos.x() = registry[id].x;
        pos.y() = registry[id].y;
        z_offset = registry[id].z; // Return the Z offset to the main loop
        return true;
    }

    ROS_WARN("ID %s not found in registry!", id.c_str());
    return false;
}

void mergeBags(const std::vector<std::string>& inputBags, const std::string& outputName, const std::string& MAP_TOPIC) {
    GridMap combinedMap;
    bool isFirst = true;

    for (const auto& path : inputBags) {
        rosbag::Bag bag;
        try {
            bag.open(path, rosbag::bagmode::Read);
        } catch (...) { continue; }

        rosbag::View view(bag, rosbag::TopicQuery(MAP_TOPIC));

        for (rosbag::MessageInstance const m : view) {
    grid_map_msgs::GridMap::ConstPtr msg = m.instantiate<grid_map_msgs::GridMap>();
    if (msg != nullptr) {
        GridMap submap;
        GridMapRosConverter::fromMessage(*msg, submap);

        Position calculatedPos;
        float z_offset = 0.0; // Declare it here

        // Pass the third argument (z_offset) here
        if (getPositionFromFilename(path, calculatedPos, z_offset)) {
            submap.setPosition(calculatedPos);
            
            // Apply the shift using the .array() method we fixed earlier
            submap.get("elevation").array() += z_offset;

            if (isFirst) {
                combinedMap = submap;
                combinedMap.setFrameId("odom");
                isFirst = false;
            } else {
                combinedMap.addDataFrom(submap, true, true, true);
            }
            ROS_INFO("Merged %s: X=%f, Y=%f, Z_offset=%f", path.c_str(), calculatedPos.x(), calculatedPos.y(), z_offset);
        }
    }
}
        bag.close();
    }

    if (isFirst) return; // No maps were loaded

    rosbag::Bag outBag;
    outBag.open(outputName, rosbag::bagmode::Write);
    grid_map_msgs::GridMap outMsg;
    GridMapRosConverter::toMessage(combinedMap, outMsg);
    outMsg.info.header.stamp = ros::Time(1600000000); 
    outBag.write(MAP_TOPIC, outMsg.info.header.stamp, outMsg);
    outBag.close();
    ROS_INFO("Saved: %s", outputName.c_str());
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "grid_map_matrix_merger");
    ros::NodeHandle nh;

    // List all your maps here
    std::vector<std::string> baseNames = {"42", "32", "22", "52", "53", "54", "44", "23", "15", "14", "35", "34", "24"}; // Add 11, 12, 13... as needed
    
    std::vector<std::string> standardBags, rawBags;
    for (const auto& b : baseNames) {
        standardBags.push_back("/home/semanur/maps/" + b + ".bag");
        rawBags.push_back("/home/semanur/maps/" + b + ".bag_raw");
    }

    mergeBags(standardBags, "/home/semanur/maps/combined.bag", "/elevation_mapping/elevation_map");
    mergeBags(rawBags, "/home/semanur/maps/combined.bag_raw", "/elevation_mapping/elevation_map_raw");

    return 0;
}