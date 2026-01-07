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
    // 1. Extract the filename from the path
    size_t lastSlash = filename.find_last_of("/\\");
    std::string name = (lastSlash == std::string::npos) ? filename : filename.substr(lastSlash + 1);

    // --- ADD THIS: Remove the extension (e.g., .bag) ---
    size_t lastDot = name.find_last_of(".");
    if (lastDot != std::string::npos) {
        name = name.substr(0, lastDot);
    }
    // --------------------------------------------------

    // Now name is "eklenti24" instead of "eklenti24.bag"
    std::map<std::string, MapOffset> manual_registry = {
        {"merkez24", {-1.0, -3.0, 0.0}},
        {"eklenti24", {-1.0, -3.0, -0.6}},
        {"merkez34", {-21.0, -3.0, -0.6}},
        {"merkez33", {-24.0, 17.0, -0.6}},
        {"merkez42", {-38.0, 35.2, -1.2}},
        {"46ama45", {-45.5, -43.0, 0.0}},
        {"mag1", {-41.0, -83.0, 0.0}},
        {"mag2", {-41.0, -83.0, 0.0}},
        {"mag3", {-41.0, -84.0, 0.0}},
        {"mag11", {-43.0, -83, 0.0}},
        {"mag10", {-44, -83, 0.0}},
        {"mag5", {-40.5, -86.0, 1.0}},
        {"mag7", {-42.0, -85.0, 1.0}},
        {"mag8", {-44.0, -85, 0.0}},
        {"MAG", {-30.0, -93.0, 0.8}},
        {"DIS", {-1.0, -3.0, 0.0}},
        {"FIN", {-45.5, -53.9, 0.0}},
    };

    // Check if the filename exists exactly in the manual registry
    if (manual_registry.count(name)) {
        pos.x() = manual_registry[name].x;
        pos.y() = manual_registry[name].y;
        z_offset = manual_registry[name].z;
        return true;
    }

    // 3. Fallback: Standard ID extraction logic (e.g., "24")
    if (name.length() >= 2) {
        std::string id = name.substr(0, 2);

        std::map<std::string, MapOffset> registry = {
            {"11", {19.0,  57.0, 0.0}}, {"12", {19.0,  37.0, 0.0}}, {"13", {19.0,  17.0, 0.0}}, {"14", {19.0,  -3.0, 0.0}}, {"15", {21.0, -23.0, -0.0}},
            {"21", {-1.0,  57.0, 0.0}}, {"22", {-1.0,  37.0, -0}}, {"23", {-1.0,  17.0, -0.0}}, {"24", {-1, -3.0, 0.0}}, {"25", {-1.0, -23.0, 0.0}},
            {"31", {-21.0, 57.0, 0.0}}, {"32", {-21.0, 37.0, -0}}, {"33", {-21.0, 17.0, 0.0}}, {"34", {-21.0, -3.0, 0.0}}, {"35", {-21.0, -23, -0.0}},
            {"41", {-41.0, 57.0, 0.0}}, {"42", {-41.0, 37.0, -0}}, {"43", {-41.0, 17.0, 0.0}}, {"44", {-41.0, -3.0, -0.9}}, {"45", {-41.0, -23.0, 0.0}},
            {"51", {-61.0, 57.0, 0.0}}, {"52", {-63.0, 37.0, 0}}, {"53", {-61.0, 17.0, -0.0}}, {"54", {-61.0, -3.0, -0.0}}, {"55", {-61.0, -23.0, 0.0}},
            {"36", {-21.0, -43.0, 0.0}}, {"37", {-21, -63.0, 0}}, {"38", {-26.0, -87.5, -0.0}}, {"39", {-21.0, -103.0, -0.0}},
            {"46", {-46.5, -45.0, 0.6}}, {"47", {-41.0, -63.0, 0}}, {"48", {-46.4, -85, 0.5}}, {"49", {-41.0, -103.0, -0.0}},
            {"56", {-68, -45.0, 1.7}}, {"57", {-68.0, -66.0, 0.5}}, {"58", {-61.0, -83.0, -0.0}}, {"59", {-61.0, -103.0, -0.0}},
        };

        if (registry.count(id)) {
            pos.x() = registry[id].x;
            pos.y() = registry[id].y;
            z_offset = registry[id].z;
            return true;
        }
    }

    // 4. Default Fail Case
    ROS_WARN("Filename or ID '%s' not found in registry! Defaulting to 0.0.0", name.c_str());
    pos.x() = 0;
    pos.y() = 0;
    z_offset = 0;
    return true;
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
    //std::vector<std::string> baseNames = {"42", "32", "22", "52", "53", "54", "44", "23", "15", "14", "35", "34", "24"}; // Add 11, 12, 13... as needed
    //std::vector<std::string> baseNames = {"35", "33", "13", "15", "34", "23", "25", "14", "24"}; // Add 11, 12, 13... as needed
    //std::vector<std::string> baseNames = {"merkez42", "merkez33", "merkez34", "eklenti24", "merkez24", "46ama45"}; // Add 11, 12, 13... as needed
    //std::vector<std::string> baseNames = {"mag8", "mag7", "mag10", "mag5", "mag11", "mag3", "mag2", "mag1"}; // Add 11, 12, 13... as needed
    //std::vector<std::string> baseNames = {"DIS", "MAG"};
    std::vector<std::string> baseNames = {"FIN"};

    std::vector<std::string> standardBags, rawBags;
    for (const auto& b : baseNames) {
        standardBags.push_back("/home/semanur/maps/" + b + ".bag");
        rawBags.push_back("/home/semanur/maps/" + b + ".bag_raw");
    }

    mergeBags(standardBags, "/home/semanur/maps/combined.bag", "/elevation_mapping/elevation_map");
    mergeBags(rawBags, "/home/semanur/maps/combined.bag_raw", "/elevation_mapping/elevation_map_raw");

    return 0;
}