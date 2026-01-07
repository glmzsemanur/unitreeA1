#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>

class FastHeightNav {
public:
  FastHeightNav() : nh_("~") {
    nh_.param<double>("resolution", resolution_, 0.1);
    nh_.param<double>("height_threshold", height_threshold_, 0.0); 
    nh_.param<double>("min_z", min_z_, -2.0); 
    nh_.param<std::string>("robot_frame", robot_frame_, "base");
    nh_.param<std::string>("map_frame", map_frame_, "map");

    sub_ = nh_.subscribe("/elevation_mapping/elevation_map_raw", 1, &FastHeightNav::callback, this);
    map_sub_ = nh_.subscribe("/map", 1, &FastHeightNav::mapCallback, this);
    
    octo_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_3d", 1);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap_2d", 1);
    merged_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("merged_global_map", 1);

    ROS_INFO("Nav Node Active. Using Elevation Map Ground Reference for Slope Handling.");
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
      global_map_ = *msg;
      map_received_ = true;
      map_sub_.shutdown(); 
  }

  void callback(const grid_map_msgs::GridMapConstPtr& message) {
    if (!map_received_) return;

    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*message, map);

    tf::StampedTransform transform;
    double rx, ry, rz;
    try {
        listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
        rx = transform.getOrigin().x();
        ry = transform.getOrigin().y();
        rz = transform.getOrigin().z();
    } catch (tf::TransformException &ex) { return; }

    // --- NEW: GROUND TRUTH Z FROM ELEVATION MAP ---
    double ground_z = rz; // Default to TF Z
    grid_map::Position robot_pos(rx, ry);
    if (map.isInside(robot_pos)) {
        float map_z = map.atPosition("elevation_smooth", robot_pos);
        if (std::isfinite(map_z)) {
            ground_z = map_z; // Trust the sensor under the robot
        }
    }

    octomap::OcTree tree(resolution_);
    ros::Time timestamp = ros::Time::now();

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        if (!map.isValid(*it, "elevation_smooth")) continue;
        float cell_z = map.at("elevation_smooth", *it);
        
        grid_map::Position pos; 
        map.getPosition(*it, pos);

        // Calculate height diff relative to local ground_z instead of global rz
        if (std::abs(cell_z - ground_z) > height_threshold_) {
            for (double z = cell_z; z >= min_z_; z -= resolution_) 
                tree.updateNode(octomap::point3d(pos.x(), pos.y(), z), true);
        } else {
            tree.updateNode(octomap::point3d(pos.x(), pos.y(), cell_z), false);
        }
    }

    double g_res = global_map_.info.resolution;
    double g_ox = global_map_.info.origin.position.x;
    double g_oy = global_map_.info.origin.position.y;

    int robot_cell_x = std::floor((rx - g_ox) / g_res);
    int robot_cell_y = std::floor((ry - g_oy) / g_res);
    int local_width_cells = std::round(30.0 / g_res);
    
    // Pass ground_z to costmap generator
    nav_msgs::OccupancyGrid local_grid = generateAlignedCostmap(tree, robot_cell_x, robot_cell_y, ground_z, local_width_cells, timestamp);
    costmap_pub_.publish(local_grid);

    updateAndPublishGlobalMap(local_grid, robot_cell_x, robot_cell_y, timestamp);

    octomap_msgs::Octomap octo_msg;
    octo_msg.header.frame_id = map_frame_;
    octo_msg.header.stamp = timestamp;
    if (octomap_msgs::binaryMapToMsg(tree, octo_msg)) octo_pub_.publish(octo_msg);
  }

private:
  nav_msgs::OccupancyGrid generateAlignedCostmap(const octomap::OcTree& tree, int r_cell_x, int r_cell_y, double ground_z, int cells, ros::Time ts) {
    nav_msgs::OccupancyGrid grid;
    grid.header.frame_id = map_frame_;
    grid.header.stamp = ts;
    grid.info.resolution = global_map_.info.resolution;
    grid.info.width = cells;
    grid.info.height = cells;
    
    int start_cell_x = r_cell_x - (cells / 2);
    int start_cell_y = r_cell_y - (cells / 2);
    grid.info.origin.position.x = global_map_.info.origin.position.x + (start_cell_x * grid.info.resolution);
    grid.info.origin.position.y = global_map_.info.origin.position.y + (start_cell_y * grid.info.resolution);
    grid.info.origin.orientation.w = 1.0;

    grid.data.resize(cells * cells, -1);

    for (int y = 0; y < cells; ++y) {
        for (int x = 0; x < cells; ++x) {
            double world_x = grid.info.origin.position.x + (x + 0.5) * grid.info.resolution;
            double world_y = grid.info.origin.position.y + (y + 0.5) * grid.info.resolution;

            bool has_data = false;
            double max_z_obs = -999.0;

            // Search vertically around our local ground reference
            for (double z = ground_z + 1.0; z >= min_z_; z -= resolution_) {
                const octomap::OcTreeNode* n = tree.search(world_x, world_y, z);
                if (n) {
                    has_data = true;
                    if (tree.isNodeOccupied(n)) {
                        max_z_obs = std::max(max_z_obs, z);
                        break; 
                    }
                }
            }

            int idx = y * cells + x;
            if (max_z_obs > -900.0) {
                // Obstacle logic relative to sensor ground height
                float diff = std::abs(max_z_obs - ground_z);
                // If it is roughly at robot base height (with your 0.33m offset), mark as free or obstacle
                if (diff <= 0.4) grid.data[idx] = 0; // Flat enough relative to legs
                else grid.data[idx] = 100;           // Actual obstacle
            } else if (has_data) {
                grid.data[idx] = 0; 
            } else {
                grid.data[idx] = -1; 
            }
        }
    }
    return grid;
  }

  void updateAndPublishGlobalMap(const nav_msgs::OccupancyGrid& local_grid, int rx_cell, int ry_cell, ros::Time ts) {
      global_map_.header.stamp = ts;
      double res = global_map_.info.resolution;

      int off_x = std::round((local_grid.info.origin.position.x - global_map_.info.origin.position.x) / res);
      int off_y = std::round((local_grid.info.origin.position.y - global_map_.info.origin.position.y) / res);

      int half_5m_cells = std::round(2 / res);
      int min_occ_x = rx_cell - half_5m_cells;
      int max_occ_x = rx_cell + half_5m_cells;
      int min_occ_y = ry_cell - half_5m_cells;
      int max_occ_y = ry_cell + half_5m_cells;

      for (int y = 0; y < (int)local_grid.info.height; ++y) {
          for (int x = 0; x < (int)local_grid.info.width; ++x) {
              int gx = off_x + x;
              int gy = off_y + y;

              if (gx >= 0 && gx < (int)global_map_.info.width && gy >= 0 && gy < (int)global_map_.info.height) {
                  int8_t val = local_grid.data[y * local_grid.info.width + x];
                  if (val == 0) {
                      global_map_.data[gy * global_map_.info.width + gx] = 0;
                  } 
                  else if (val == 100) {
                      if (gx >= min_occ_x && gx <= max_occ_x && gy >= min_occ_y && gy <= max_occ_y) {
                          global_map_.data[gy * global_map_.info.width + gx] = 100;
                      }
                  }
              }
          }
      }
      merged_pub_.publish(global_map_);
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_, map_sub_;
  ros::Publisher octo_pub_, costmap_pub_, merged_pub_; 
  tf::TransformListener listener_;
  nav_msgs::OccupancyGrid global_map_;
  bool map_received_ = false;
  double resolution_, height_threshold_, min_z_;
  std::string robot_frame_, map_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_to_map_merger");
  FastHeightNav node;
  ros::spin();
  return 0;
}