#include <ros/ros.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/transform_listener.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <boost/foreach.hpp>

class FastHeightNav {
public:
  FastHeightNav() : nh_("~") {
    // Parameters
    nh_.param<double>("resolution", resolution_, 0.1);
    nh_.param<double>("min_z", min_z_, -2.0); 
    nh_.param<std::string>("robot_frame", robot_frame_, "base");
    nh_.param<std::string>("map_frame", map_frame_, "map");
    nh_.param<std::string>("map_save_path", map_save_path_, "/tmp/global_map_persistence.bag");

    // 1. Load existing map from disk
    if (loadMapFromDisk()) {
        ROS_INFO("Persistent map loaded from %s", map_save_path_.c_str());
        map_received_ = true; 
    }

    sub_ = nh_.subscribe("/elevation_mapping/elevation_map_raw", 1, &FastHeightNav::callback, this);
    if (!map_received_) {
        map_sub_ = nh_.subscribe("/map", 1, &FastHeightNav::mapCallback, this);
    }
    
    octo_pub_ = nh_.advertise<octomap_msgs::Octomap>("octomap_3d", 1);
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap_2d", 1);
    merged_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("merged_global_map", 1);

    ROS_INFO("Nav Node Active. Directional Overwrite (Forward Only) enabled.");
  }

  bool loadMapFromDisk() {
      try {
          rosbag::Bag bag;
          bag.open(map_save_path_, rosbag::bagmode::Read);
          rosbag::View view(bag, rosbag::TopicQuery("global_map"));
          BOOST_FOREACH(rosbag::MessageInstance const m, view) {
              nav_msgs::OccupancyGrid::ConstPtr s = m.instantiate<nav_msgs::OccupancyGrid>();
              if (s != nullptr) {
                  global_map_ = *s;
                  bag.close();
                  return true;
              }
          }
          bag.close();
      } catch (std::exception &e) {
          ROS_WARN("No persistent map found: %s", e.what());
      }
      return false;
  }

  void saveMapToDisk() {
      try {
          rosbag::Bag bag;
          bag.open(map_save_path_, rosbag::bagmode::Write);
          bag.write("global_map", ros::Time::now(), global_map_);
          bag.close();
      } catch (std::exception &e) {
          ROS_ERROR("Failed to save map: %s", e.what());
      }
  }

  void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg) {
      global_map_ = *msg;
      map_received_ = true;
      map_sub_.shutdown(); 
      saveMapToDisk();
  }

  void callback(const grid_map_msgs::GridMapConstPtr& message) {
    if (!map_received_) return;

    grid_map::GridMap map;
    grid_map::GridMapRosConverter::fromMessage(*message, map);

    tf::StampedTransform transform;
    double rx, ry, rz, r_yaw;
    try {
        listener_.lookupTransform(map_frame_, robot_frame_, ros::Time(0), transform);
        rx = transform.getOrigin().x();
        ry = transform.getOrigin().y();
        rz = transform.getOrigin().z();
        r_yaw = tf::getYaw(transform.getRotation());
    } catch (tf::TransformException &ex) { return; }

    double ground_z = rz; 
    grid_map::Position robot_pos(rx, ry);
    if (map.isInside(robot_pos)) {
        float map_z = map.atPosition("elevation_smooth", robot_pos);
        if (std::isfinite(map_z)) ground_z = map_z;
    }

    octomap::OcTree tree(resolution_);
    ros::Time timestamp = ros::Time::now();

    for (grid_map::GridMapIterator it(map); !it.isPastEnd(); ++it) {
        if (!map.isValid(*it, "elevation_smooth")) continue;
        float cell_z = map.at("elevation_smooth", *it);
        grid_map::Position pos; 
        map.getPosition(*it, pos);

        if (std::abs(cell_z - ground_z) > 0) {
            for (double z = cell_z; z >= min_z_; z -= resolution_) 
                tree.updateNode(octomap::point3d(pos.x(), pos.y(), z), true);
        } else {
            tree.updateNode(octomap::point3d(pos.x(), pos.y(), cell_z), false);
        }
    }

    double g_res = global_map_.info.resolution;
    int robot_cell_x = std::floor((rx - global_map_.info.origin.position.x) / g_res);
    int robot_cell_y = std::floor((ry - global_map_.info.origin.position.y) / g_res);
    int local_width_cells = std::round(30.0 / g_res);
    
    nav_msgs::OccupancyGrid local_grid = generateAlignedCostmap(tree, robot_cell_x, robot_cell_y, ground_z, local_width_cells, timestamp);
    costmap_pub_.publish(local_grid);

    updateAndPublishGlobalMap(local_grid, robot_cell_x, robot_cell_y, timestamp, r_yaw);

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
                float diff = std::abs(max_z_obs - ground_z);
                if (diff <= 0.5) grid.data[idx] = 0;
                else grid.data[idx] = 100;
            } else if (has_data) {
                grid.data[idx] = 0; 
            } else {
                grid.data[idx] = -1; 
            }
        }
    }
    return grid;
  }

  void updateAndPublishGlobalMap(const nav_msgs::OccupancyGrid& local_grid, int rx_cell, int ry_cell, ros::Time ts, double r_yaw) {
      bool changed = false;
      global_map_.header.stamp = ts;
      double res = global_map_.info.resolution;

      int off_x = std::round((local_grid.info.origin.position.x - global_map_.info.origin.position.x) / res);
      int off_y = std::round((local_grid.info.origin.position.y - global_map_.info.origin.position.y) / res);

      // Robot orientation vectors
      double dir_x = cos(r_yaw);
      double dir_y = sin(r_yaw);

      // Lethal obstacle window (3,5m radius)
      int half_2m_cells = std::round(3.5 / res);
      int min_occ_x = rx_cell - half_2m_cells;
      int max_occ_x = rx_cell + half_2m_cells;
      int min_occ_y = ry_cell - half_2m_cells;
      int max_occ_y = ry_cell + half_2m_cells;

      for (int y = 0; y < (int)local_grid.info.height; ++y) {
          for (int x = 0; x < (int)local_grid.info.width; ++x) {
              int gx = off_x + x;
              int gy = off_y + y;

              if (gx >= 0 && gx < (int)global_map_.info.width && gy >= 0 && gy < (int)global_map_.info.height) {
                  int8_t val = local_grid.data[y * local_grid.info.width + x];
                  int8_t& global_val = global_map_.data[gy * global_map_.info.width + gx];
                  
                  // Dot product logic: is cell in the front 180-degree semi-circle?
                  double dx = (double)(gx - rx_cell);
                  double dy = (double)(gy - ry_cell);
                  bool is_forward = (dx * dir_x + dy * dir_y >= 0);

                  // 1. UNKNOWN & FREE: Only update if the cell is in front of the robot
                  if (is_forward) {
                      if (val == -1 && global_val != -1) {
                          global_val = -1;
                          changed = true;
                      }
                      else if (val == 0 && global_val != 0) {
                          global_val = 0;
                          changed = true;
                      }
                  }

                  // 2. OCCUPIED: Always add if within the 2m proximity window
                  // (Usually, we want obstacles added regardless of heading if they are close)
                  if (val == 100 && global_val != 100) {
                      if (gx >= min_occ_x && gx <= max_occ_x && gy >= min_occ_y && gy <= max_occ_y) {
                          global_val = 100;
                          changed = true;
                      }
                  }
              }
          }
      }
      merged_pub_.publish(global_map_);
      if (changed) saveMapToDisk();
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_, map_sub_;
  ros::Publisher octo_pub_, costmap_pub_, merged_pub_; 
  tf::TransformListener listener_;
  nav_msgs::OccupancyGrid global_map_;
  bool map_received_ = false;
  double resolution_, min_z_;
  std::string robot_frame_, map_frame_, map_save_path_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "elevation_to_map_merger");
  FastHeightNav node;
  ros::spin();
  return 0;
}