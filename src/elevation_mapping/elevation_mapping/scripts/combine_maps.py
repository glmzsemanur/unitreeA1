#!/usr/bin/env python3
import rospy
import rosbag
from grid_map_msgs.msg import GridMap

def load_separate_maps():
    rospy.init_node('map_loader_separate')

    # Define your paths
    map_paths = {
        '/home/semanur/maps/trial_1': '/map_1',
        '/home/semanur/maps/trial_2': '/map_2'
    }

    # Create publishers for each topic
    publishers = {
        topic: rospy.Publisher(topic, GridMap, latch=True, queue_size=1)
        for topic in map_paths.values()
    }

    for file_path, topic in map_paths.items():
        rospy.loginfo(f"Processing bag: {file_path}")
        try:
            bag = rosbag.Bag(file_path)
            found_msg = False

            for _, msg, _ in bag.read_messages():
                # We look for the first GridMap message in the bag
                if isinstance(msg, GridMap):
                    rospy.loginfo(f"Found GridMap! Publishing to {topic}...")
                    
                    # Ensure the frame_id is set so RViz can display it
                    if not msg.info.header.frame_id:
                        msg.info.header.frame_id = "map"
                    
                    publishers[topic].publish(msg)
                    found_msg = True
                    break # Stop after finding the first map in this bag

            bag.close()
            if not found_msg:
                rospy.logwarn(f"No GridMap message found in {file_path}")

        except Exception as e:
            rospy.logerr(f"Failed to load {file_path}: {e}")

    rospy.loginfo("Maps loaded and latched. Keep this node running to view in RViz.")
    rospy.spin()

if __name__ == "__main__":
    try:
        load_separate_maps()
    except rospy.ROSInterruptException:
        pass