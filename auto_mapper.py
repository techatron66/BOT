#!/usr/bin/env python
"""
Autonomous Mapping Script
Monitors exploration progress and saves map when complete.
Can be run alongside exploration.launch.
"""

import rospy
import roslaunch
import os
import subprocess
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

class AutoMapper:
    def __init__(self):
        rospy.init_node('auto_mapper')

        # Parameters
        self.map_topic = rospy.get_param('~map_topic', '/map')
        self.save_path = rospy.get_param('~save_path', 
                                         os.path.expanduser('~/catkin_ws/src/articubot_one/maps/auto_map'))
        self.check_interval = rospy.get_param('~check_interval', 10.0)  # seconds
        self.stable_threshold = rospy.get_param('~stable_threshold', 5)  # consecutive checks

        # State
        self.map_sub = rospy.Subscriber(self.map_topic, OccupancyGrid, self.map_callback)
        self.prev_map_size = 0
        self.stable_count = 0
        self.map_saved = False

        rospy.loginfo("AutoMapper started. Monitoring map growth...")
        rospy.loginfo("Map will be saved to: %s", self.save_path)

    def map_callback(self, msg):
        # Count known cells (not -1 = unknown)
        known_cells = sum(1 for cell in msg.data if cell != -1)

        if known_cells == self.prev_map_size:
            self.stable_count += 1
        else:
            self.stable_count = 0
            self.prev_map_size = known_cells
            rospy.loginfo("Map growing... Known cells: %d", known_cells)

        # If map hasn't grown for N checks, it's probably complete
        if self.stable_count >= self.stable_threshold and not self.map_saved:
            rospy.logwarn("Map appears complete! Saving...")
            self.save_map()

    def save_map(self):
        try:
            # Ensure directory exists
            map_dir = os.path.dirname(self.save_path)
            if not os.path.exists(map_dir):
                os.makedirs(map_dir)

            # Save map using map_server
            cmd = ['rosrun', 'map_server', 'map_saver', 
                   '-f', self.save_path]
            subprocess.call(cmd)

            self.map_saved = True
            rospy.loginfo("Map saved successfully to: %s", self.save_path)
            rospy.loginfo("You can now use this map for navigation:")
            rospy.loginfo("roslaunch articubot_one navigation.launch map:=%s.yaml", self.save_path)

        except Exception as e:
            rospy.logerr("Failed to save map: %s", str(e))

    def run(self):
        rate = rospy.Rate(1.0 / self.check_interval)
        while not rospy.is_shutdown() and not self.map_saved:
            rate.sleep()

        if self.map_saved:
            rospy.loginfo("Auto-mapping complete. Shutting down...")
            rospy.sleep(2.0)

if __name__ == '__main__':
    try:
        mapper = AutoMapper()
        mapper.run()
    except rospy.ROSInterruptException:
        pass
