#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.client import Client

class AdaptiveController:
    def __init__(self):
        rospy.init_node('adaptive_controller', anonymous=True)
        
        # TEB dynamic reconfigure client
        self.client = Client("/move_base/TebLocalPlannerROS", timeout=10)
        
        # Threshold to decide open vs tight space
        self.threshold = 1.5  # meters
        self.current_mode = None
        
        # Subscribe to LiDAR
        rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)
        
        rospy.loginfo("Adaptive Controller started!")
        rospy.spin()
    
    def lidar_callback(self, msg):
        # Get minimum distance to nearest obstacle
        ranges = [r for r in msg.ranges if r > 0.1 and r < 30.0]
        if not ranges:
            return
        min_range = min(ranges)
        
        if min_range > self.threshold:
            # OPEN SPACE - go fast
            if self.current_mode != "fast":
                self.current_mode = "fast"
                self.client.update_configuration({
                    "max_vel_x": 2.0,
                    "weight_obstacle": 30.0,
                    "inflation_dist": 0.4
                })
                rospy.loginfo("OPEN SPACE — switching to FAST mode (%.2fm)" % min_range)
        else:
            # TIGHT SPACE - go careful
            if self.current_mode != "careful":
                self.current_mode = "careful"
                self.client.update_configuration({
                    "max_vel_x": 0.5,
                    "weight_obstacle": 100.0,
                    "inflation_dist": 0.6
                })
                rospy.loginfo("TIGHT SPACE — switching to CAREFUL mode (%.2fm)" % min_range)

if __name__ == '__main__':
    try:
        AdaptiveController()
    except rospy.ROSInterruptException:
        pass
