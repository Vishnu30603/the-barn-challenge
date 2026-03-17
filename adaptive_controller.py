#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
import subprocess

class AdaptiveController:
    def __init__(self):
        rospy.init_node('adaptive_controller', anonymous=True)
        
        self.threshold = 1.5
        self.current_mode = None
        
        rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)
        rospy.loginfo("Adaptive Controller started!")
        rospy.spin()
    
    def lidar_callback(self, msg):
        ranges = [r for r in msg.ranges if r > 0.1 and r < 30.0]
        if not ranges:
            return
        min_range = min(ranges)
        
        if min_range > self.threshold:
            if self.current_mode != "fast":
                self.current_mode = "fast"
                subprocess.Popen([
                    'rosrun', 'dynamic_reconfigure', 'dynparam', 'set',
                    '/move_base/TebLocalPlannerROS',
                    'max_vel_x', '2.0'
                ])
                rospy.loginfo("OPEN SPACE — FAST mode (%.2fm)" % min_range)
        else:
            if self.current_mode != "careful":
                self.current_mode = "careful"
                subprocess.Popen([
                    'rosrun', 'dynamic_reconfigure', 'dynparam', 'set',
                    '/move_base/TebLocalPlannerROS',
                    'max_vel_x', '0.5'
                ])
                rospy.loginfo("TIGHT SPACE — CAREFUL mode (%.2fm)" % min_range)

if __name__ == '__main__':
    try:
        AdaptiveController()
    except rospy.ROSInterruptException:
        pass
