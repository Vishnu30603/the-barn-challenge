#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AdaptiveController:
    def __init__(self):
        rospy.init_node('adaptive_controller', anonymous=True)
        
        # Thresholds
        self.open_threshold = 2.0    # meters - open space
        self.tight_threshold = 1.0   # meters - tight space
        
        # State
        self.current_mode = None
        self.scan_data = None
        
        # Publisher - velocity scaling
        self.vel_pub = rospy.Publisher('/adaptive/status', Twist, queue_size=1)
        
        # Subscriber
        rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)
        
        # Parameters
        self.max_vel_fast = 2.0
        self.max_vel_careful = 0.5
        self.max_vel_medium = 1.2
        
        rospy.loginfo("Adaptive Controller initialized!")
        self.run()
    
    def lidar_callback(self, msg):
        self.scan_data = msg
    
    def classify_environment(self, ranges):
        # Filter valid readings
        valid = [r for r in ranges if 0.1 < r < 30.0]
        if not valid:
            return "unknown"
        
        min_range = min(valid)
        mean_range = np.mean(valid)
        
        # Count how many readings are close
        close_count = sum(1 for r in valid if r < 1.5)
        close_ratio = close_count / len(valid)
        
        # Classification logic
        if min_range > self.open_threshold and close_ratio < 0.1:
            return "open"
        elif min_range < self.tight_threshold or close_ratio > 0.3:
            return "tight"
        else:
            return "medium"
    
    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        
        while not rospy.is_shutdown():
            if self.scan_data is None:
                rate.sleep()
                continue
            
            env_type = self.classify_environment(self.scan_data.ranges)
            
            if env_type != self.current_mode:
                self.current_mode = env_type
                
                if env_type == "open":
                    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 
                                   self.max_vel_fast)
                    rospy.set_param('/move_base/TebLocalPlannerROS/weight_obstacle', 
                                   30.0)
                    rospy.loginfo("OPEN SPACE - max speed: %.1f m/s" % self.max_vel_fast)
                    
                elif env_type == "tight":
                    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 
                                   self.max_vel_careful)
                    rospy.set_param('/move_base/TebLocalPlannerROS/weight_obstacle', 
                                   100.0)
                    rospy.loginfo("TIGHT SPACE - max speed: %.1f m/s" % self.max_vel_careful)
                    
                else:  # medium
                    rospy.set_param('/move_base/TebLocalPlannerROS/max_vel_x', 
                                   self.max_vel_medium)
                    rospy.set_param('/move_base/TebLocalPlannerROS/weight_obstacle', 
                                   60.0)
                    rospy.loginfo("MEDIUM SPACE - max speed: %.1f m/s" % self.max_vel_medium)
            
            rate.sleep()

if __name__ == '__main__':
    try:
        AdaptiveController()
    except rospy.ROSInterruptException:
        pass
