#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from dynamic_reconfigure.client import Client
from collections import deque

class AdaptiveController:
    def __init__(self):
        rospy.init_node('adaptive_controller', anonymous=True)

        self.open_threshold = 2.0
        self.tight_threshold = 0.8

        # SAFE speeds
        self.max_vel_fast = 1.5
        self.max_vel_medium = 1.2
        self.max_vel_careful = 0.6

        self.scan_data = None
        self.current_mode = None
        self.mode_buffer = deque(maxlen=7)

        rospy.sleep(3.0)

        self.client = Client("/move_base/TebLocalPlannerROS", timeout=10)
        self.update_teb_params("careful")
        self.current_mode = "careful"
        rospy.Subscriber('/front/scan', LaserScan, self.lidar_callback)

        self.run()

    def lidar_callback(self, msg):
        self.scan_data = msg

    def classify_environment(self, ranges):
        n = len(ranges)
        front = ranges[n//3:2*n//3]
        valid = [r for r in front if np.isfinite(r) and 0.1 < r < 30.0]
        if not valid:
            return "safe", 0
        min_range = min(valid)
        avg_range = np.mean(valid)
        if min_range > 2.0 and avg_range > 3.0:
            return "fast", min_range
        else:
            return "safe", min_range


    def update_teb_params(self, mode):
        try:
            if mode == "fast":
                self.client.update_configuration({
                    "max_vel_x": 1.0,
                    "weight_obstacle": 45.0,
                    "min_obstacle_dist": 0.25,
                    "weight_kinematics_forward_drive": 1.0,
                    "weight_optimaltime": 1.0 
                })
            elif mode == "safe":
                self.client.update_configuration({
                    "max_vel_x": 0.6,
                    "weight_obstacle": 20.0,
                    "min_obstacle_dist": 0.1,
                    "weight_kinematics_forward_drive": 10.0,
                    "weight_optimaltime": 5.0
                })
            elif mode == "careful":
                self.client.update_configuration({
                    "max_vel_x": 0.4,
                    "weight_obstacle": 10.0,
                    "min_obstacle_dist": 0.02,
                    "weight_kinematics_forward_drive": 100.0,
                    "weight_optimaltime": 15.0
                })
        except rospy.ServiceException as e:
            rospy.logwarn(f"Failed to update TEB params: {e}")
        except Exception as e:
            pass

    def run(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.scan_data is None:
                rate.sleep()
                continue

            mode, min_range = self.classify_environment(
                self.scan_data.ranges)

            self.mode_buffer.append(mode)

            if len(self.mode_buffer) < 3:
                rate.sleep()
                continue

            stable_mode = max(set(self.mode_buffer), key=self.mode_buffer.count)

            if stable_mode != self.current_mode:
                self.current_mode = stable_mode
                self.update_teb_params(stable_mode)


            rate.sleep()

if __name__ == '__main__':
    try:
        AdaptiveController()
    except rospy.ROSInterruptException:
        pass
