#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from costmap_2d.msg import ObstacleArray
from sensor_msgs.msg import LaserScan
import numpy as np
from sklearn.cluster import DBSCAN

# def scan_callback(scan_data):

#     ranges = np.array(scan_data.ranges)
#     angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))




def scan_callback(self, scan_data):
        # process laser scan data
        ranges = np.array(scan_data.ranges)
        angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))

        # ranges = np.array(scan_data['ranges'])
        # angles = np.linspace(scan_data['angle_min'], scan_data['angle_max'], len(ranges))
        point_list = []
        
        for i in range(len(ranges)):
            scan_range = ranges[i]
            angle = angles[i]

            if scan_range < ( scan_data.range_max - 0.01):
                point = np.array([ [scan_range * np.cos(angle)], [scan_range * np.sin(angle)]  ])
                point_list.append(point)

        if len(point_list) < 3 or self.robot_state is None:
            rospy.loginfo_throttle(1, 'No obstacles are converted to polygon') 
            return

        else:
            if (self.static_obstacle and not self.calibrate) or not self.static_obstacle:
                
                self.obs_list = []

                point_array = np.hstack(point_list).T
                labels = DBSCAN(eps=self.eps, min_samples=self.min_samples).fit_predict(point_array)

                for label in labels:
                    if label == -1:
                        continue
                    else:
                        point_array2 = point_array[labels == label]
                        rect = cv2.minAreaRect(point_array2.astype(np.float32))
                        box = cv2.boxPoints(rect)

                        vertices = box.T

                        trans = self.robot_state[0:2]
                        rot = self.robot_state[2, 0]
                        R = np.array([[np.cos(rot), -np.sin(rot)], [np.sin(rot), np.cos(rot)]])
                        global_vertices = trans + R @ vertices

                        self.obs_list.append(rda_obs_tuple(None, None, global_vertices, 'Rpositive', 0))


# if __name__ == '__main__':

#     rospy.init_node('rda_node', anonymous=True)
    
#     rospy.Subscriber('/scan', LaserScan, scan_callback)
#     rospy.Subscriber('odom', Odometry, odom_callback)
#     rospy.Publisher('/rda_obstacles', ObstacleArray, queue_size=10)

#     while not rospy.is_shutdown():
#         pass


          







