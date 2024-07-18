import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Point
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from nav_msgs.msg import Odometry, Path 
from visualization_msgs.msg import Marker, MarkerArray
import tf
from collections import namedtuple
from RDA_planner.mpc import MPC
from math import atan2
from gctl.curve_generator import curve_generator
from sklearn.cluster import DBSCAN
from sensor_msgs.msg import LaserScan
import cv2
from ir_sim.util.util import get_transform

robot_tuple = namedtuple('robot_tuple', 'G h cone_type wheelbase max_speed max_acce dynamics')
rda_obs_tuple = namedtuple('rda_obs_tuple', 'center radius vertex cone_type velocity') # vertex: 2*number of vertex

class rda_core:
    def __init__(self) -> None:

        # publish topics
        self.vel_pub = rospy.Publisher('/rda_cmd_vel', Twist, queue_size=10)
        self.rda_path_pub = rospy.Publisher('/rda_opt_path', Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher('/rda_ref_path', Path, queue_size=10)
        self.ref_states_pub = rospy.Publisher('/rda_ref_states', Path, queue_size=10)
        self.obs_pub = rospy.Publisher('/rda_obs_markers', MarkerArray, queue_size=10)

        rospy.init_node('rda_node', anonymous=True)

        # ros parameters

        ## robot info
        robot_info = rospy.get_param('robot_info', {'vertices': None, 'radius': None, 'max_speed': [10, 1], 'max_acce': [10, 0.5], 'length': 2, 'width': 1, 'wheelbase': 1.5, 'dynamics': 'diff', 'cone_type': 'Rpositive'})

        ## For MPC
        receding = rospy.get_param('receding', 10)
        iter_num = rospy.get_param('iter_num', 3)
        enable_reverse = rospy.get_param('enable_reverse', False)
        sample_time = rospy.get_param('sample_time', 0.1)
        process_num = rospy.get_param('process_num', 4)
        iter_threshold = rospy.get_param('iter_threshold', 0.2)
        obstacle_order=rospy.get_param('obstacle_order', True)
        self.max_obstacle_num = rospy.get_param('max_obs_num', 4)
        self.max_edge_num = rospy.get_param('max_edge_num', 5)
        self.goal_threshold = rospy.get_param('goal_threshold', 0.2)
        self.ref_speed = rospy.get_param('ref_speed', 4.0) # ref speed

        ## Tune parameters
        slack_gain = rospy.get_param('slack_gain', 8)
        max_sd = rospy.get_param('max_sd', 1.0)
        min_sd = rospy.get_param('min_sd', 0.1)
        ws = rospy.get_param('ws', 1.0)
        wu = rospy.get_param('wu', 0.5)
        ro1 = rospy.get_param('ro1', 200)
        ro2 = rospy.get_param('ro2', 1.0)

        ## for scan
        use_scan_obstacle = rospy.get_param('use_scan_obstacle', False)
        self.scan_eps = rospy.get_param('scan_eps', 0.2)
        self.scan_min_samples = rospy.get_param('scan_min_samples', 6)
        lidar_offset_para = rospy.get_param('lidar_offset', '0.0 0.0 0.0 0.0 0.0 0.0') # x y z yaw pitch roll
        self.lidar_offset = np.fromstring(lidar_offset_para, dtype=np.float32, sep=' ')
        x, y, z, yaw, pitch, roll = self.lidar_offset
        self.l_trans, self.l_R = get_transform(np.c_[x, y, yaw].reshape(3, 1))
        # self.marker_color = rospy.get_param('marker_color', 'red')

        ## for reference path
        self.waypoints = rospy.get_param('waypoints', [])
        self.loop = rospy.get_param('loop', False)
        self.curve_type = rospy.get_param('curve_type', 'dubins')
        self.step_size = rospy.get_param('step_size', 0.1)
        self.min_radius = rospy.get_param('min_radius', 1.0)

        ## for frame
        self.target_frame = rospy.get_param('target_frame', 'map')

        # initialize
        self.robot_state = None
        self.obstacle_list = []
        self.cg = curve_generator()
        self.listener = tf.TransformListener()
        self.ref_path_list = self.generate_ref_path_list()
        robot_info_tuple = self.generate_robot_tuple(robot_info)

        self.rda_opt = MPC(robot_info_tuple, self.ref_path_list, receding, sample_time, iter_num, enable_reverse, False, process_num=process_num, iter_threshold=iter_threshold, slack_gain=slack_gain, max_sd=max_sd, min_sd=min_sd, ws=ws, wu=wu, ro1=ro1, ro2=ro2, obstacle_order=obstacle_order, max_edge_num=self.max_edge_num, max_obs_num=self.max_obstacle_num)

        self.arrive_flag = False

        # subscribe
        if not use_scan_obstacle:
            rospy.Subscriber('/rda_obstacles', ObstacleArrayMsg, self.obstacle_callback)
        else:
            rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/ref_path', Path, self.path_callback)
        rospy.Subscriber('move_base_simple_goal', PoseStamped, self.goal_callback)


    def control(self):

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            if self.robot_state is None:
                rospy.loginfo_throttle(1, 'waiting for robot states') 
                continue

            if not self.ref_path_list: 
                rospy.loginfo_throttle(1, 'waiting for reference path')
                continue
            else:
                ref_path = self.convert_to_path(self.ref_path_list)
                self.ref_path_pub.publish(ref_path)

            
            if len(self.obstacle_list) == 0:
                rospy.loginfo_throttle(1, 'No obstacles, perform path tracking algorithm')
            else:
                rda_obs_markers = self.convert_to_markers(self.obstacle_list)
                self.obs_pub.publish(rda_obs_markers) 


            if self.max_obstacle_num == 0:
                opt_vel, info = self.rda_opt.control(self.robot_state, self.ref_speed, [])
            else:
                opt_vel, info = self.rda_opt.control(self.robot_state, self.ref_speed, self.obstacle_list)

            if info['arrive']:
                if self.loop:
                    self.rda_opt.curve_index = 0
                    self.rda_opt.cur_index = 0
                else: 
                    opt_vel = np.zeros((2, 1))
                    print('arrive at the goal!')
            

            vel = self.convert_to_twist(opt_vel)
            rda_opt_path = self.convert_to_path(info['opt_state_list'])
            ref_states = self.convert_to_path(info['ref_traj_list'])

            self.ref_states_pub.publish(ref_states)
            self.vel_pub.publish(vel)
            self.rda_path_pub.publish(rda_opt_path)
            
            rate.sleep()


    def obstacle_callback(self, obstacle_array):

        temp_obs_list = []

        if self.max_obstacle_num == 0:
            return("No obstacles are allowed")

        for obstacles in obstacle_array.obstacles:

            vertex = obstacles.polygon.points
            vertex_num = len(vertex)

            if vertex_num == 1:
                # circle obstacle

                center = np.array([[vertex[0].x], [vertex[0].y]])
                radius = obstacles.radius
                
                linear, angular = obstacles.velocities.twist.linear.x, obstacles.velocities.twist.angular.z
                velocity = np.array([[linear], [angular]])

                circle_obs = rda_obs_tuple(center, radius, None, 'norm2', velocity)

                temp_obs_list.append(circle_obs)

            elif vertex_num == 2:
                # line obstacle 
                pass

            elif vertex_num > 2:
                # polygon obstacle
                vertex_list = [ np.array([[p.x], [p.y]]) for p in vertex]
                vertexes = np.hstack(vertex_list)

                linear, angular = obstacles.velocity.Twist.linear.x, obstacles.velocity.Twist.angular.z
                velocity = np.array([[linear], [angular]])

                polygon_obs = rda_obs_tuple(None, None, vertexes, 'Rpositive', velocity)

                temp_obs_list.append(polygon_obs)


        self.obstacle_list[:] = temp_obs_list[:]



    def odom_callback(self, data):

        # get robot current state
        x = data.pose.pose.position.x
        y = data.pose.pose.position.y
        z = data.pose.pose.position.z
        theta = rda_core.quat_to_yaw( data.pose.pose.orientation)

        current_frame = data.header.frame_id

        temp_state = np.array([[x], [y], [theta]])
        
        try:
            (trans,rot) = self.listener.lookupTransform(self.target_frame, current_frame, rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.loginfo_throttle(1, 'waiting for tf for the transform from {} to {}'.format(current_frame, self.target_frame)) 
            return 

        raw = rda_core.quat_to_yaw_list(rot)
        rot_matrix = np.array([[np.cos(raw), -np.sin(raw)], [np.sin(raw), np.cos(raw)]])
        trans_matrix = np.array([[trans[0]], [trans[1]]])

        new_state = np.zeros((3, 1))

        # breakpoint()
        new_state[0:2] = rot_matrix @ temp_state[0:2] + trans_matrix
        new_state[2, 0] = temp_state[2, 0] + raw

        self.robot_state = new_state


    def scan_callback(self, scan_data):

        ranges = np.array(scan_data.ranges)
        angles = np.linspace(scan_data.angle_min, scan_data.angle_max, len(ranges))
        
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
            point_array = np.hstack(point_list).T
            labels = DBSCAN(eps=self.scan_eps, min_samples=self.scan_min_samples).fit_predict(point_array)

            self.obstacle_list = []

            for label in np.unique(labels):
                if label == -1:
                    continue
                else:
                    point_array2 = point_array[labels == label]
                    rect = cv2.minAreaRect(point_array2.astype(np.float32))
                    box = cv2.boxPoints(rect)

                    vertices = box.T

                    temp_vertices = self.l_R @ vertices + self.l_trans
                    r_trans, r_R = get_transform(self.robot_state)
                    global_vertices = r_trans + r_R @ temp_vertices

                    self.obstacle_list.append(rda_obs_tuple(None, None, global_vertices, 'Rpositive', 0))

    def path_callback(self, data):
        pass


    def goal_callback(self, data):
        pass
    

    def generate_ref_path_list(self):

        if len(self.waypoints) == 0:
            return []
        
        else:
            point_list = [np.array([[p[0]], [p[1]], [p[2]]]).astype('float64') for p in self.waypoints]
            ref_path_list = self.cg.generate_curve(self.curve_type, point_list, self.step_size, self.min_radius)

            return  ref_path_list


    def convert_to_markers(self, obs_list):
    
        marker_array = MarkerArray()

        for obs_index, obs in enumerate(obs_list):
            marker = Marker()
            marker.header.frame_id = self.target_frame
            marker.type = marker.LINE_LIST
            marker.header.stamp = rospy.get_rostime()

            marker.scale.x = 0.05

            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0

            # breakpoint()

            if obs.vertex is not None:

                temp_matrix = np.hstack((obs.vertex, obs.vertex[:, 0:1])) 
                for i in range(temp_matrix.shape[1] - 1):
                    vp = temp_matrix[:, i]
                    vp1 = temp_matrix[:, i+1]

                    marker.points.append(Point(vp[0], vp[1], 0))
                    marker.points.append(Point(vp1[0], vp1[1], 0))

                marker.id = obs_index
                marker_array.markers.append(marker)

        return marker_array



    def convert_to_path(self, state_list):
        # from state list to path
        path = Path()

        path.header.seq = 0
        path.header.stamp = rospy.get_rostime()
        path.header.frame_id = self.target_frame

        for i in range(len(state_list)):
            ps = PoseStamped()

            ps.header.seq = i
            ps.header.stamp = rospy.get_rostime()
            ps.header.frame_id = self.target_frame

            ps.pose.position.x = state_list[i][0, 0]
            ps.pose.position.y = state_list[i][1, 0]
            ps.pose.orientation.w = 1

            path.poses.append(ps)

        return path

    def convert_to_twist(self, rda_vel):
        # from 2*1 vector to twist

        vel = Twist()
        vel.linear.x = rda_vel[0, 0]   # linear 
        vel.angular.z = rda_vel[1, 0]  # steering

        return vel




    @staticmethod
    def quat_to_yaw(quater):
         
        x = quater.x
        y = quater.y
        z = quater.z
        w = quater.w

        raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw
    
    @staticmethod
    def quat_to_yaw_list(quater):
         
        x = quater[0]
        y = quater[1]
        z = quater[2]
        w = quater[3]

        raw = atan2(2* ( w*z + x *y), 1 - 2 * (pow(z, 2) + pow(y, 2)))

        return raw



    def generate_robot_tuple(self, robot_info):

        if robot_info is None:
            print('Lack of car information, please check the robot_info in config file')
            return

        if robot_info.get('vertices', None) is None or robot_info.get('vertices', None) == 'None':
            length = robot_info['length']
            width = robot_info['width']
            wheelbase = robot_info['wheelbase']

            start_x = -(length - wheelbase)/2
            start_y = -width/2

            point0 = np.array([ [start_x], [start_y] ]) # left bottom point
            point1 = np.array([ [start_x+length], [start_y] ])
            point2 = np.array([ [start_x+length], [start_y+width]])
            point3 = np.array([ [start_x], [start_y+width]])

            vertex = np.hstack((point0, point1, point2, point3))
            
            G, h = self.generate_Gh(vertex)
        else:
            G, h = self.generate_Gh(robot_info['vertices'])


    
        cone_type = robot_info['cone_type']
        max_speed = robot_info['max_speed']
        max_acce = robot_info['max_acce']
        dynamics = robot_info['dynamics']

        robot_info_tuple = robot_tuple(G, h, cone_type, wheelbase, max_speed, max_acce, dynamics)

        return robot_info_tuple
    

    def generate_Gh(self, vertex):
        
        '''
        vertex: 2*num
        '''

        num = vertex.shape[1]

        G = np.zeros((num, 2)) 
        h = np.zeros((num, 1)) 
    
        for i in range(num):
            if i + 1 < num:
                pre_point = vertex[:, i]
                next_point = vertex[:, i+1]
            else:
                pre_point = vertex[:, i]
                next_point = vertex[:, 0]
            
            diff = next_point - pre_point
            
            a = diff[1]
            b = -diff[0]
            c = a * pre_point[0] + b * pre_point[1]

            G[i, 0] = a
            G[i, 1] = b
            h[i, 0] = c 
    
        return G, h
    
    