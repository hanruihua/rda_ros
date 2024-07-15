import numpy as np
import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped, Point
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from nav_msgs.msg import Odometry, Path 
from visualization_msgs.msg import Marker, MarkerArray
import tf
from collections import namedtuple


robot_tuple = namedtuple('robot_tuple', 'G h cone_type wheelbase max_speed max_acce')
rda_obs_tuple = namedtuple('rda_obs_tuple', 'center radius vertex cone_type velocity') # vertex: 2*number of vertex

class rda_core:
    def __init__(self) -> None:

        # publish topics
        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rda_path_pub = rospy.Publisher('rda_opt_path', Path, queue_size=10)
        self.ref_path_pub = rospy.Publisher('rda_ref_path', Path, queue_size=10)
        self.ref_traj_pub = rospy.Publisher('rda_ref_states', Path, queue_size=10)
        self.obs_pub = rospy.Publisher('rda_obs_markers', MarkerArray, queue_size=10)

        rospy.init_node('rda_node', anonymous=True)

        # ros parameters
        robot_info = rospy.get_param('robot_info', {'vertices': None, 'radius': None, 'max_speed': [10, 1], 'max_acce': [10, 0.5], 'length': None, 'width': None, 'wheelbase': None})
        receding = rospy.get_param('receding', 10)
        iter_num = rospy.get_param('iter_num', 3)
        enable_reverse = rospy.get_param('enable_reverse', False)
        sample_time = rospy.get_param('sample_time', 0.1)
        process_num = rospy.get_param('process_num', 4)
        iter_threshold = rospy.get_param('iter_threshold', 0.2)

        slack_gain = rospy.get_param('slack_gain', 8)
        max_sd = rospy.get_param('max_sd', 1.0)
        min_sd = rospy.get_param('min_sd', 0.1)
        ws = rospy.get_param('ws', 1.0)
        wu = rospy.get_param('wu', 0.5)
        ro1 = rospy.get_param('ro1', 200)
        ro2 = rospy.get_param('ro2', 1.0)

        obstacle_order=rospy.get_param('obstacle_order', True)
        self.max_obstacle_num = rospy.get_param('max_obs_num', 4)
        self.max_edge_num = rospy.get_param('max_edge_num', 5)

        self.goal_threshold = rospy.get_param('goal_threshold', 0.2)

        self.ref_speed = rospy.get_param('ref_speed', 4.0) # ref speed

        self.robot_state = None

        robot_info_tuple = self.generate_robot_tuple(robot_info)

        self.listener = tf.TransformListener()






    def generate_robot_tuple(self, robot_info):

        if robot_info is None:
            print('Lack of car information, please check the robot_info in config file')
            return

        if robot_info.get('vertices') is not None:
            G, h = self.generate_Gh(robot_info['vertices'])
            

        else:
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
        
        cone_type = robot_info['cone_type']
        max_speed = robot_info['max_speed']
        max_acce = robot_info['max_acce']

        robot_info_tuple = robot_tuple(G, h, cone_type, wheelbase, max_speed, max_acce)

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