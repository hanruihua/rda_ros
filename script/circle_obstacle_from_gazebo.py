#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from costmap_2d.msg import ObstacleArray



def obstacle_callback():
    pass




if __name__ == '__main__':

    rospy.init_node('rda_node', anonymous=True)
    rospy.Subscriber('gazebo/model_states', ModelStates, obstacle_callback)
    rospy.Publisher('rda_obstacles', ObstacleArray, queue_size=10)


    while not rospy.is_shutdown():
        

          







