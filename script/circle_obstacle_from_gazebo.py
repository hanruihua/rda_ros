#! /usr/bin/env python
import rospy
from gazebo_msgs.msg import ModelStates
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg


model_radius = 0.05
model_name = "agent"
obstacles_msg = ObstacleArrayMsg()


def obs_state_callback(data):

    global model_name, model_radius, obstacles

    obstacles_msg.obstacles = []

    for index, name in enumerate(data.name):
        if model_name in name:

            obstacle = ObstacleMsg()
            obstacle.radius = model_radius
            obstacle.polygon.points.append(data.pose[index].position)
            obstacle.velocities.twist = data.twist[index]

            obstacles_msg.obstacles.append(obstacle)


if __name__ == "__main__":

    model_radius = rospy.get_param("model_radius", 0.05)
    model_name = rospy.get_param("model_name", "agent")

    rospy.init_node("gazebo_rda_convert_node", anonymous=True)
    rospy.Subscriber("gazebo/model_states", ModelStates, obs_state_callback)
    obs_pub = rospy.Publisher("rda_obstacles", ObstacleArrayMsg, queue_size=10)

    rate = rospy.Rate(50)

    while not rospy.is_shutdown():

        if len(obstacles_msg.obstacles) == 0:
            rospy.logwarn_throttle(1, "No obstacle detected")
            continue
        else:
            obs_pub.publish(obstacles_msg)
            rospy.loginfo_throttle(5, "Publish rda obstacles successfully")

        rate.sleep()
