#! /usr/bin/env python
import rospy
import carla
import json
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg

if __name__ == "__main__":
    rospy.init_node("generate_traffic_node")

    client = carla.Client("localhost", 2000)
    client.set_timeout(10.0)

    actors_list = []

    try:
        world = client.get_world()

        blueprint_library = world.get_blueprint_library()
        blueprint = blueprint_library.find("vehicle.audi.tt")

        actors_list = []

        with open(rospy.get_param("~objects_file"), "r") as file:
            objects = json.load(file)
            obstacles = objects["obstacles"]
            roads = objects["roads"]

        for obs in obstacles:
            location = carla.Location(
                obs["location"]["x"], obs["location"]["y"], obs["location"]["z"]
            )
            rotation = carla.Rotation(0, obs["rotation"]["yaw"], 0)
            transform = carla.Transform(location, rotation)
            actor = world.spawn_actor(blueprint, transform)

            if obs["autopilot"]:
                actor.set_autopilot(True)

            actors_list.append(actor)

        obs_pub = rospy.Publisher("rda_obstacles", ObstacleArrayMsg, queue_size=10)

        rate = rospy.Rate(50)

        while not rospy.is_shutdown():
            world.wait_for_tick()

            rda_obstacles = ObstacleArrayMsg()
            rda_obstacles.header = Header(stamp=rospy.Time.now(), frame_id="map")
            rda_obstacles.obstacles = []

            for actor in actors_list:
                if not actor.is_alive:
                    continue
                obstacle = ObstacleMsg()
                transform = actor.get_transform()
                vertices = actor.bounding_box.get_local_vertices()
                vertices = [
                    transform.transform(vertices[index]) for index in [0, 2, 6, 4]
                ]

                obstacle.polygon = Polygon(
                    points=[Point32(x=v.x, y=-v.y) for v in vertices]
                )

                obstacle.velocities.twist.linear.x = actor.get_velocity().x
                obstacle.velocities.twist.linear.y = -actor.get_velocity().y
                obstacle.velocities.twist.linear.z = actor.get_velocity().z

                rda_obstacles.obstacles.append(obstacle)

            for road in roads:
                obstacle = ObstacleMsg()
                obstacle.polygon = Polygon(
                    points=[Point32(x=v["x"], y=-v["y"]) for v in road]
                )
                rda_obstacles.obstacles.append(obstacle)


            obs_pub.publish(rda_obstacles)

            rate.sleep()

    finally:
        rospy.loginfo("destroying {} actors".format(len(actors_list)))
        for actor in actors_list:
            if not actor.is_alive:
                continue
            actor.destroy()
