#! /usr/bin/env python
import rospy
import carla
import json
import numpy
from std_msgs.msg import Header
from geometry_msgs.msg import Polygon, Point32
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg


def lateral_shift(transform, shift):
    """Makes a lateral shift of the forward vector of a transform"""
    transform.rotation.yaw += 90
    return transform.location + shift * transform.get_forward_vector()


def add_topology():
    """Add polygons for topology"""
    map = world.get_map()
    precision = 0.1
    topology = map.get_topology()
    topology = [x[0] for x in topology]
    topology = sorted(topology, key=lambda w: w.transform.location.z)
    set_waypoints = []
    for waypoint in topology:
        waypoints = [waypoint]
        nxt = waypoint.next(precision)
        if len(nxt) > 0:
            nxt = nxt[0]
            while nxt.road_id == waypoint.road_id:
                waypoints.append(nxt)
                nxt = nxt.next(precision)
                if len(nxt) > 0:
                    nxt = nxt[0]
                else:
                    break
        set_waypoints.append(waypoints)

    for waypoints in set_waypoints:
        waypoint = waypoints[0]
        straight_road = 1
        boundary_road = 1
        points_list = []
        if not waypoint.is_junction:
            # form the polygon
            wp = waypoints[0]
            vertex1 = lateral_shift(wp.transform, wp.lane_width * 1.0)
            points_list.append(Point32(x=vertex1.x, y=-vertex1.y))
            vertex2 = lateral_shift(wp.transform, wp.lane_width * 2.0)
            points_list.append(Point32(x=vertex2.x, y=-vertex2.y))

            wp = waypoints[-1]
            vertex4 = lateral_shift(wp.transform, wp.lane_width * 2.0)
            points_list.append(Point32(x=vertex4.x, y=-vertex4.y))
            vertex3 = lateral_shift(wp.transform, wp.lane_width * 1.0)
            points_list.append(Point32(x=vertex3.x, y=-vertex3.y))

            # check straight road
            for wp in waypoints:
                way_yaw = numpy.mod(abs(wp.transform.rotation.yaw), 90)
                if way_yaw >= 2 and way_yaw <= 88:
                    straight_road = 0
                    break

            # check boundary road
            for wp in topology:
                dist = numpy.array(
                    [
                        wp.transform.location.x - vertex1.x,
                        wp.transform.location.y - vertex1.y,
                    ]
                )
                if numpy.linalg.norm(dist, ord=2) <= wp.lane_width * 0.5:
                    boundary_road = 0

            if straight_road == 1 and boundary_road == 1:
                obstacle = ObstacleMsg()
                obstacle.polygon = Polygon(points=points_list)
                rda_obstacles.obstacles.append(obstacle)


if __name__ == "__main__":
    rospy.init_node("generate_traffic_node")

    topology = rospy.get_param("~topology", False)

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
            obstacles = objects.get("obstacles", [])
            roads = objects.get("roads", [])

            # topology = objects["topology"]

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

        rda_obstacles = ObstacleArrayMsg()

        while not rospy.is_shutdown():
            world.wait_for_tick()

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

            if topology:
                add_topology()

            obs_pub.publish(rda_obstacles)

            rate.sleep()

    finally:
        rospy.loginfo("destroying {} actors".format(len(actors_list)))
        for actor in actors_list:
            if not actor.is_alive:
                continue
            actor.destroy()
