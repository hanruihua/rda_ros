#!/usr/bin/python3

import carla
import time
import rospy
from rospy import ROSException

from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
from geometry_msgs.msg import Point


class CarlaMapVisualization:

    def __init__(self):
        client = carla.Client('localhost', 2000)
        world = client.get_world()
        time.sleep(1)
        self.map = world.get_map()
        self.weather = world.get_weather()
        self.weather.sun_altitude_angle = 10
        self.weather.fog_density = 0
        self.weather.fog_distance = 0
        world.set_weather(self.weather)
        # world.set_weather(carla.WeatherParameters.WetSunset)
        # while len(world.get_actors().filter("vehicle.tesla.model3")) == 0:
        #     pass
        # vehicle = world.get_actors().filter("vehicle.tesla.model3")[0]
        # spec = world.get_spectator()
        # location = vehicle.get_location() + carla.Location(x=-50.0, y=0.0, z=60.0)
        # transform = carla.Transform(location, carla.Rotation(pitch=-30))
        # spec.set_transform(transform)
        
        self.map_viz_publisher = rospy.Publisher('/carla/map_visualization', MarkerArray, latch=True, queue_size=1)

        self.id = 0
        self.marker_array = MarkerArray()

    def publish_msgs(self):
        """
        Function (override) to update this object.
        """
        self.draw_map()

        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.map_viz_publisher.publish(self.marker_array)
            r.sleep()

    @staticmethod
    def lateral_shift(transform, shift):
        """Makes a lateral shift of the forward vector of a transform"""
        transform.rotation.yaw += 90
        return transform.location + shift * transform.get_forward_vector()

    def set_marker_id(self):
        self.id += 1
        return self.id - 1

    def draw_map(self):
        precision = 0.1
        topology = self.map.get_topology()
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
            road_left_side = [self.lateral_shift(w.transform, -w.lane_width * 0.5) for w in waypoints]
            road_right_side = [self.lateral_shift(w.transform, w.lane_width * 0.5) for w in waypoints]
            # road_points = road_left_side + [x for x in reversed(road_right_side)]
            # self.add_line_strip_marker(points=road_points)

            if len(road_left_side) > 2:
                self.add_line_strip_marker(points=road_left_side)
            if len(road_right_side) > 2:
                self.add_line_strip_marker(points=road_right_side)

            if not waypoint.is_junction:
                for n, wp in enumerate(waypoints):
                    if ((n + 1) % 400) == 0:
                        self.add_arrow_line_marker(wp.transform)

    def add_arrow_line_marker(self, transform):
        arrow_marker = Marker()
        arrow_marker.type = Marker.LINE_LIST
        arrow_marker.header.frame_id = "map"
        arrow_marker.id = self.set_marker_id()
        arrow_marker.ns = "map_visulization"
        arrow_marker.color = ColorRGBA(0.8, 0.8, 0.8, 1)
        arrow_marker.scale.x = 0.2
        arrow_marker.pose.orientation.w = 1
        transform.rotation.yaw += 180
        forward = transform.get_forward_vector()
        transform.rotation.yaw += 90
        right_dir = transform.get_forward_vector()
        end = transform.location
        start = end - 2.0 * forward
        right = start + 0.8 * forward + 0.4 * right_dir
        left = start + 0.8 * forward - 0.4 * right_dir
        points = [start, end, start, left, start, right]
        for p in points:
            point = Point()
            point.x = p.x
            point.y = -p.y
            point.z = p.z
            # point.z = 0
            arrow_marker.points.append(point)
        self.marker_array.markers.append(arrow_marker)

    def add_line_strip_marker(self, color=None, points=None):
        marker = Marker()
        marker.id = self.set_marker_id()
        marker.type = Marker.LINE_STRIP
        marker.header.stamp = rospy.Time.now()
        marker.header.frame_id = "map"
        marker.ns = "map_visulization"

        if color is None:
            marker.color = ColorRGBA(0, 0, 0, 1)
        else:
            marker.color = color

        marker.scale.x = 0.25
        marker.pose.orientation.w = 1

        if points is not None:
            for p in points:
                point = Point()
                point.x = p.x
                point.y = -p.y
                point.z = p.z
                point.z = 0
                marker.points.append(point)
        self.marker_array.markers.append(marker)
        return marker

def main(args=None):

    rospy.init_node("carla_map_visualization", args)

    carla_map_visualization = None
    
    try:
        carla_map_visualization = CarlaMapVisualization()
        carla_map_visualization.publish_msgs()

        rospy.spin()
    except (RuntimeError, ROSException):
        pass
    except KeyboardInterrupt:
        print("User requested shut down.")
    finally:
        print("Shutting down.")


if __name__ == "__main__":
    main()
    