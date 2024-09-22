#!/usr/bin/env python

import rospy
import carla
import math
import time
from geometry_msgs.msg import PolygonStamped, Point32
from visualization_msgs.msg import Marker
from threading import Lock
from std_msgs.msg import Header

def carla_to_ros_point(carla_location):
    """
    Convert a Carla Location to a ROS Point32, adjusting coordinate systems.
    Carla uses left-handed coordinates; ROS uses right-handed.
    """
    return Point32(x=carla_location.x, y=-carla_location.y, z=carla_location.z)

def compute_lane_boundaries(waypoint, distance_ahead=30, sampling_resolution=1.0):
    """
    Compute left and right lane boundaries ahead of the given waypoint.
    """
    left_boundary = []
    right_boundary = []

    total_distance = 0.0
    current_wp = waypoint

    while total_distance < distance_ahead:
        lane_width = current_wp.lane_width
        transform = current_wp.transform

        # Yaw angle in radians
        yaw_rad = math.radians(transform.rotation.yaw)

        # Left and right offsets
        offset = lane_width / 2.0
        left_offset = carla.Location(
            x = offset * math.cos(yaw_rad + math.pi / 2),
            y = offset * math.sin(yaw_rad + math.pi / 2),
            z = 0.0
        )
        right_offset = carla.Location(
            x = offset * math.cos(yaw_rad - math.pi / 2),
            y = offset * math.sin(yaw_rad - math.pi / 2),
            z = 0.0
        )

        # Left and right boundary points
        left_point = transform.location + left_offset
        right_point = transform.location + right_offset

        # Convert to ROS coordinates
        left_point_ros = carla_to_ros_point(left_point)
        right_point_ros = carla_to_ros_point(right_point)

        left_boundary.append(left_point_ros)
        right_boundary.append(right_point_ros)

        # Move to the next waypoint
        next_waypoints = current_wp.next(sampling_resolution)
        if not next_waypoints:
            break
        current_wp = next_waypoints[0]
        total_distance += sampling_resolution

    return left_boundary, right_boundary

def main():
    rospy.init_node('lane_boundaries_publisher', anonymous=True)

    try:
        # Connect to Carla server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Get the world and map
        world = client.get_world()
        carla_map = world.get_map()

        # Get the vehicle named 'agent_0'
        vehicle = None
        actors = world.get_actors()
        for actor in actors:
            if actor.type_id.startswith('vehicle') and actor.attributes.get('role_name') == 'agent_0':
                vehicle = actor
                break

        if vehicle is None:
            rospy.logerr("Could not find vehicle with role_name 'agent_0'.")
            return

        rospy.loginfo("Vehicle 'agent_0' found.")

        # Publishers
        lane_boundaries_pub = rospy.Publisher('/lane_boundaries', PolygonStamped, queue_size=1)
        marker_pub = rospy.Publisher('/lane_boundaries_marker', Marker, queue_size=1)

        rate = rospy.Rate(10)  # 10 Hz

        while not rospy.is_shutdown():
            # Get vehicle transform
            vehicle_transform = vehicle.get_transform()
            vehicle_location = vehicle_transform.location

            # Get the current waypoint
            current_waypoint = carla_map.get_waypoint(vehicle_location, project_to_road=True, lane_type=carla.LaneType.Driving)

            # Compute lane boundaries
            left_boundary, right_boundary = compute_lane_boundaries(current_waypoint)

            if not left_boundary or not right_boundary:
                rospy.logwarn("No lane boundaries found.")
                rate.sleep()
                continue

            # Create a PolygonStamped message
            polygon_msg = PolygonStamped()
            polygon_msg.header = Header()
            polygon_msg.header.frame_id = 'map'
            polygon_msg.header.stamp = rospy.Time.now()

            # Add left boundary points
            polygon_msg.polygon.points.extend(left_boundary)

            # Add right boundary points in reverse order to close the polygon
            polygon_msg.polygon.points.extend(reversed(right_boundary))

            # Optionally, close the polygon by adding the first point again
            if polygon_msg.polygon.points:
                polygon_msg.polygon.points.append(polygon_msg.polygon.points[0])

            # Publish the polygon
            lane_boundaries_pub.publish(polygon_msg)

            # Create a Marker message for visualization in RViz
            marker = Marker()
            marker.header = polygon_msg.header
            marker.ns = 'lane_boundaries'
            marker.id = 0
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.1  # Line width
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0

            # Combine boundaries for visualization
            marker.points = []
            marker.points.extend(left_boundary)
            marker.points.extend(right_boundary)
            # Close the loop
            if marker.points:
                marker.points.append(marker.points[0])

            # Publish the marker
            marker_pub.publish(marker)

            rate.sleep()

    except rospy.ROSInterruptException:
        rospy.loginfo("Lane boundaries publisher interrupted.")

if __name__ == '__main__':
    main()