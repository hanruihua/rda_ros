#!/usr/bin/env python

import carla
import rospy
import math
from geometry_msgs.msg import Point, PoseStamped, Pose, Polygon
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import Quaternion, Point32
from tf.transformations import quaternion_from_euler
import tf

# Import the GlobalRoutePlanner from CARLA
from agents.navigation.global_route_planner import GlobalRoutePlanner

def main():
    rospy.init_node('waypoint_and_obstacle_publisher', anonymous=True)

    # Create ROS publishers
    waypoint_pub = rospy.Publisher('/carla/agent_0/waypoints', Path, queue_size=1, latch=True)
    obstacle_pub = rospy.Publisher('/rda_obstacles', ObstacleArrayMsg, queue_size=1)
    marker_pub = rospy.Publisher('/road_boundary_markers', MarkerArray, queue_size=1)

    try:
        # Connect to the CARLA server
        client = carla.Client('localhost', 2000)
        client.set_timeout(10.0)

        # Get the world and the map
        world = client.get_world()
        carla_map = world.get_map()

        # Find the vehicle actor
        vehicle_list = world.get_actors().filter('vehicle.*')
        if not vehicle_list:
            rospy.logerr("No vehicles found in the simulation.")
            return

        vehicle = vehicle_list[0]

        # Initial goal (you can change it or set via RViz)
        goal_location = carla.Location(x=202, y=-150.0, z=0)

        # Subscribe to the goal topic (e.g., from RViz)
        rospy.Subscriber('/move_base_simple/goal', PoseStamped, lambda msg: update_goal(msg, vehicle, waypoint_pub))

        # Generate global waypoints
        route = calculate_route(world, vehicle, goal_location)

        # Publish global waypoints
        publish_waypoints(route, waypoint_pub)

        rate = rospy.Rate(1)  # Hz

        while not rospy.is_shutdown():
            # Get the vehicle's current location
            vehicle_location = vehicle.get_location()

            # Generate road boundaries along the route
            left_boundary_points, right_boundary_points = compute_road_boundaries(route)

            # Publish obstacles as ObstacleArrayMsg
            publish_obstacles(left_boundary_points, right_boundary_points, obstacle_pub)

            # Optionally, publish markers for visualization
            publish_boundary_markers(left_boundary_points, right_boundary_points, marker_pub)

            rate.sleep()

    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr("An error occurred: {}".format(e))

def update_goal(goal_msg, vehicle, waypoint_pub):
    # Update the goal based on the received message
    goal_transform = pose_to_carla_transform(goal_msg.pose)
    goal_location = goal_transform.location
    rospy.loginfo("Received new goal: x={}, y={}, z={}".format(
        goal_location.x, goal_location.y, goal_location.z
    ))
    # Recalculate the route
    world = vehicle.get_world()
    route = calculate_route(world, vehicle, goal_location)

    # Publish the new waypoints
    publish_waypoints(route, waypoint_pub)

def calculate_route(world, vehicle, goal_location):
    # Calculate a route from the current location to goal_location
    rospy.loginfo("Calculating route to x={}, y={}, z={}".format(
        goal_location.x, goal_location.y, goal_location.z
    ))

    sampling_resolution = 1.0
    grp = GlobalRoutePlanner(world.get_map(), sampling_resolution)
    route = grp.trace_route(vehicle.get_location(), goal_location)

    return route

def publish_waypoints(route, waypoint_pub):
    msg = Path()
    msg.header.frame_id = "map"
    msg.header.stamp = rospy.Time.now()
    for wp, _ in route:
        pose = PoseStamped()
        pose.pose = carla_transform_to_pose(wp.transform)
        msg.poses.append(pose)

    waypoint_pub.publish(msg)
    rospy.loginfo("Published {} waypoints.".format(len(msg.poses)))

def compute_road_boundaries(route):
    left_boundary = []
    right_boundary = []

    for wp, _ in route:
        # Initialize cumulative offsets
        cumulative_offset_left = wp.lane_width / 2.0
        cumulative_offset_right = wp.lane_width / 2.0

        # Left side
        left_wp = wp.get_left_lane()
        visited_left_lane_ids = set()
        visited_left_lane_ids.add(wp.lane_id)
        while left_wp is not None and left_wp.lane_type == carla.LaneType.Driving:
            if left_wp.lane_id in visited_left_lane_ids:
                # Break to avoid infinite loop
                break
            visited_left_lane_ids.add(left_wp.lane_id)
            cumulative_offset_left += left_wp.lane_width
            left_wp = left_wp.get_left_lane()

        # Right side
        right_wp = wp.get_right_lane()
        visited_right_lane_ids = set()
        visited_right_lane_ids.add(wp.lane_id)
        while right_wp is not None and right_wp.lane_type == carla.LaneType.Driving:
            if right_wp.lane_id in visited_right_lane_ids:
                # Break to avoid infinite loop
                break
            visited_right_lane_ids.add(right_wp.lane_id)
            cumulative_offset_right += right_wp.lane_width
            right_wp = right_wp.get_right_lane()

        # Compute the boundary points
        wp_transform = wp.transform
        wp_location = wp_transform.location
        wp_rotation = wp_transform.rotation
        yaw = math.radians(wp_rotation.yaw)
        # Unit perpendicular vector to the left
        perpendicular_vector = carla.Vector3D(-math.sin(yaw), math.cos(yaw), 0)

        # Left boundary point
        left_boundary_point = wp_location - perpendicular_vector * cumulative_offset_left

        # Right boundary point
        right_boundary_point = wp_location + perpendicular_vector * cumulative_offset_right

        # Append to lists
        left_boundary.append(left_boundary_point)
        right_boundary.append(right_boundary_point)

    return left_boundary, right_boundary

def publish_obstacles(left_boundary_points, right_boundary_points, obstacle_pub):
    obstacle_array_msg = ObstacleArrayMsg()
    obstacle_array_msg.header.stamp = rospy.Time.now()
    obstacle_array_msg.header.frame_id = "map"

    # Create obstacles from boundary points
    # For each segment between consecutive points, create a thin rectangle obstacle
    # representing a line along the boundary
    # Left boundary obstacles
    for i in range(len(left_boundary_points) - 1):
        p_start = left_boundary_points[i]
        p_end = left_boundary_points[i + 1]

        # Calculate rectangle corners to create a thin rectangle along the left boundary line
        width = 0.05  # The thickness of the obstacle line

        # Direction vector from p_start to p_end
        dx = p_end.x - p_start.x
        dy = p_end.y - p_start.y
        length = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        # Calculate the corners of the rectangle
        offset_x = width / 2.0 * math.sin(angle)
        offset_y = width / 2.0 * -math.cos(angle)

        corner1 = Point32(x=p_start.x - offset_x, y=-(p_start.y - offset_y), z=p_start.z)
        corner2 = Point32(x=p_start.x + offset_x, y=-(p_start.y + offset_y), z=p_start.z)
        corner3 = Point32(x=p_end.x + offset_x, y=-(p_end.y + offset_y), z=p_end.z)
        corner4 = Point32(x=p_end.x - offset_x, y=-(p_end.y - offset_y), z=p_end.z)

        obstacle = ObstacleMsg()
        obstacle.header.stamp = rospy.Time.now()
        obstacle.header.frame_id = "map"

        polygon = Polygon()
        polygon.points = [corner1, corner2, corner3, corner4]
        obstacle.polygon = polygon
        obstacle.id = i
        obstacle.orientation = Quaternion()
        obstacle.orientation.w = 1.0
        obstacle.radius = 0.0  # Not used for polygon obstacles

        obstacle_array_msg.obstacles.append(obstacle)

    # Right boundary obstacles
    offset = len(left_boundary_points) - 1  # Offset the IDs for right obstacles
    for i in range(len(right_boundary_points) - 1):
        p_start = right_boundary_points[i]
        p_end = right_boundary_points[i + 1]

        # Calculate rectangle corners to create a thin rectangle along the right boundary line
        width = 0.05  # The thickness of the obstacle line

        # Direction vector from p_start to p_end
        dx = p_end.x - p_start.x
        dy = p_end.y - p_start.y
        length = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)

        # Calculate the corners of the rectangle
        offset_x = width / 2.0 * math.sin(angle)
        offset_y = width / 2.0 * -math.cos(angle)

        corner1 = Point32(x=p_start.x - offset_x, y=-(p_start.y - offset_y), z=p_start.z)
        corner2 = Point32(x=p_start.x + offset_x, y=-(p_start.y + offset_y), z=p_start.z)
        corner3 = Point32(x=p_end.x + offset_x, y=-(p_end.y + offset_y), z=p_end.z)
        corner4 = Point32(x=p_end.x - offset_x, y=-(p_end.y - offset_y), z=p_end.z)

        obstacle = ObstacleMsg()
        obstacle.header.stamp = rospy.Time.now()
        obstacle.header.frame_id = "map"

        polygon = Polygon()
        polygon.points = [corner1, corner2, corner3, corner4]
        obstacle.polygon = polygon
        obstacle.id = offset + i
        obstacle.orientation = Quaternion()
        obstacle.orientation.w = 1.0
        obstacle.radius = 0.0  # Not used for polygon obstacles

        obstacle_array_msg.obstacles.append(obstacle)

    obstacle_pub.publish(obstacle_array_msg)
    rospy.loginfo("Published {} obstacles.".format(len(obstacle_array_msg.obstacles)))

def publish_boundary_markers(left_boundary_points, right_boundary_points, marker_pub):
    markers = []

    # Left boundary markers
    left_marker = Marker()
    left_marker.header.frame_id = "map"
    left_marker.header.stamp = rospy.Time.now()
    left_marker.ns = "left_boundary"
    left_marker.id = 0
    left_marker.type = Marker.LINE_STRIP
    left_marker.action = Marker.ADD
    left_marker.scale.x = 0.2  # Line width
    left_marker.color.r = 1.0
    left_marker.color.g = 0.0
    left_marker.color.b = 0.0
    left_marker.color.a = 1.0  # Fully opaque

    for point in left_boundary_points:
        p = Point()
        p.x = point.x
        p.y = -point.y  # Invert Y to match ROS coordinate system
        p.z = point.z
        left_marker.points.append(p)

    markers.append(left_marker)

    # Right boundary markers
    right_marker = Marker()
    right_marker.header.frame_id = "map"
    right_marker.header.stamp = rospy.Time.now()
    right_marker.ns = "right_boundary"
    right_marker.id = 1
    right_marker.type = Marker.LINE_STRIP
    right_marker.action = Marker.ADD
    right_marker.scale.x = 0.2  # Line width
    right_marker.color.r = 0.0
    right_marker.color.g = 0.0
    right_marker.color.b = 1.0
    right_marker.color.a = 1.0  # Fully opaque

    for point in right_boundary_points:
        p = Point()
        p.x = point.x
        p.y = -point.y  # Invert Y to match ROS coordinate system
        p.z = point.z
        right_marker.points.append(p)

    markers.append(right_marker)

    marker_array = MarkerArray()
    marker_array.markers.extend(markers)
    marker_pub.publish(marker_array)

def carla_transform_to_pose(transform):
    pose = Pose()
    pose.position.x = transform.location.x
    pose.position.y = -transform.location.y  # Invert Y-axis for ROS
    pose.position.z = transform.location.z

    yaw = -math.radians(transform.rotation.yaw)  # Invert yaw for ROS
    pitch = -math.radians(transform.rotation.pitch)
    roll = -math.radians(transform.rotation.roll)

    quaternion = quaternion_from_euler(roll, pitch, yaw)
    pose.orientation.x = quaternion[0]
    pose.orientation.y = quaternion[1]
    pose.orientation.z = quaternion[2]
    pose.orientation.w = quaternion[3]

    return pose

def pose_to_carla_transform(pose):
    # Convert ROS Pose to CARLA Transform
    transform = carla.Transform()
    transform.location.x = pose.position.x
    transform.location.y = -pose.position.y  # Invert Y-axis
    transform.location.z = pose.position.z

    roll, pitch, yaw = euler_from_quaternion([
        pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    ])
    transform.rotation.roll = -math.degrees(roll)
    transform.rotation.pitch = -math.degrees(pitch)
    transform.rotation.yaw = -math.degrees(yaw)

    return transform

def euler_from_quaternion(quat):
    # Helper function to convert quaternion to Euler angles
    x, y, z, w = quat
    roll, pitch, yaw = tf.transformations.euler_from_quaternion([x, y, z, w])
    return roll, pitch, yaw

if __name__ == '__main__':
    main()