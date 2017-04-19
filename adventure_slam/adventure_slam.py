#!/usr/bin/env python
import rospy, pcl_ros, tf
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Point, Quaternion
from visualization_msgs.msg import MarkerArray, Marker
from nav_msgs.msg import OccupancyGrid, Odometry

import cv2, math, pcl
import numpy as np
from math import sqrt

pub = rospy.Publisher('/slam_debug', MarkerArray)
pub_odom = rospy.Publisher('/odom', Odometry)
br = tf.TransformBroadcaster()
pre_lines = []
pose = (0,0,0)

def get_line(p1, v1, id_, color=(0,0,1)):
    marker = Marker()
    marker.header.frame_id = "camera_depth_frame"
    marker.header.stamp = rospy.Time()
    marker.lifetime = rospy.Duration(1)
    marker.ns = "slam_debug_ns"
    marker.id = id_
    marker.type = Marker.LINE_STRIP
    marker.action = Marker.ADD
    marker.pose.position.x = 0
    marker.pose.position.y = 0
    marker.pose.position.z = 0
    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 0.0
    marker.scale.x = 0.01
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.a = 1.0
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]
    marker.points.append(Point(p1[0] + 100 * v1[0], p1[1] + 100 * v1[1], 0))
    marker.points.append(Point(p1[0] - 100 * v1[0], p1[1] - 100 * v1[1], 0))
    return marker


def laser_callback(scan):
    marker_array = MarkerArray()

    # Convert the laserscan to coordinates
    angle = scan.angle_min
    points = []
    for r in scan.ranges:
        theta = angle
        angle += scan.angle_increment
        if (r > scan.range_max) or (r < scan.range_min):
            continue
        if (math.isnan(r)):
            continue

        points.append([r * math.sin(theta), r * math.cos(theta)])

    # Fit the line
    ## convert points to pcl type
    points = np.array(points, dtype=np.float32)
    pcl_points = np.concatenate((points, np.zeros((len(points), 1))), axis=1)

    cur_lines = []
    min_inliers = int(rospy.get_param('min inliers'))
    id = 0
    while len(pcl_points) >= min_inliers:
        p = pcl.PointCloud(np.array(pcl_points, dtype=np.float32))
        ## create a segmenter object
        seg = p.make_segmenter()
        seg.set_model_type(pcl.SACMODEL_LINE)
        seg.set_method_type(pcl.SAC_RANSAC)
        seg.set_distance_threshold (0.003)

        ## apply RANSAC
        indices, model = seg.segment()
        # print "Found", len(indices), "inliers", model

        if len(indices) < min_inliers:
            break

        next_points = []
        indices = set(indices)
        for index, point in enumerate(pcl_points):
            if index not in indices:
                next_points.append(point)

        pcl_points = np.array(next_points, dtype=np.float32)

        # OpenCV line fitter - least squares
        # line = cv2.fitLine(points, 2, 0, 0.01, 0.01)
        # Publish detected lines so we can see them in Rviz
        # marker_array.markers.append(get_line((line[3], line[2]), (line[1], line[0]), 1, (0,1,0)))
        # pub.publish(marker_array)

        id += 1
        marker_array.markers.append(get_line((model[1], model[0]), (model[4], model[3]), id))
        v1 = (0,0,1)
        v2 = (model[4], model[3], 0)
        unit_vector = np.cross(v1,v2) / (sqrt(np.dot(v1,v1))*sqrt(np.dot(v2,v2)))
        v3 = (model[1], model[0], 0)
        vector_magnitude = np.dot(v3, unit_vector)
        vector = unit_vector * vector_magnitude
        # print "vector:", vector
        # print "vector length:",sqrt(np.dot(vector,vector))
        cur_lines.append((vector[0], vector[1]))

    global br
    global pre_lines
    match_pairs = []
    # max_angle = rospy.get_param("max delta angle")
    max_delta_angle = 0.3
    # max_len_diff = rospy.get_param("max len diff")
    max_len_diff = 0.3
    for v1 in pre_lines:
        for v2 in cur_lines:
            v1_len = sqrt(np.dot(v1,v1))
            v2_len = sqrt(np.dot(v2,v2))
            delta_len = v1_len - v2_len
            if abs(delta_len) < max_len_diff:
                pre_angle = math.atan2(v1[1], v1[0])
                cur_angle = math.atan2(v2[1], v2[0])
                delta_angle = pre_angle - cur_angle
                if abs(delta_angle) < max_delta_angle:
                    # print "current angle:", cur_angle
                    # print "pre_angle:", pre_angle
                    avg_angle = (pre_angle + cur_angle) / 2
                    match_pairs.append((avg_angle, delta_angle, delta_len))

    pre_lines = cur_lines
    global pose
    pre_x, pre_y, pre_orientation = pose[0], pose[1], pose[2]
    delta_angle_sum = 0
    delta_translation_sum = 0
    if len(match_pairs) > 0:
        print "Found",len(match_pairs),"match"
        actual_used_pairs_for_length = 0
        for i,pair in enumerate(match_pairs):
            delta_angle_sum += pair[1]
            # print "Delta angle",i,":",pair[1]
            # print "Delta vector length:",pair[2]
            delta_len = pair[2]
            print "avg angle between vector and robot:",pair[0]
            if abs(abs(pair[0]) - (math.pi/2)) > 0.2:
                delta_translation = delta_len / math.cos(pair[0])
                print "Used Delta translation:", delta_translation
                delta_translation_sum += delta_translation
                actual_used_pairs_for_length += 1

        avg_delta_angle = delta_angle_sum / len(match_pairs)
        avg_delta_translation = delta_translation_sum / actual_used_pairs_for_length

        # print "avg delta angle:", avg_delta_angle
        # print "avg delta translation:", avg_delta_translation
        cur_x = pre_x + avg_delta_translation * math.cos(pre_orientation)
        cur_y = pre_y + avg_delta_translation * math.sin(pre_orientation)
        cur_orientation = pre_orientation + avg_delta_angle
        pose = (cur_x, cur_y, cur_orientation)
    else:
        print "Failed to match"

    print "Pose in odom:", pose
    t = tf.transformations
    # Broadcast tf
    br.sendTransform((-pose[0], -pose[1], 0),
                     t.quaternion_from_euler(0, 0, -pose[2]),
                     rospy.Time.now(),
                     'odom_visual',
                     'base_footprint')
    # Publish Odometry
    odom = Odometry()
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = 'base_footprint'
    odom.child_frame_id = 'odom_visual'
    odom.pose.pose.position = Point(-pose[0],-pose[1],0)
    q = t.quaternion_from_euler(0, 0, -pose[2])
    odom.pose.pose.orientation = Quaternion(q[0],q[1],q[2],q[3])
    pub_odom.publish(odom)

    pub.publish(marker_array)


def main():
    rospy.init_node('adventure_slam', anonymous=True)
    rospy.Subscriber("/scan", LaserScan, laser_callback)
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
