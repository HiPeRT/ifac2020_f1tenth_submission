#!/usr/bin/env python2.7

import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
import time
import thread
import sys
import geometry_msgs
import tf2_ros
import tf_conversions
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Path, Odometry
from tf.transformations import euler_from_quaternion
import math

input_map_ = OccupancyGrid()

new_obstacles_ = []

car_data_received = False
pose = Pose()

def odom_callback(data):

    global car_data_received
    global pose

    pose = data.pose.pose

    car_data_received = True

def spin():
    print "spin start"
    rospy.spin()

class translation:
    x = 0
    y = 0
    z = 0

class rotation:
    x = 0
    y = 0
    z = 0
    w = 0
    
trs = translation()
rtn = rotation()

            
def scan_callback(data):

    angle = data.angle_min
    center = len(data.ranges)/2
    frontal_distance = data.ranges[center]
    left_distance = 0
    right_distance = 0
    n_right = 0
    n_left = 0
    field_of_view = 1.7
    max_range = 10
    min_range = 0.5
    clear_obstacles_count_ = 0
    
    theta = data.angle_min - data.angle_increment
    
    for actual_range in data.ranges:
        
        if(actual_range is None or angle < data.angle_min or angle > data.angle_max \
        or abs(angle) > field_of_view or actual_range > max_range or actual_range < min_range \
        or actual_range < data.range_min or actual_range > data.range_max):
            continue
            
        if angle > 0: #left
            left_distance += actual_range
            n_left +=1
        elif angle < 0: #right
            right_distance += actual_range
            n_right +=1
            
        #laser hit x, y in base_link frame	
        x_base_link = actual_range * cos(theta)
        y_base_link = actual_range * sin(theta)
        
        #get yaw
        o = pose.pose.orientation
        orientation = euler_from_quaternion([o.x, o.y, o.z, o.w])

        yaw = orientation[-1]
        
        # laser hit x, y in base_link frame
        x_map = x_base_link*cos(yaw) - y_base_link*sin(yaw) + trs.x
        y_map = x_base_link*sin(yaw) + y_base_link*cos(yaw) + trs.y
        
        index_of_expanded_obstacles = get_expanded_row_major_indices(x_map, y_map)
        
        for index in index_of_expanded_obstacles:
            if input_map_.data[index] != 100 :
                input_map_.data[index] = 100
                new_obstacles_.append(index)
        
        angle+=data.angle_increment
        
    clear_obstacles_count_ += 1
    
    if clear_obstacles_count_ > 10:
        for index in new_obstacles_:
            input_map_.data[index] = 0
        
        new_obstacles_ = []
        clear_obstacles_count_ = 0
        
    map_update_pub.publish(input_map_)
    
def map_callback(data):

    input_map_ = data
    
def get_expanded_row_major_indices(x_map, y_map):
    expanded_row_major_indices = []
    x_index = x_map - input_map_.info.origin.position.x / input_map_.info.resolution
    y_index = y_map - input_map_.info.origin.position.y / input_map_.info.resolution
    
    i = -5+x_index
    while i<6+x_index :
        j = -5+y_index
        while j < 6+y_index :
            expanded_row_major_indices.append(j*input_map_.info.width + i)
            j+=1
        i+=1
    
    return y_index*input_map_.info.width + x_index

if __name__ == "__main__":
    rospy.init_node('save_tf')
    thread.start_new_thread(spin, ())
    listener = tf.TransformListener()
    started = False

    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("scan", LaserScan, scan_callback)
    rospy.Subscriber("map", OccupancyGrid, map_callback)
    pub_path = rospy.Publisher('path', Path, queue_size=1)
    map_update_pub = rospy.Publisher('map_updated', OccupancyGrid, queue_size=1)

    print "START"
    r = rospy.Rate(40)
    while not rospy.is_shutdown():

        try:
            print "working"
            
            #input_map_ = rospy.wait_for_message('map', OccupancyGrid)
            
            (trans,rot) = listener.lookupTransform('ego_racecar/base_link', '/map', rospy.Time(0))
            
            print trans
            print
            print rot

            trs.x = trans[0]
            trs.y = trans[1]
            trs.z = trans[2]

            rtn.x = rot[0]
            rtn.y = rot[1]
            rtn.z = rot[2]
            rtn.w = rot[3]

            br = tf2_ros.TransformBroadcaster()
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "map2"
            t.child_frame_id = "ego_racecar/prova"
            t.transform.translation = trs
            t.transform.rotation = rtn
            br.sendTransform(t)

            #br = tf.TransformBroadcaster()
            #br.sendTransform(trans, rot, rospy.Time.now(), "ego_racecar/base_link", "map2")
            #(lint, angt)= listener.lookupTwist('/map', '/odom', rospy.Time(0), rospy.Duration(0.1))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "NO tf POSE"
            continue
        r.sleep()
    print "FINISHED"


