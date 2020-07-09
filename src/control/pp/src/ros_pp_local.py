#!/usr/bin/env python2.7
import rospy
import tf
from std_msgs.msg import String, Bool, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion
from muretto.msg import Strategy
import sys

import numpy as np
import thread

from cubic_spline_planner import *
import time

# car sim state
state = { "x": 0, "y": 0, "speed": 0, "yaw": 0}

L = 0.32
k = 0.5
look_ahead = 1.5
max_steer = 0.41
throttle_limit = 0.9
start_throttle_limit = 0.0
exit = False

speed_limit = 2.5
k_p = 2.5

car_data_received = False

local_path = None

def pp_step(target, speed):

    alpha = math.atan2(target[1], target[0] - 0.32)
    delta = alpha
    #Lf = k * speed + look_ahead
    #delta = math.atan2(2.0 * L * math.sin(alpha) / Lf, 1.0)
    if delta > max_steer:
        delta = max_steer
    if delta < -max_steer:
        delta = -max_steer
    return delta

def build_path_msg(xs, ys):
    p_msg = Path()
    p_msg.header.frame_id = "map"
    
    for i in xrange(len(xs)):
        p = PoseStamped()
        p.pose.position.x = xs[i] 
        p.pose.position.y = ys[i]
        p_msg.poses.append(p)
    return p_msg


def point_transform(target, pose, yaw):

    local_straight = [trg[0] - pose[0], trg[1] - pose[1]]
    local_trg = [local_straight[0]*math.cos(yaw) + local_straight[1]*math.sin(yaw), \
        -local_straight[0]*math.sin(yaw) + local_straight[1]*math.cos(yaw)]

    return local_trg

def load_flag_path(file_path):
    file = open(file_path, "r")
    
    xs = []
    ys = []
    zs = []

    while(file.readline()):
        lx = file.readline()
        ly = file.readline()
        lz = file.readline()
        xs.append( float(lx.split(" ")[-1]) )
        ys.append( float(ly.split(" ")[-1]) )
        zs.append( float(lz.split(" ")[-1]) )
    return xs, ys, zs

def estop_callback(data):
    global start_throttle_limit
    if(data.data == False):
        start_throttle_limit = 0.1

def odom_callback(data):

    global car_data_received

    state["x"]      = data.pose.pose.position.x
    state["y"]      = data.pose.pose.position.y
    state["yaw"]    = data.twist.twist.angular.z
    state["vx"]     = data.twist.twist.linear.x
    state["vy"]     = data.twist.twist.linear.y
    state["speed"] = np.sqrt( state["vx"]**2 + state["vy"]**2 )

    o = data.pose.pose.orientation
    orientation = euler_from_quaternion([o.x, o.y, o.z, o.w])

    state["orientation_yaw"] = orientation[-1]

    car_data_received = True

def calc_distance(pose, path_pose):
	return (pose[0] - path_pose[0])**2 + (pose[1] - path_pose[1])**2

'''def localpath_callback(data):
    global local_path
    a=[]
    b=[]

    for i in data.poses:
        a.append(i.pose.position.x)
        b.append(i.pose.position.y)

    local_path = Spline2D(a,b)'''

def localpath_callback(data):
    global local_path
    a=[]
    b=[]
    lx = 0
    ly = 0
    for i in data.poses:
        ax = i.pose.position.x
        ay = i.pose.position.y

        if local_path is not None and ax == local_path.calc_position(0)[0] and ay == local_path.calc_position(0)[1] :
            return

        distance = calc_distance((ax,ay),(lx,ly))
        #print "disance: ", distance
        if i == 0 or distance > 0:
            a.append(ax)
            b.append(ay)
            lx = ax
            ly = ay


    #print("LENGTH: ", len(a))

    local_path = Spline2D(a,b)

def strategy_callback(data):
    if data.race_type == 0:
        global exit
        exit = True
        sys.exit(0)
        
def spin():
    print "spin start"
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_pp_local')
    rospy.Subscriber("commands/stop", Bool, estop_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("local_path", Path, localpath_callback)
    rospy.Subscriber("strategy", Strategy, strategy_callback)

    pub      = rospy.Publisher('drive', AckermannDriveStamped, queue_size=10)
    pub_path = rospy.Publisher('path', Path, queue_size=10)
    pub_pose = rospy.Publisher('/actual_pos', PoseStamped, queue_size=10)
    pub_goal = rospy.Publisher('/target_pos', PoseStamped, queue_size=10)
    pub_fren = rospy.Publisher('fren_pos', Path, queue_size=10)
    pub_time = rospy.Publisher('exec_time', Float64, queue_size=10)
    thread.start_new_thread(spin, ())

    global steer


    steer = 0.1

    # spline to follow

    opt_path = rospy.get_param("~global_trajectory")
    xs, ys, speeds = load_flag_path(opt_path)
    global_path = Spline2D(xs, ys)

    p_msg = build_path_msg([ global_path.calc_position(t)[0] for t in np.linspace(0.1, global_path.s[-1]-0.0001, 200) ],
                           [ global_path.calc_position(t)[1] for t in np.linspace(0.1, global_path.s[-1]-0.0001, 200) ])

    listener = tf.TransformListener()

    max_throttle = 0

    starting_path_pose = []

    max_tracking_error = 0
    path_tracking_error = 0
    last_lap_error = 0
    lap_error = 0
    lap_time = 0

    last_pose = []
    sampling_distance = 0.5
    i_error = 0

    print "START"
    r = rospy.Rate(80)
    while not rospy.is_shutdown():
        if exit:
            break

        if local_path is None:
            print "local path not received"
            continue

        # get pose and update spline coords
        pose = state["x"], state["y"]
        local_path.update_current_s(pose)
        global_path.update_current_s(pose)

        # get pose on spline
        global_path_pose = global_path.calc_position(global_path.cur_s)
        msg_fren = build_path_msg([pose[0], global_path_pose[0]], [pose[1], global_path_pose[1]])
        pub_fren.publish(msg_fren)

        p_pos =  global_path.cur_s + L

        speed_idx = int(( (p_pos) / global_path.s[-1])*len(speeds))
        max_speed = speeds[speed_idx % len(speeds)]
        max_speed = min(max_speed, speed_limit)        # limit abs speed

        '''if(abs(steer) > 0.7):
          look_ahead = 0.5
        elif(abs(steer) > 0.5):
          look_ahead = 1
        elif(abs(steer) > 0.3):
          look_ahead = 1.5
        elif(abs(steer) > 0.1):
          look_ahead = 1.7
        else:
          look_ahead = 2'''

        if(abs(steer) > 0.3):
          look_ahead = 1
        elif(abs(steer) > 0.2):
          look_ahead = 1
        elif(abs(steer) > 0.1):
          look_ahead = 1.1
        else:
          look_ahead = 1.3

        # get target pose
        s_pos = local_path.cur_s + k * max_throttle + look_ahead

        if(start_throttle_limit > 0 and start_throttle_limit<throttle_limit):
            start_throttle_limit += 1.0/40.0
            starting_pose = global_path_pose
            last_pose = pose
            t = time.time()
            print "START: ", start_throttle_limit

        trg = local_path.calc_position(s_pos)
        trg = [ trg[0], trg[1] ]
        loc_trg = point_transform(local_path.calc_position(s_pos), pose, state["orientation_yaw"])

        mpc_path = build_path_msg([L, loc_trg[0]], [0, loc_trg[1]])
        mpc_path.header.frame_id = "ego_racecar/base_link"

        steer = pp_step(loc_trg, max_speed)

        print "STEER: ", steer
        print "CURRENT SPEED: ", state["speed"]
        print "TARGET SPEED: ", max_speed
        print "YAW: ", state["yaw"]
           
        #----------- PATH TRACKING AND LAP TIME -----------

        if(last_pose and calc_distance(last_pose, global_path_pose) > sampling_distance):
            i_error += 1
            path_tracking_error = calc_distance(pose, global_path_pose)
            lap_error += path_tracking_error
            if(path_tracking_error > max_tracking_error):
                max_tracking_error = path_tracking_error

            if(calc_distance(global_path_pose, starting_pose) < 1 and i_error > 50):
                last_lap_error = lap_error / i_error
                lap_time = time.time() - t
                max_tracking_error = 0
                i_error = 0
                lap_error = 0
                t = time.time()

            last_pose = pose

        print "-----------------------------------ACTUAL TRACKING ERROR: ", path_tracking_error
        print "-----------------------------------MAX TRACKING ERROR: ", max_tracking_error
        print "-----------------------------------LAST LAP ERROR: ", last_lap_error
        print "-----------------------------------LAST LAP TIME: ", lap_time

        p = PoseStamped()
        p.header.frame_id = 'map'
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]

        g = PoseStamped()
        g.header.frame_id = 'map'
        g.pose.position.x = trg[0] 
        g.pose.position.y = trg[1]

        # actuate
        msg = AckermannDriveStamped()
        msg.drive.speed = max_speed
        msg.drive.steering_angle = steer

        pub.publish(msg)
        pub_pose.publish(p)
        pub_goal.publish(g)
        pub_path.publish(p_msg)
        r.sleep()

