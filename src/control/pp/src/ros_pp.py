import rospy
import tf
from std_msgs.msg import String, Bool, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion

import numpy as np
import thread

from cubic_spline_planner import *
import time

# car sim state
state = { "x": 0, "y": 0, "speed": 0, "yaw": 0}

L = 0.33
k = 0
look_ahead = 3
max_steer = 0.41

car_data_received = False

speed_limit = 10

def pp_step(target, speed):

    alpha = math.atan2(target[1], target[0] - L/2)
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

def point_transform(target, pose, yaw):

    local_straight = [trg[0] - pose[0], trg[1] - pose[1]]
    local_trg = [local_straight[0]*math.cos(yaw) + local_straight[1]*math.sin(yaw), \
        -local_straight[0]*math.sin(yaw) + local_straight[1]*math.cos(yaw)]

    return local_trg

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

def spin():
    print "spin start"
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_pp')
    rospy.Subscriber("/opp/commands/stop", Bool, estop_callback)
    rospy.Subscriber("/opp/odom", Odometry, odom_callback)
    pub      = rospy.Publisher('/opp/drive', AckermannDriveStamped, queue_size=10)
    pub_path = rospy.Publisher('/opp/path', Path, queue_size=10)
    pub_pose = rospy.Publisher('/opp/actual_pos', PoseStamped, queue_size=10)
    pub_goal = rospy.Publisher('/opp/target_pos', PoseStamped, queue_size=10)
    pub_fren = rospy.Publisher('/opp/fren_pos', Path, queue_size=10)
    pub_time = rospy.Publisher('/opp/exec_time', Float64, queue_size=10)
    thread.start_new_thread(spin, ())

    global steer

    steer = 0.1

    # spline to follow
    xs, ys, speeds = load_flag_path("../../../muretto/conf/fattibile.txt")
    path = Spline2D(xs, ys)
    p_msg = build_path_msg([ path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                           [ path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])

    listener = tf.TransformListener()

    last_pose = []
    sampling_distance = 0.5
    i_error = 0

    print "START"
    r = rospy.Rate(80) 
    while not rospy.is_shutdown():

        t2 = time.time()

        if not car_data_received:
            print "car data not received yet"
            continue

        # get pose and update spline coords
        pose = state["x"], state["y"]
        path.update_current_s(pose)

        # get pose on spline
        path_pose = path.calc_position(path.cur_s)
        msg_fren = build_path_msg([pose[0], path_pose[0]], [pose[1], path_pose[1]])
        pub_fren.publish(msg_fren)

        p_pos =  path.cur_s + L

        speed_idx = int(( (p_pos) / path.s[-1])*len(speeds))
        max_speed = speeds[speed_idx % len(speeds)]
        max_speed = max_speed + 0.5 #boost
        max_speed = min(max_speed, speed_limit)         # limit abs speed

        if(abs(steer) > 0.7):
          look_ahead = 0.5
        elif(abs(steer) > 0.5):
          look_ahead = 1
        elif(abs(steer) > 0.3):
          look_ahead = 1.5
        elif(abs(steer) > 0.1):
          look_ahead = 1.7
        else:
          look_ahead = 2

        # get target pose
        s_pos = path.cur_s + k * max_speed + look_ahead

        trg = path.calc_position(s_pos)
        trg = [ trg[0], trg[1] ]
        loc_trg = point_transform(trg, pose, state["orientation_yaw"])

        mpc_path = build_path_msg([L, loc_trg[0]], [0, loc_trg[1]])
        mpc_path.header.frame_id = "ego_racecar/base_link"

        temp = time.time() - t2
        print "dopo target TIME: ", temp*1000

        steer = pp_step(loc_trg, max_speed)

        print "STEER: ", steer
        print "CURRENT SPEED: ", state["speed"]
        print "TARGET SPEED: ", max_speed

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
        tot_time = time.time() - t2
        print "TIME: ", tot_time*1000
        pub_time.publish(tot_time*1000)
        r.sleep()
