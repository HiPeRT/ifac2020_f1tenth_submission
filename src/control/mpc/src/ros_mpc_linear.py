import rospy
import tf
from std_msgs.msg import String, Bool, Float64
from ackermann_msgs.msg import AckermannDrive
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PointStamped

import numpy as np
import thread
import time

from cubic_spline_planner import *
from casadi import *
from casadi.tools import *

# mpc control of a Ackermann driven car
# works with F1tenth-sim
# Gatti's initial implementation, adjusted for dynamic model by Ayoub

# MPC time
T =  1.0
dt = 0.1
N = int(T/dt)

L = 0.33
k = 0

# car sim state
state = { "x": 0, "y": 0, "speed": 0, "yaw": 0}

# model function
F = None
sim = None

throttle_limit = 0.9
start_throttle_limit = 0.0

speed_limit = 20

max_throttle = 0.99
min_throttle = -0.99

max_steer = 0.4
min_steer = -0.4

global path;

def casadi_model():
    global F, sim, state

    # Control
    # Create 1r-2c matrix containing control inputs. 
    # Set steer and throttle as elements
    u = MX.sym("u",2)
    steer = u[0]
    throttle = u[1]

    model = "dyn"

    # Constants - Model parameters
    L = 0.33 # wheelbase
    Lr = 0.17145 
    Lf = 0.15875

    Cm1 = 8.0#8 # Motor model constant
    Cm2 = 1.4#1.4 
    Cd =  0.01#0.01 # Drag
    Cr =  0.0#1.1 # Rolling resistance
    Ca =  3.1#1.1

    ca = 1.633
    cm = 0.2
    ch = 4

    m  =  3.47 #Vehicle mass    
    Iz = 0.04712 #Inertia
    #car_length = 0
    #car_width = 0

    # Simplified Pacejka tire model

    #30s and 0.072 avg error
    B_r = 7.3
    B_f = 7.3
    C_r = 1.2
    C_f = 1.2
    D_f = 1.3
    D_r = 1.3

    Cs_f = 4.7180
    Cs_r = 5.4562

    #B stiffness factor
    #C shape factor (scalar value)
    #D peak value from lateral force in the tire force curvature
    #E curvature factor (NOT USED HERE)

    # State
    x = MX.sym("x",5)
    sx    = x[0]  # position x
    sy    = x[1]  # position y
    yaw   = x[2]  # yaw
    vx = x[3]     # longitudinal velocity
    vy = x[4]     # lateral velocity
    speed = np.sqrt(vx**2 + vy**2)

    #Slip angles
    upper_R = vy - Lr*yaw
    upper_F = vy + Lf*yaw
    alpha_R = atan(upper_R/vx)
    alpha_F = atan(upper_F/vx) - steer

    # ODE right hand side

    F_rx = (Cm1 - Cm2*speed)*throttle - Cd*speed**2 - Cr - Ca*(speed*steer)**2
    #F_rx = Cm1*throttle - Cr - Cd*speed**2
    #F_rx = Cm1*throttle
    F_ry = D_r*sin(C_r*atan(B_r*alpha_R))
    F_fy = D_f*sin(C_f*atan(B_f*alpha_F))

    sxdot  = speed*cos(yaw)
    sydot  = speed*sin(yaw)
    vxdot  = (F_rx - F_fy*sin(steer) + m*vy*yaw)/m
    vydot  = (F_ry + F_fy*cos(steer) - m*vx*yaw)/m
    yawdot = (F_fy*Lf*cos(steer) - F_ry*Lr)/Iz

    # Concatenate vertically the expressions creating a row vector
    xdot = vertcat(sxdot, sydot, yawdot, vxdot, vydot)

    # ODE right hand side function
    # as input are used the state and control inputs,
    # and as output the row vector containing the model expressions
    f = Function('f', [x,u],[xdot])

    # Integrate the step with Explicit Euler
    IN = 1
    xj = x
    for i in xrange(IN):
        fj = f(xj,u)
        xj += dt*fj/IN

    # Discrete time dynamics function
    # Funzione che mette in relazione lo stato attuale a quello futuro
    F = Function('F', [x,u],[xj])

def opt_step(target):
    global F
    global last_ctrl

    # Control for all segments
    nu = N #number of states
    Us = MX.sym("U",nu) #steer control input
    Ut = MX.sym("U",nu) #throttle control input

    # Initial conditions
    x0 = [0, 0, 0, state["vx"], state["vy"]]
    X0 = MX(x0) # vector containing the initial state

    J = 0 # objective function that should be minimized by the nlp solver

    # build graph
    X=X0
    G = None
    lbg = []
    ubg = []

    # For every temporal step
    for k in range(nu):
        X = F(X, vertcat(Us[k], Ut[k])) #Integrate the step
        gain = 1
        # give more importance to the last step using a bigger gain
        if(k == nu-1):
            gain=150
        J += gain*(X[0]-target[k][0])**2 #longitudinal error cost
        J += gain*(X[1]-target[k][1])**2 #lateral error cost
        #J += 2*gain*(target[k][2]-state["speed"])
        #J += gain*(X[2])**2 #yaw cost
        #J += 2*(Us[i]**2)/(X[1]**2+0.1)
    G = X[3]


    # Objective function and constraints
    J += mtimes(Us.T,Us)*1000
    J += mtimes(Ut.T,Ut)*5

    # NLP
    nlp = {'x':vertcat(Us, Ut), 'f':J, 'g':G}
    
    # Allocate an NLP solver
    opts = {"ipopt.tol":1e-10, "ipopt.print_level":0, "expand":True}
    solver = nlpsol("solver", "ipopt", nlp, opts)
    arg = {}

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(min_steer*np.ones(nu), min_throttle*np.ones(nu)) # lower bound for steer and drive command 
    arg["ubx"] =  vertcat(max_steer*np.ones(nu), max_throttle*np.ones(nu)) # upper bound for steer and drive
    arg["x0"] =    0.0 # first guess 0.4 per tutti i controlli

    # Bounds on g
    arg["lbg"] = 0
    arg["ubg"] = inf

    # Solve the problem
    res = solver(**arg)
    print "f:", res["f"]
    ctrls = reshape(res["x"], (nu,2)).T #reshape to have a row for each step
    ctrls = reshape(ctrls, (2, nu)) #reshape to have as rows steer and throttle, and as columns the steps


    print [ t[2] for t in targets ]
    #print "CTRLS: ", ctrls[1,:]
    #print "OUT: ", out[:,3]
    return ctrls[1][0], ctrls[0][0], None# out_path_msg


def build_path_msg(xs, ys):
    p_msg = Path()
    p_msg.header.frame_id = "map"
    
    for i in xrange(len(xs)):
        p = PoseStamped()
        p.pose.position.x = xs[i] 
        p.pose.position.y = ys[i]
        p_msg.poses.append(p)
    return p_msg


def point_transform(tf_lis, pos):
    target_global = PointStamped()
    target_global.header.frame_id = "map"
    target_global.header.stamp =rospy.Time(0)
    target_global.point.x=pos[0]
    target_global.point.y=pos[1]
    target_global.point.z=0.0
    target = listener.transformPoint("odom",target_global)
    return [target.point.x, target.point.y]

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

# Get data from unity sim
def callback(data):
    x = np.fromstring(data.data, dtype=float, sep=' ')
	
    if x[0] == 0:
        path.cur_s = 0;

    state["x"] = x[2]
    state["y"] = x[3]
    #state["yaw"] = x[4] 
    state["speed"] = np.sqrt( x[5]**2 + x[6]**2 )
    state["vx"] = x[5]
    state["vy"] = x[6]

def estop_callback(data):
    global start_throttle_limit
    if(data.data == False):
        start_throttle_limit = 0.1

def calc_distance(pose, path_pose):
	return (pose[0] - path_pose[0])**2 + (pose[1] - path_pose[1])**2

def yaw_callback(data):
    global state
    state["yaw"] = data.data

def spin():
    print "spin start"
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_mpc')
    rospy.Subscriber("data", String, callback)
    rospy.Subscriber("commands/stop", Bool, estop_callback)
    rospy.Subscriber("yaw_rate", Float64, yaw_callback)
    pub      = rospy.Publisher('drive_parameters', AckermannDrive, queue_size=1)
    pub_path = rospy.Publisher('path', Path, queue_size=1)
    pub_time = rospy.Publisher('exec_time', Float64, queue_size=1)
    pub_mpc_path = rospy.Publisher('mpc_path', Path, queue_size=1)
    pub_fren = rospy.Publisher('fren_pos', Path, queue_size=1)
    thread.start_new_thread(spin, ())

    global state

    # spline to follow
    #xs, ys, speeds = load_flag_path("../maps/unity/cleaned_opt.txt")
    xs, ys, speeds = load_flag_path("../maps/unity/race_maybe_too_much.trj")
    path = Spline2D(xs, ys)
    p_msg = build_path_msg([ path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                           [ path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])

    casadi_model()

    listener = tf.TransformListener()

    max_speed = 0

    starting_path_pose = []

    max_tracking_error = 0
    path_tracking_error = 0
    last_lap_error = 0
    lap_error = 0
    lap_time = 0
    lap_yaw = 0
    last_yaw = 0

    last_pose = []
    sampling_distance = 0.5
    i_error = 0

    max_time = 0

    print "START"
    r = rospy.Rate(40) 
    while not rospy.is_shutdown():
        t3 = time.time()
        try:
            (trans,rot) = listener.lookupTransform('/map', '/odom', rospy.Time(0))
            (lint, angt)= listener.lookupTwist('/map', '/odom', rospy.Time(0), rospy.Duration(0.1))
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            print "NO tf POSE"
            continue

        # get pose and update spline coords
        pose = trans[0], trans[1]
        path.update_current_s(pose)

        # get pose on spline
        path_pose = path.calc_position(path.cur_s)
        msg_fren = build_path_msg([pose[0], path_pose[0]], [pose[1], path_pose[1]])
        pub_fren.publish(msg_fren)

        # get future horizon targets pose and speed
        targets = [ ]
        s_pos = path.cur_s
        for i in xrange(N):
            curv = path.calc_curvature(s_pos)    
            speed_idx = int((s_pos / path.s[-1])*len(speeds))
            max_speed = speeds[speed_idx % len(speeds)]

            #max_speed = 10

            if(i==0):
                print "SPEED ", max_speed, state["speed"]

            # Static horizon lookahead
            #look_ahead = 1.2

            # Dynamic horizon lookahead
            look_ahead = (max_speed+2)*dt

            trg = point_transform(listener, path.calc_position(s_pos))

            # Using the next target, avoiding the initial one, brings fast lap but less accuracy
            #trg = point_transform(listener, path.calc_position(s_pos + look_ahead))

            #SPEED UP
            max_speed = max_speed+5 if max_speed > 4 else max_speed
            max_speed = min(max_speed, speed_limit)

            trg = [ trg[0], trg[1], max_speed ]
            targets.append(trg)
            s_pos += look_ahead

        # Publish mpc horizon path
        targets_resh = np.reshape(targets, (N,3))
        msg_mpc = build_path_msg(targets_resh[:,0], targets_resh[:,1])
        msg_mpc.header.frame_id = "odom"
        pub_mpc_path.publish(msg_mpc)

        # optimize controls to reach target
        t2 = time.time()
        throttle, steer, mpc_path_msg = opt_step(targets)
        opt_time = time.time() - t2
        print "OPT TIME: ", opt_time*1000
        pub_time.publish(opt_time*1000)

        if opt_time > max_time and opt_time < 100:
            max_time = opt_time

        print "MAX TIME: ", max_time*1000

        # Limit starting acceleration
        if(start_throttle_limit > 0 and start_throttle_limit<throttle_limit):
            start_throttle_limit += 1.0/40.0
            starting_pose = path_pose
            last_pose = pose
            t = time.time()
            print "START: ", start_throttle_limit

        throttle = min(throttle, throttle_limit)         # limit abs speed
        throttle = min(throttle, start_throttle_limit)   # limit speed at start

        print "CONTROL: ", throttle, steer 

        if(last_pose and calc_distance(last_pose, pose) > sampling_distance):
            i_error += 1
            path_tracking_error = calc_distance(pose, path_pose)
            lap_error += path_tracking_error
            lap_yaw += abs(state["yaw"])
            if(path_tracking_error > max_tracking_error):
                max_tracking_error = path_tracking_error

            if(calc_distance(path_pose, starting_pose) < 1 and i_error > 50):
                last_lap_error = lap_error / i_error
                last_yaw = lap_yaw / i_error            
                lap_time = time.time() - t
                max_tracking_error = 0
                i_error = 0
                lap_yaw = 0
                lap_error = 0
                t = time.time()

            last_pose = pose

        print "-----------------------------------ACTUAL TRACKING ERROR: ", path_tracking_error
        print "-----------------------------------MAX TRACKING ERROR: ", max_tracking_error
        print "-----------------------------------LAST LAP ERROR: ", last_lap_error
        print "-----------------------------------LAST LAP TIME: ", lap_time
        print "-----------------------------------LAST YAW: ", last_yaw
        
        # actuate
        msg = AckermannDrive()
        msg.speed = throttle
        msg.steering_angle = steer
        print "ANGLE: ", msg.steering_angle
        pub.publish(msg)
        pub_path.publish(p_msg)
        print "TOTAL TIME: ", (time.time() - t3)*1000
        r.sleep()
