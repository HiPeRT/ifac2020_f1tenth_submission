import rospy
import tf
from std_msgs.msg import String, Bool, Float64
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped
from tf.transformations import euler_from_quaternion

import numpy as np
import thread
import time

from cubic_spline_planner import *
from casadi import *
from casadi.tools import *

# mpc control of a Ackermann driven car
# works with F1tenth-sim
# Gatti's initial implementation, adjusted by Ayoub

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

speed_limit = 10
start_throttle_limit = 0.0
throttle_limit = 1.0

max_throttle = 0.99
min_throttle = -0.99

max_steer = 0.41
min_steer = -0.41

car_data_received = False

global path;

def casadi_model():
    global F, sim, state

    # Control
    # Create 1r-2c matrix containing control inputs. 
    # Set steer and throttle as elements
    u = MX.sym("u",2)
    steer = u[0]
    speed_input = u[1]

    model = "kin"

    # Constants - Model parameters
    L = 0.33 # wheelbase
    Lr = 0.17145 
    Lf = 0.15875

    ca = 1.633
    cm = 0.2
    ch = 4

    # State
    x = MX.sym("x",4)
    sx    = x[0]  # position x
    sy    = x[1]  # position y
    yaw   = x[2]  # yaw
    speed = x[3]
    throttle = 16
    # ODE right hand side


    if model == "kin":
        sxdot    = speed_input*cos(yaw)
        sydot    = speed_input*sin(yaw)
        yawdot   = (speed_input/L)*tan(steer)
        #speeddot = -ca*speed_input + ca*cm*(throttle-ch)
        speeddot = speed_input
    elif model == "kin_slipangle":
        slipangle = atan(tan(steer)*(Lr/(Lr+Lf)))
        sxdot    = speed_input*cos(yaw + slipangle)
        sydot    = speed_input*sin(yaw + slipangle)
        yawdot   = (speed_input*cos(slipangle)/L)*tan(steer)
        speeddot = speed_input
        #speeddot = -ca*speed + ca*cm*(throttle-ch)

    # Concatenate vertically the expressions creating a row vector
    xdot = vertcat(sxdot, sydot, yawdot, speeddot)

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
    Ut = MX.sym("U",nu) #speed control input

    # Initial conditions
    x0 = [0, 0, 0, state["speed"]]
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
            gain=1
        J += gain*(X[0]-target[k][0])**2 #longitudinal error cost
        J += gain*(X[1]-target[k][1])**2 #lateral error cost
        #J += gain*(X[2])**2 #yaw cost
        #J += 2*(Us[i]**2)/(X[1]**2+0.1)
    G = X[3] #speed


    # Objective function and constraints
    J += mtimes(Us.T,Us)*1
    J += mtimes(Ut.T,Ut)*1

    # NLP
    nlp = {'x':vertcat(Us, Ut), 'f':J, 'g':G}
    
    # Allocate an NLP solver
    opts = {"ipopt.tol":1e-10, "ipopt.print_level":0, "expand":True}
    solver = nlpsol("solver", "ipopt", nlp, opts)
    arg = {}

    lower_bound_speed = target[1][2]

    # Bounds on u and initial condition
    arg["lbx"] =  vertcat(min_steer*np.ones(nu), lower_bound_speed*np.ones(nu)) # lower bound for steer and drive command 
    arg["ubx"] =  vertcat(max_steer*np.ones(nu), speed_limit*np.ones(nu)) # upper bound for steer and drive
    arg["x0"] =    0.0 # first guess 0.4 per tutti i controlli

    # Bounds on g
    arg["lbg"] = lower_bound_speed
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

def point_transform(target, pose, yaw):

    local_straight = [target[0] - pose[0], target[1] - pose[1]]
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

def estop_callback(data):
    global start_throttle_limit
    if(data.data == False):
        start_throttle_limit = 0.1

def calc_distance(pose, path_pose):
	return (pose[0] - path_pose[0])**2 + (pose[1] - path_pose[1])**2

def spin():
    print "spin start"
    rospy.spin()

if __name__ == '__main__':
    rospy.init_node('ros_mpc_kin')

    rospy.Subscriber("commands/stop", Bool, estop_callback)
    rospy.Subscriber("odom", Odometry, odom_callback)
    pub      = rospy.Publisher('drive', AckermannDriveStamped, queue_size=1)
    pub_path = rospy.Publisher('path', Path, queue_size=1)
    pub_pose = rospy.Publisher('/actual_pos', PoseStamped, queue_size=1)
    pub_goal = rospy.Publisher('/target_pos', PoseStamped, queue_size=1)

    pub_mpc_path = rospy.Publisher('mpc_path', Path, queue_size=1)
    pub_fren = rospy.Publisher('fren_pos', Path, queue_size=1)
    pub_time = rospy.Publisher('exec_time', Float64, queue_size=1)
    thread.start_new_thread(spin, ())

    global state

    # spline to follow
    xs, ys, speeds = load_flag_path("../../global_planner/berlin/opt3.txt")
    path = Spline2D(xs, ys)
    p_msg = build_path_msg([ path.calc_position(t)[0] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ],
                           [ path.calc_position(t)[1] for t in np.linspace(0.1, path.s[-1]-0.0001, 200) ])

    casadi_model()

    max_speed = 0

    starting_path_pose = []

    max_tracking_error = 0
    path_tracking_error = 0
    last_lap_error = 0
    lap_error = 0
    lap_time = 0

    last_pose = []
    sampling_distance = 0.5
    i_error = 0
  
    max_time = 0

    starting_pose = None

    print "START"
    r = rospy.Rate(80) 
    while not rospy.is_shutdown():
        t3 = time.time()

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

        # get future horizon targets pose and speed
        targets = [ ]
        s_pos = path.cur_s
        for i in xrange(N):
            curv = path.calc_curvature(s_pos)    
            speed_idx = int((s_pos / path.s[-1])*len(speeds))
            max_speed = speeds[speed_idx % len(speeds)]
            if(i==(N-1)):
                print "SPEED ", max_speed, state["speed"]

            #max_speed = max_speed + 2 #boost
            #step = max_speed+4 if max_speed > 6 else max_speed+3

            step = max_speed+2
            look_ahead = (step)*dt
            trg = point_transform(path.calc_position(s_pos), pose, state["orientation_yaw"])

            max_speed = min(max_speed, speed_limit)

            trg = [ trg[0], trg[1], max_speed ]
            targets.append(trg)
            s_pos += look_ahead

        temp = time.time() - t3
        print "dopo horizon TIME: ", temp*1000

        # Publish mpc horizon path
        targets_resh = np.reshape(targets, (N,3))
        msg_mpc = build_path_msg(targets_resh[:,0], targets_resh[:,1])
        msg_mpc.header.frame_id = "ego_racecar/base_link"
        pub_mpc_path.publish(msg_mpc)

        # optimize controls to reach target
        t2 = time.time()
        speed, steer, mpc_path_msg = opt_step(targets)
        opt_time = time.time() - t2
        print "OPT TIME: ", opt_time*1000
        pub_time.publish(opt_time*1000)

        if opt_time > max_time and opt_time < 100:
            max_time = opt_time

        #print "MAX TIME: ", max_time*1000

        # Limit starting acceleration
        if(starting_pose is None and state["speed"] > 0):
            starting_pose = path_pose
            last_pose = pose
            t = time.time()

        print "CONTROL: ", speed, steer 

        if(last_pose and calc_distance(last_pose, pose) > sampling_distance):
            i_error += 1
            path_tracking_error = calc_distance(pose, path_pose)
            lap_error += path_tracking_error
            if(path_tracking_error > max_tracking_error):
                max_tracking_error = path_tracking_error

            if(calc_distance(path_pose, starting_pose) < 1 and i_error > 10):
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
        
        # actuate
        msg = AckermannDriveStamped()
        msg.drive.speed = speed
        msg.drive.steering_angle = steer

        pub.publish(msg)
        pub_path.publish(p_msg)
        print "TOTAL TIME: ", (time.time() - t3)*1000
        r.sleep()
