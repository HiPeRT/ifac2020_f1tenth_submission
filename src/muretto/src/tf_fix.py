import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped, PointStamped
import time
import thread
import sys
import geometry_msgs
import tf2_ros
import tf_conversions

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

if __name__ == "__main__":
    rospy.init_node('save_tf')
    thread.start_new_thread(spin, ())
    listener = tf.TransformListener()
    started = False
    print "START"
    r = rospy.Rate(40)
    while not rospy.is_shutdown():

        try:
            print "working"
            (trans,rot) = listener.lookupTransform('ego_racecar/base_link', '/map', rospy.Time(0))
            
            print trans
            print
            print rot

            trs = translation()
            trs.x = trans[0]
            trs.y = trans[1]
            trs.z = trans[2]

            rtn = rotation()
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


