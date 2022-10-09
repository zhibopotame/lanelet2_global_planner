#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
import tf

odom = Odometry()
odom.pose.pose.orientation.w = 1.0
tf_br = tf.TransformBroadcaster()
odom_pub = 0

def callback_init_pose(msg):
    print(msg)
    odom.pose.pose = msg.pose.pose        

def publish_odom():
    odom.header.stamp = rospy.Time.now()
    odom.header.frame_id = '/odom'
    odom.child_frame_id = '/base_link'

    # odom.pose.pose.position = Point(self.x, self.y, self.z)
    # odom.pose.pose.orientation = Quaternion(*(kdl.Rotation.RPY(R, P, Y).GetQuaternion()))

    pos = (odom.pose.pose.position.x,
            odom.pose.pose.position.y,
            odom.pose.pose.position.z)

    ori = (odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y,
            odom.pose.pose.orientation.z,
            odom.pose.pose.orientation.w)

    odom_pub.publish(odom)
    tf_br.sendTransform(pos, ori, odom.header.stamp, odom.child_frame_id, odom.header.frame_id)

if __name__ == '__main__':
    rospy.init_node("fake_odom")
    rospy.Subscriber("/initialpose", PoseWithCovarianceStamped, callback_init_pose, queue_size=1)
    odom_pub = rospy.Publisher("/fake_odom", Odometry, queue_size = 1)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        publish_odom()
        rate.sleep()