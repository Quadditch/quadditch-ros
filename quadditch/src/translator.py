#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped, TwistStamped

class Translator:
    
    def __init__(self):
        rospy.init_node('translator', anonymous=True)
        ns = rospy.get_namespace()
        base = ns+"unreal/"
        self.xPub  = rospy.Publisher(base + 'x',  Float32, queue_size=10)
        self.yPub  = rospy.Publisher(base + 'y',  Float32, queue_size=10)
        self.zPub  = rospy.Publisher(base + 'z',  Float32, queue_size=10)
        self.qxPub = rospy.Publisher(base + 'qx', Float32, queue_size=10)
        self.qyPub = rospy.Publisher(base + 'qy', Float32, queue_size=10)
        self.qzPub = rospy.Publisher(base + 'qz', Float32, queue_size=10)
        self.qwPub = rospy.Publisher(base + 'qw', Float32, queue_size=10)

        self.vxPub = rospy.Publisher(base + 'vx', Float32, queue_size=10)
        self.vyPub = rospy.Publisher(base + 'vy', Float32, queue_size=10)
        self.vzPub = rospy.Publisher(base + 'vz', Float32, queue_size=10)
        self.wxPub = rospy.Publisher(base + 'wx', Float32, queue_size=10)
        self.wyPub = rospy.Publisher(base + 'wy', Float32, queue_size=10)
        self.wzPub = rospy.Publisher(base + 'wz', Float32, queue_size=10)

        rospy.Subscriber(ns+'mavros/local_position/pose', PoseStamped, self.poseCallback)
        rospy.Subscriber(ns+'mavros/local_position/velocity_local', TwistStamped, self.twistCallback)
        rospy.spin()

    def poseCallback(self, poseMsg):
        self.xPub.publish(poseMsg.pose.position.x)
        self.yPub.publish(poseMsg.pose.position.y)
        self.zPub.publish(poseMsg.pose.position.z)
        self.qxPub.publish(poseMsg.pose.orientation.x)
        self.qyPub.publish(poseMsg.pose.orientation.y)
        self.qzPub.publish(poseMsg.pose.orientation.z)
        self.qwPub.publish(poseMsg.pose.orientation.w)  

    def twistCallback(self, twistMsg):
        self.vxPub.publish(twistMsg.twist.linear.x)
        self.vyPub.publish(twistMsg.twist.linear.y)
        self.vzPub.publish(twistMsg.twist.linear.z)
        self.wxPub.publish(twistMsg.twist.angular.x)
        self.wyPub.publish(twistMsg.twist.angular.y)
        self.wzPub.publish(twistMsg.twist.angular.z)


        


if __name__ == '__main__':
    try:
        t = Translator()
    except rospy.ROSInterruptException:
        pass
