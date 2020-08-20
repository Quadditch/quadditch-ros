#!/usr/bin/env python2

import rospy
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped

class Translator:
    
    def __init__(self):
        rospy.init_node('translator', anonymous=True)
        ns = rospy.get_namespace()
        base = "/unreal/"
        self.xPub  = rospy.Publisher(base + 'x',  Float32, queue_size=10)
        self.yPub  = rospy.Publisher(base + 'y',  Float32, queue_size=10)
        self.zPub  = rospy.Publisher(base + 'z',  Float32, queue_size=10)
        self.qxPub = rospy.Publisher(base + 'qx', Float32, queue_size=10)
        self.qyPub = rospy.Publisher(base + 'qy', Float32, queue_size=10)
        self.qzPub = rospy.Publisher(base + 'qz', Float32, queue_size=10)
        self.qwPub = rospy.Publisher(base + 'qw', Float32, queue_size=10)

        rospy.Subscriber(ns+'mavros/local_position/pose', PoseStamped, self.poseCallback)
        rospy.spin()

    def poseCallback(self, data):
        self.xPub.publish(data.pose.position.x)
        self.yPub.publish(data.pose.position.y)
        self.zPub.publish(data.pose.position.z)
        self.qxPub.publish(data.pose.orientation.x)
        self.qyPub.publish(data.pose.orientation.y)
        self.qzPub.publish(data.pose.orientation.z)
        self.qwPub.publish(data.pose.orientation.w)



        


if __name__ == '__main__':
    try:
        t = Translator()
    except rospy.ROSInterruptException:
        pass
