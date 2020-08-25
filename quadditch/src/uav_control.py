#!/usr/bin/env python3

import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg
import sys

class UAV:
	def __init__(self, uav_id):
		self.ready = False
		self.uav_id = uav_id

		path_base = "/uav"+str(uav_id)+"/mavros/"

		self.state = mavros_msgs.msg.State()
		rospy.Subscriber(path_base + "state", mavros_msgs.msg.State, self.stateCb)

		self.home = False
		rospy.Subscriber(path_base + "home_position/home", mavros_msgs.msg.HomePosition, self.homeCb)

		self.desiredVel = geometry_msgs.msg.Twist(geometry_msgs.msg.Vector3(0,0,0),geometry_msgs.msg.Vector3(0,0,0))
		rospy.Subscriber("/uav"+str(uav_id)+"/unreal/cmd_vel", geometry_msgs.msg.Twist, self.velCmdCb)

		rospy.wait_for_service(path_base + "cmd/land")
		rospy.wait_for_service(path_base + "cmd/takeoff")
		rospy.wait_for_service(path_base + "cmd/arming")
		rospy.wait_for_service(path_base + "set_mode")
		rospy.wait_for_service(path_base + "param/set")
		self.landService	   = rospy.ServiceProxy(path_base + "cmd/land",	mavros_msgs.srv.CommandTOL)
		self.takeoffService    = rospy.ServiceProxy(path_base + "cmd/takeoff", mavros_msgs.srv.CommandTOL)
		self.armService		   = rospy.ServiceProxy(path_base + "cmd/arming",  mavros_msgs.srv.CommandBool)
		self.flightModeService = rospy.ServiceProxy(path_base + "set_mode",	mavros_msgs.srv.SetMode)
		self.paramSetService   = rospy.ServiceProxy(path_base + "param/set", mavros_msgs.srv.ParamSet)

		self.pub_local      = rospy.Publisher(path_base + "setpoint_position/local", geometry_msgs.msg.PoseStamped, queue_size=10)
		self.pub_local_vel  = rospy.Publisher(path_base + "setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, queue_size=10)
		self.seqId = 0
		r = rospy.Rate(5)
		while not self.pub_local.get_num_connections():
			r.sleep() # wait for drone to subscribe to waypoint commands

		while not self.home:
			r.sleep()

		while self.state.system_status != 3:
			r.sleep() # wait for drone to be in MAV_STATE_STANDBY or MAV_STATE_ACTIVE

		self.ready = True
		print("ready")

	def stateCb(self, stateMsg):
		self.state = stateMsg

	def homeCb(self, homeMsg):
		self.home = homeMsg

	def setMode(self, mode = 'OFFBOARD'):
		try:
			self.flightModeService(custom_mode=mode)
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s"%e)

	def setParam(self, paramId, paramValue):
		try:
			self.paramSetService(param_id = paramId, value = mavros_msgs.msg.ParamValue(integer=0, real=paramValue))
		except rospy.ServiceException as e:
			print("service set param call failed: %s"%e)

	def setArmed(self, armed = True):
		try:	
			self.armService(True)
		except rospy.ServiceException as e:
			print("Service arm call failed: %s"%e)

	def setLocalPose(self, movePose):
		header = std_msgs.msg.Header(seq = self.seqId, stamp = rospy.Time.now(), frame_id = "map")	#message header
		self.seqId += 1
		poseStamped = geometry_msgs.msg.PoseStamped(header = header, pose = movePose)	#create the pose message to send
		self.pub_local.publish(poseStamped)

	def setLocalVel(self, moveVel):
		self.pub_local_vel.publish(moveVel)

	def velCmdCb(self, velMsg):
		self.desiredVel = velMsg

if __name__ == '__main__':
	rospy.init_node('uav_control', anonymous=True)

	uavID = int(rospy.get_namespace()[-2])
	uav = UAV(uavID)
	while not uav.ready:
		rospy.sleep(1)
	rospy.loginfo("UAV starting")

	rospy.loginfo("Waiting for home position estimate")
	while not uav.home:
		rospy.sleep(1)

	rospy.loginfo("Arming")
	while not uav.state.armed:
		uav.setArmed(True)
		rospy.sleep(1)
	rospy.loginfo("Armed")

	rospy.loginfo("Entering auto takeoff mode")
	while uav.state.mode!="AUTO.TAKEOFF":
		uav.setMode('AUTO.TAKEOFF')
		rospy.sleep(1)

	while uav.state.mode == "AUTO.TAKEOFF":
		rospy.sleep(1)
	rospy.loginfo("Takeoff complete")

	uav.setParam('MPC_ACC_HOR', 0.1) # horizontal acceleration for jerk limited trajectory mode
	uav.setParam("MPC_ACC_HOR_MAX", 0.1) # horizontal acceleration for line tracking mode
	uav.setParam("MPC_XY_VEL_MAX", 1.0) #max horizontal velocity
	uav.setParam("MPC_Z_VEL_MAX_DN", 0.8)#max descend vel
	uav.setParam("MPC_Z_VEL_MAX_UP", 1.0)#max ascend vel

	r = rospy.Rate(30)
	while not rospy.is_shutdown():
		uav.setMode('OFFBOARD')
		r.sleep()
		uav.setLocalVel(uav.desiredVel)