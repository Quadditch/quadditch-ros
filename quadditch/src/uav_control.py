#!/usr/bin/env python2

import rospy
import mavros_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg

class UAV:
	def __init__(self, uav_id):
		self.ready = False
		self.uav_id = uav_id

		path_base = "/uav"+str(uav_id)+"/mavros/"

		self.state = mavros_msgs.msg.State()
		rospy.Subscriber(path_base + "state", mavros_msgs.msg.State, self.stateCb)

		self.estimate = False
		rospy.Subscriber(path_base + "local_position/pose", geometry_msgs.msg.PoseStamped, self.locPosCb)

		rospy.wait_for_service(path_base + "cmd/land")
		rospy.wait_for_service(path_base + "cmd/takeoff")
		rospy.wait_for_service(path_base + "cmd/arming")
		rospy.wait_for_service(path_base + "set_mode")
		self.landService	   = rospy.ServiceProxy(path_base + "cmd/land",	mavros_msgs.srv.CommandTOL)
		self.takeoffService	= rospy.ServiceProxy(path_base + "cmd/takeoff", mavros_msgs.srv.CommandTOL)
		self.armService		= rospy.ServiceProxy(path_base + "cmd/arming",  mavros_msgs.srv.CommandBool)
		self.flightModeService = rospy.ServiceProxy(path_base + "set_mode",	mavros_msgs.srv.SetMode)

		self.pub_local = rospy.Publisher(path_base + "setpoint_position/local", geometry_msgs.msg.PoseStamped, queue_size=10)
		self.seqId = 0
		r = rospy.Rate(5)
		while not self.pub_local.get_num_connections():
			r.sleep() # wait for drone to subscribe to waypoint commands

		while not self.estimate:
			r.sleep()

		while self.state.system_status != 3:
			r.sleep() # wait for drone to be in MAV_STATE_STANDBY or MAV_STATE_ACTIVE

		self.ready = True
		print("ready")

	def stateCb(self, stateMsg):
		self.state = stateMsg

	def locPosCb(self, poseMsg):
		self.estimate = True

	def setMode(self, mode = 'GUIDED'):
		try:
			self.flightModeService(custom_mode=mode)
		except rospy.ServiceException as e:
			print("service set_mode call failed: %s"%e)

	def setArmed(self, armed = True):
		try:	
			self.armService(armed)
		except rospy.ServiceException as e:
			print("Service arm call failed: %s"%e)

	def setTakeoffMode(self, altitude = 5):
		try:
			self.takeoffService(altitude = altitude, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
		except rospy.ServiceException as e:
			print("Service takeoff call failed: %s"%e)

	def setLandMode(self):
		try:
			self.landService(altitude = 0, latitude = 0, longitude = 0, min_pitch = 0, yaw = 0)
		except rospy.ServiceException as e:
			print("Service land call failed: %s"%e)

	def setLocalPos(self, movePose):
		header = std_msgs.msg.Header(seq = self.seqId, stamp = rospy.Time.now(), frame_id = "map")	#message header
		self.seqId += 1
		poseStamped = geometry_msgs.msg.PoseStamped(header = header, pose = movePose)	#create the pose message to send
		self.pub_local.publish(poseStamped)

if __name__ == '__main__':
	#print("this is a module, not meant to be run")

	rospy.init_node('uav_control', anonymous=True)

	uavID = int(rospy.get_namespace()[-2])
	uav = UAV(uavID)
	while not uav.ready:
		rospy.sleep(1)
	rospy.loginfo("UAV starting")

	rospy.loginfo("Entering Guided mode")
	while not uav.state.guided:
		uav.setMode('GUIDED')
		rospy.sleep(1)

	rospy.loginfo("Waiting for pose estimate")
	while not uav.ready:
		rospy.sleep(1)

	rospy.loginfo("Arming")
	while not uav.state.armed:
		uav.setArmed(True)
		rospy.sleep(1)
	rospy.loginfo("Armed")

	rospy.loginfo("Taking off")
	while uav.state.system_status == 3:
  		uav.setTakeoffMode()
		rospy.sleep(1)
	rospy.loginfo("Takeoff complete")
