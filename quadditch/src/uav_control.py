#!/usr/bin/env python

import rospy
import mavros_msgs.msg
import std_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import sys


#ToDo fix tabs and spaces

num_drones = 8

alt_ground = 587 + 32.8  # need to add offset for some reason
alt_standard = 8
alt_min = 5

alt_layer_inc = 1.5
alt_max = alt_min + alt_layer_inc*num_drones

#location = "swatara"
location = "cage"

if location == "cage":
	cage_origin = (37.22305889702098, -80.43259539019778, alt_ground)
	landing_positions = [(37.222895, -80.432404),
						(37.222977, -80.432511),
						(37.223041, -80.432635),
						(37.223107, -80.432737)]
	starting_positions = [	(37.222859, -80.432452),
							(37.222938, -80.432361),
							(37.223184, -80.432906),
							(37.223252, -80.432790)]

elif location == "swatara":
	cage_origin=(40.291227, -76.672903, alt_ground)
	landing_positions = [(40.291227, -76.672903),
	                     (40.201227, -76.672903),
	                     (40.211227, -76.672903),
	                     (40.221227, -76.672903) ]
	starting_positions = [(40.200889, -76.672399),
						  (40.210889, -76.672399),
						  (40.220889, -76.672399),
						  (40.290889, -76.672399)]


class UAV:
	def __init__(self, uav_id):
		self.ready = False
		self.home_set = False
		self.uav_id = uav_id
		self.TOL = None # takeoff or landing mode
		self.TOL_state = None # prep, move, finish
		self.land_index = 0 # to be set by game master at landing time
		self.alt_sorted = alt_standard
		self.land_final = False
		self.possessed = False
		self.vel_ts_last = rospy.Time.now()

		path_base = "/uav"+str(uav_id)+"/mavros/"

		self.state = mavros_msgs.msg.State()
		rospy.Subscriber(path_base + "state", mavros_msgs.msg.State, self.stateCb)
		rospy.Subscriber(path_base + "home_position/home", mavros_msgs.msg.HomePosition, self.homeCb)
		self.gpsPos = sensor_msgs.msg.NavSatFix()
		rospy.Subscriber(path_base + "global_position/global", sensor_msgs.msg.NavSatFix, self.globalCb)
		rospy.Subscriber("/admin/cmd", std_msgs.msg.String, self.adminCb)
		rospy.Subscriber("/admin/land_idx", std_msgs.msg.String, self.landIndexCb)
		rospy.Subscriber(path_base + "mission/reached", mavros_msgs.msg.WaypointReached, self.wpCb)
		rospy.Subscriber(path_base +  "setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, self.velCb)

		rospy.wait_for_service(path_base + "cmd/land")
		rospy.wait_for_service(path_base + "cmd/takeoff")
		rospy.wait_for_service(path_base + "cmd/arming")
		rospy.wait_for_service(path_base + "set_mode")
		rospy.wait_for_service(path_base + "param/set")
		self.landService	   = rospy.ServiceProxy(path_base + "cmd/land",	     mavros_msgs.srv.CommandTOL)
		self.takeoffService    = rospy.ServiceProxy(path_base + "cmd/takeoff",   mavros_msgs.srv.CommandTOL)
		self.armService		   = rospy.ServiceProxy(path_base + "cmd/arming",    mavros_msgs.srv.CommandBool)
		self.homeService	   = rospy.ServiceProxy(path_base + "cmd/set_home",  mavros_msgs.srv.CommandHome)
		self.flightModeService = rospy.ServiceProxy(path_base + "set_mode",	     mavros_msgs.srv.SetMode)
		self.paramSetService   = rospy.ServiceProxy(path_base + "param/set",     mavros_msgs.srv.ParamSet)
		self.wpClearService    = rospy.ServiceProxy(path_base + "mission/clear", mavros_msgs.srv.WaypointClear)
		self.wpSetService      = rospy.ServiceProxy(path_base + "mission/push",  mavros_msgs.srv.WaypointPush)

		self.pub_alive      = rospy.Publisher("/alive", std_msgs.msg.String, queue_size=10)
		self.pub_admin_res  = rospy.Publisher("/admin/result", std_msgs.msg.String, queue_size=10)
		self.seqId = 0
		r = rospy.Rate(5)

		while not self.home_set:
			r.sleep()

		while self.state.system_status != 3:
			r.sleep() # wait for drone to be in MAV_STATE_STANDBY or MAV_STATE_ACTIVE

		self.ready = True
		print("ready")

	def stateCb(self, stateMsg):
		self.state = stateMsg

	def homeCb(self, homeMsg):
		if not self.home_set:
			result = self.homeService(current_gps=False, yaw = 90, latitude = cage_origin[0], longitude = cage_origin[1], altitude = cage_origin[2])
			if result.success:
				self.home_set = True


	def globalCb(self, gpsMsg):
		self.gpsPos = gpsMsg

	def velCb(self, velMsg):
		vel_ts_now = rospy.Time.now()
		self.possessed = vel_ts_now - self.vel_ts_last <= rospy.Duration(1)
		self.vel_ts_last = vel_ts_now


	def wpCb(self, wpMsg):
		if self.TOL == "takeoff":
			if self.TOL_state == "prep":
				self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" TAKEOFF PREP"))
				rospy.loginfo("Takeoff prep")
			elif self.TOL_state == "move":
				self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" TAKEOFF MOVE"))
				rospy.loginfo("Takeoff half")
			elif self.TOL_state == "finish":
				# set slow parameters for gameplay
				self.setParam("MPC_TKO_SPEED", 1)
				self.setParam("MPC_ACC_HOR", 0.01) 		# horizontal acceleration for jerk limited trajectory mode
				self.setParam("MPC_ACC_HOR_MAX", 0.01) 	# horizontal acceleration for line tracking mode
				self.setParam("MPC_XY_VEL_MAX", 5.0) 	# max horizontal velocity
				self.setParam("MPC_Z_VEL_MAX_DN", 1.0)	# max descend vel
				self.setParam("MPC_Z_VEL_MAX_UP", 1.0)	# max ascend vel
				self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" TAKEOFF FINISH"))
				rospy.loginfo("Takeoff complete")
				self.TOL = None
				self.TOL_state = None

				while self.state.mode != "OFFBOARD":
				    self.setMode("OFFBOARD")
				    rospy.sleep(0.5)

		elif self.TOL == "landing":
			if self.TOL_state == "prep":
				self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" LANDING PREP"))
				rospy.loginfo("Landing prep")
			elif self.TOL_state == "move":
				rospy.loginfo("Landing half")
				self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" LANDING MOVE"))
			elif self.TOL_state == "finish":
				if self.land_final:
					rospy.loginfo("Landing complete")
					self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" LANDING FINISH"))
					self.TOL = None
					self.TOL_state = None
					self.land_final = False
				else:
					self.land_final = True


	def landIndexCb(self, indexMsg):
		splits = indexMsg.data.split()
		if splits[0] == "/uav"+str(self.uav_id):
			self.land_index = int(splits[1])
			rospy.loginfo("landing index: "+str(self.land_index))


	def adminCb(self, cmdMsg):
                print("Admin message received!")

                ## ToDo drone doesn't know it's possessed because it doesn't receive vel cmd because game doesn't allow control on ground
		#if not self.possessed:
		#	return

		splits = cmdMsg.data.split()
		if splits[0]=="TAKEOFF":
			if splits[1]=="PREP":
				self.TOL = "takeoff"
				rospy.loginfo("Arming")
				while not self.state.armed:
					self.setArmed(True)
					rospy.sleep(1)
				rospy.loginfo("Armed")

				# Set fast parameters
				#self.setParam("MAV_SYS_ID", float(self.uav_id+1))
				self.setParam("MIS_TAKEOFF_ALT", alt_min)
				self.setParam("MIS_LTRMIN_ALT", alt_min)
				self.setParam("MPC_TKO_SPEED", 10)
				self.setParam("MPC_ACC_HOR", 1) 		# horizontal acceleration for jerk limited trajectory mode
				self.setParam("MPC_ACC_HOR_MAX", 1) 	# horizontal acceleration for line tracking mode
				self.setParam("MPC_XY_VEL_MAX", 50.0) 	# max horizontal velocity
				self.setParam("MPC_Z_VEL_MAX_DN", 10)	# max descend vel
				self.setParam("MPC_Z_VEL_MAX_UP", 10)	# max ascend vel

				self.setMode("AUTO.TAKEOFF")
				# wait for takeoff to begin
				while self.state.mode != "AUTO.TAKEOFF":
					rospy.sleep(0.5)
				# wait for takeoff to end
				while self.state.mode == "AUTO.TAKEOFF":
					rospy.sleep(0.5)

				# send takeoff mission
				self.wpClearService()
				self.TOL_state = "prep"
				self.wpSetService(start_index=0, waypoints = [
					mavros_msgs.msg.Waypoint(frame=0, command=16, is_current=True, autocontinue=True,
											param1=0,		# hold time
											param2=2,		# acceptance radius
											param3=0, 		# pass radius
											param4=0, 		# yaw
											x_lat=self.gpsPos.latitude,  	# latitude
											y_long=self.gpsPos.longitude, 	# longitude
											z_alt=alt_ground+alt_min+alt_layer_inc*(self.uav_id+1))				# altitude
					])
				while self.state.mode != "AUTO.MISSION":
					self.setMode("AUTO.MISSION")
					rospy.sleep(0.5)

			elif splits[1]=="MOVE": 
					# move to game start position, sorted altitude
					self.wpClearService()
					self.TOL_state = "move"
					self.wpSetService(start_index=0, waypoints = [
						mavros_msgs.msg.Waypoint(frame=0, command=16, is_current=True, autocontinue=True,
												param1=0,		# hold time
												param2=1,		# acceptance radius
												param3=0, 		# pass radius
												param4=0, 		# yaw
												x_lat=starting_positions[self.uav_id][0],	# latitude
												y_long=starting_positions[self.uav_id][1], 	# longitude
												z_alt=alt_ground+alt_min+alt_layer_inc*(self.uav_id+1))				# altitude
						])



			elif splits[1]=="FINISH":
				# move to start position at standard altitude
				self.wpClearService()
				self.TOL_state = "finish"
				self.wpSetService(start_index=0, waypoints = [
					mavros_msgs.msg.Waypoint(frame=0, command=16, is_current=True, autocontinue=True,
											param1=0,		# hold time
											param2=1,		# acceptance radius
											param3=0, 		# pass radius
											param4=0, 		# yaw
											x_lat=starting_positions[self.uav_id][0],	# latitude
											y_long=starting_positions[self.uav_id][1], 	# longitude
											z_alt=alt_ground+alt_standard)				# altitude
					])


		elif splits[0]=="LANDING":
			if splits[1]=="PREP":
				self.TOL = "landing"

				# set fast parameters
				self.setParam("MPC_ACC_HOR", 1) 		# horizontal acceleration for jerk limited trajectory mode
				self.setParam("MPC_ACC_HOR_MAX", 1) 	# horizontal acceleration for line tracking mode
				self.setParam("MPC_XY_VEL_MAX", 50.0) 	# max horizontal velocity
				self.setParam("MPC_Z_VEL_MAX_DN", 10)	# max descend vel
				self.setParam("MPC_Z_VEL_MAX_UP", 10)	# max ascend vel

				# ascend to sorted altitude
				self.alt_sorted = alt_ground+alt_max-alt_layer_inc*self.land_index
				rospy.loginfo("alt_sorted = "+str(self.alt_sorted))
				if self.alt_sorted < alt_ground + alt_min:
					rospy.loginfo("commanded landing altitude too low!!")
					rospy.logfatal("commanded landing altitude too low!!")
					sys.exit(-1)

				self.wpClearService()
				self.TOL_state = "prep"
				self.wpSetService(start_index=0, waypoints = [
					mavros_msgs.msg.Waypoint(frame=0, command=16, is_current=True, autocontinue=True,
											param1=0,		# hold time
											param2=1,		# acceptance radius
											param3=0, 		# pass radius
											param4=0, 		# yaw
											x_lat=self.gpsPos.latitude,  	# latitude
											y_long=self.gpsPos.longitude, 	# longitude
											z_alt=self.alt_sorted) 				# altitude
											])
				while self.state.mode != "AUTO.MISSION":
					self.setMode("AUTO.MISSION")
					rospy.sleep(0.5)


			elif splits[1]=="MOVE":
				# move and hold above landing position
				self.wpClearService()
				self.TOL_state = "move"
				self.wpSetService(start_index=0, waypoints = [
					mavros_msgs.msg.Waypoint(frame=0, command=16, is_current=True, autocontinue=True,
											param1=0,		# hold time
											param2=1,		# acceptance radius
											param3=0, 		# pass radius
											param4=0, 		# yaw
											x_lat=landing_positions[self.uav_id][0], 	# latitude
											y_long=landing_positions[self.uav_id][1], 	# longitude
											z_alt=self.alt_sorted) 		  				# altitude
					])


			elif splits[1]=="FINISH":
				# set slow parameters for landing
				self.setParam("MPC_TKO_SPEED", 1)
				self.setParam("MPC_Z_VEL_MAX_DN", 1.0)	# max descend vel
				# land
				self.wpClearService()
				self.TOL_state = "finish"
				## Waypoint mission doesn't work if first waypoint is to land
				self.wpSetService(start_index=0, waypoints = [
					mavros_msgs.msg.Waypoint(frame=0, command=16, is_current=True, autocontinue=True,
											param1=0,		# hold time
											param2=1,		# acceptance radius
											param3=0, 		# pass radius
											param4=0, 		# yaw
											x_lat=landing_positions[self.uav_id][0], 	# latitude
											y_long=landing_positions[self.uav_id][1], 	# longitude
											z_alt=alt_ground+alt_min), 		  				# altitude
					mavros_msgs.msg.Waypoint(frame=0, command=21, is_current=False, autocontinue=True,
											param1=alt_ground-5, 	# abort alt
											param2=0, 				# no precision landing
											param3=0, 				# NA
											param4=0, 				# yaw
											x_lat=landing_positions[self.uav_id][0], # latitude
											y_long=landing_positions[self.uav_id][1], # longitude
											z_alt=alt_ground, 						  # altitude
											)])




	def setMode(self, mode = "OFFBOARD"):
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

	def shutdownCb(self):
		sys.exit(0)

if __name__ == "__main__":
	try:
		rospy.init_node("uav_control", anonymous=True)
		uavID = int(rospy.get_namespace()[-2])
		uav = UAV(uavID)
		rospy.on_shutdown(uav.shutdownCb)

		while not uav.ready:
			rospy.sleep(1)
		rospy.loginfo("UAV starting")

		rospy.loginfo("Waiting for home position estimate")
		while not uav.home_set:
			rospy.sleep(1)

		rospy.loginfo("ready to fly")

		while not rospy.is_shutdown():
			uav.pub_alive.publish("/uav"+str(uav.uav_id))
			rospy.sleep(0.5)
			# set offboard mode

	except rospy.exceptions.ROSInterruptException:
		sys.exit(0)
