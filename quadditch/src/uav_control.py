#!/usr/bin/env python

import rospy
import mavros_msgs.msg
import std_msgs.msg
import mavros_msgs.srv
import geometry_msgs.msg
import sensor_msgs.msg
import quadditch.srv
import sys


# command enumerations here and in mavros_msgs/CommandCode
# https://mavlink.io/en/messages/common.html#MAV_CMD


NUM_DRONES = 4
GROUND_ALTITUDE = 587 + 32.8 # need to add offset for some reason
ABORT_ALTITUDE = GROUND_ALTITUDE - 5
STANDARD_HEIGHT_ABOVE_GROUND = 8
MIN_HEIGHT_ABOVE_GROUND = 5
MAX_HEIGHT_ABOVE_GROUND = MIN_HEIGHT_ABOVE_GROUND + ALTITUDE_LAYER_INCREMENT * NUM_DRONES

ALTITUDE_LAYER_INCREMENT = 1.5

CAGE_ORIGIN = (37.22305889702098, -80.43259539019778, GROUND_ALTITUDE)
LANDING_POSITIONS = [
    (37.22309079590038, -80.4328058738124),
    (37.22316234536629, -80.4326959032434),
    (37.22302245007853, -80.4324853498369),
    (37.22295090047989, -80.4325282651809)
]
STARTING_POSITIONS = LANDING_POSITIONS


class UAV:

    def __init__(self, uav_id):

        self.ready = False
        self.home_set = False
        self.uav_id = uav_id
        self.takeoff_or_landing = None # takeoff or landing mode
        self.takeoff_or_landing_state = None # prep, move, finish
        self.cmd_last = ""
        self.land_index = 0 # to be set by game master at landing time
        self.alt_sorted = STANDARD_HEIGHT_ABOVE_GROUND
        self.land_final = False
        self.possessed = False
        self.vel_ts_last = rospy.Time.now()

        uav_ns = "/uav" + str(uav_id)
        mavros_ns = uav_ns + "/mavros/"

        self.homeService = None

        self.state = mavros_msgs.msg.State()
        self.gpsPos = sensor_msgs.msg.NavSatFix()

        rospy.Subscriber("/admin/land_idx", std_msgs.msg.String, self.on_land_index_cb)
        rospy.Subscriber(mavros_ns + "state", mavros_msgs.msg.State, self.on_state_cb)
        rospy.Subscriber(mavros_ns + "home_position/home", mavros_msgs.msg.HomePosition, self.on_home_pos_cb)
        rospy.Subscriber(mavros_ns + "global_position/global", sensor_msgs.msg.NavSatFix, self.on_global_pos_cb)
        rospy.Subscriber(mavros_ns + "mission/reached", mavros_msgs.msg.WaypointReached, self.on_waypoint_update_cb)
        rospy.Subscriber(mavros_ns +  "setpoint_velocity/cmd_vel_unstamped", geometry_msgs.msg.Twist, self.on_velocity_cb)

        rospy.wait_for_service(mavros_ns + "cmd/land")
        rospy.wait_for_service(mavros_ns + "cmd/takeoff")
        rospy.wait_for_service(mavros_ns + "cmd/arming")
        rospy.wait_for_service(mavros_ns + "set_mode")
        rospy.wait_for_service(mavros_ns + "param/set")

        self.landService       = rospy.ServiceProxy(mavros_ns + "cmd/land",      mavros_msgs.srv.CommandTOL)
        self.takeoffService    = rospy.ServiceProxy(mavros_ns + "cmd/takeoff",   mavros_msgs.srv.CommandTOL)
        self.armService        = rospy.ServiceProxy(mavros_ns + "cmd/arming",    mavros_msgs.srv.CommandBool)
        self.homeService       = rospy.ServiceProxy(mavros_ns + "cmd/set_home",  mavros_msgs.srv.CommandHome)
        self.flightModeService = rospy.ServiceProxy(mavros_ns + "set_mode",      mavros_msgs.srv.SetMode)
        self.paramSetService   = rospy.ServiceProxy(mavros_ns + "param/set",     mavros_msgs.srv.ParamSet)
        self.wpClearService    = rospy.ServiceProxy(mavros_ns + "mission/clear", mavros_msgs.srv.WaypointClear)
        self.wpSetService      = rospy.ServiceProxy(mavros_ns + "mission/push",  mavros_msgs.srv.WaypointPush)

        self.adminCmdService = rospy.Service(uav_ns + "/adminService", quadditch.srv.AdminCmd, self.on_admin_cmd_srv_cb)

        self.pub_alive     = rospy.Publisher("/alive", std_msgs.msg.String, queue_size=10)
        self.pub_admin_res = rospy.Publisher("/admin/result", std_msgs.msg.String, queue_size=10)
        self.seqId = 0
        r = rospy.Rate(5)

        # while not self.home_set and not rospy.is_shutdown():
        #     r.sleep()

        while self.state.system_status != 3 and not rospy.is_shutdown():
            r.sleep() # wait for drone to be in MAV_STATE_STANDBY or MAV_STATE_ACTIVE

        self.ready = True
        rospy.loginfo("Ready.")


    def on_state_cb(self, stateMsg):
        self.state = stateMsg


    def on_home_pos_cb(self, homeMsg):
        rospy.loginfo("Setting home: {}".format(homeMsg))
        if self.homeService is None:
            return
        if not self.home_set:
            result = self.homeService(
                current_gps=False,
                yaw=90,
                latitude=CAGE_ORIGIN[0],
                longitude=CAGE_ORIGIN[1],
                altitude=CAGE_ORIGIN[2])
            if result.success:
                self.home_set = True


    def on_global_pos_cb(self, gpsMsg):
        self.gpsPos = gpsMsg


    def on_velocity_cb(self, velMsg):

        old_possessed = self.possessed

        vel_ts_now = rospy.Time.now()
        # this is weird
        self.possessed = vel_ts_now - self.vel_ts_last <= rospy.Duration(1)
        self.vel_ts_last = vel_ts_now

        if self.possessed is not old_possessed:
            rospy.loginfo("Possession has changed: {}".format(self.possessed))


    def on_waypoint_update_cb(self, wpMsg):

        # alias for publishing a name-qualified landing status
        # message to /admin/result
        def publish_landing_state_msg(text):
            msg_text = "/uav{} {}".format(self.uav_id, text)
            self.pub_admin_res.publish(std_msgs.msg.String(msg_text))


        if self.takeoff_or_landing == "takeoff":

            if self.takeoff_or_landing_state == "prep":
                self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" TAKEOFF PREP"))
                rospy.loginfo("Takeoff prep")

            elif self.takeoff_or_landing_state == "move":
                self.pub_admin_res.publish(std_msgs.msg.String("/uav"+str(self.uav_id)+" TAKEOFF MOVE"))
                rospy.loginfo("Takeoff half")

            elif self.takeoff_or_landing_state == "finish":
                # set slow parameters for gameplay
                self.setParam("MPC_TKO_SPEED", 1)
                self.setParam("MPC_ACC_HOR", 0.01)         # horizontal acceleration for jerk limited trajectory mode
                self.setParam("MPC_ACC_HOR_MAX", 0.01)     # horizontal acceleration for line tracking mode
                self.setParam("MPC_XY_VEL_MAX", 5.0)     # max horizontal velocity
                self.setParam("MPC_Z_VEL_MAX_DN", 1.0)    # max descend vel
                self.setParam("MPC_Z_VEL_MAX_UP", 1.0)    # max ascend vel

                publish_landing_state_msg("TAKEOFF FINISH")
                rospy.loginfo("Takeoff complete")

                self.takeoff_or_landing = None
                self.takeoff_or_landing_state = None

                while self.state.mode != "OFFBOARD":
                    self.setMode("OFFBOARD")
                    rospy.sleep(0.5)

        elif self.takeoff_or_landing == "land":

            if self.takeoff_or_landing_state == "prep":
                publish_landing_state_msg("LANDING PREP")
                rospy.loginfo("Landing prep")

            elif self.takeoff_or_landing_state == "move":
                publish_landing_state_msg("LANDING MOVE")
                rospy.loginfo("Landing half")

            elif self.takeoff_or_landing_state == "finish":
                if self.land_final:
                    rospy.loginfo("Landing complete")
                    publish_landing_state_msg("LANDING FINISH")
                    self.takeoff_or_landing = None
                    self.takeoff_or_landing_state = None
                    self.land_final = False
                else:
                    self.land_final = True


    def on_land_index_cb(self, indexMsg):
        rospy.loginfo("Got a landing index message: {}".format(indexMsg))
        splits = indexMsg.data.split()
        if splits[0] == "/uav" + str(self.uav_id):
            self.land_index = int(splits[1])
            rospy.loginfo("landing index: " + str(self.land_index))


    def on_admin_cmd_srv_cb(self, req):

        rospy.loginfo("UAV{}: Got admin command request: {}".format(self.uav_id, req))

        cmd = req.command + req.intermediate

        if not self.possessed:
            rospy.loginfo("UAV{}: not possessed. Ignoring command.".format(self.uav_id))
            return quadditch.srv.AdminCmdResponse(True)

        if cmd == self.cmd_last:
            rospy.loginfo("UAV{}: duplicate command received. Ignoring.".format(self.uav_id))
            return quadditch.srv.AdminCmdResponse(True)

        self.cmd_last = cmd # storing this to ignore subsequent duplicate commands

        if req.command == "TAKEOFF":
            if req.intermediate == "PREP":
                self.takeoff_or_landing = "takeoff"
                rospy.loginfo("Arming")
                while not self.state.armed:
                    self.setArmed(True)
                    rospy.sleep(1)
                rospy.loginfo("Armed")

                # Set fast parameters
                # self.setParam("MAV_SYS_ID", float(self.uav_id+1)) # TODO: this might be useful
                self.setParam("MIS_TAKEOFF_ALT", MIN_HEIGHT_ABOVE_GROUND)
                self.setParam("MIS_LTRMIN_ALT", MIN_HEIGHT_ABOVE_GROUND)
                self.setParam("MPC_TKO_SPEED", 10)
                self.setParam("MPC_ACC_HOR", 1)       # horizontal acceleration for jerk limited trajectory mode
                self.setParam("MPC_ACC_HOR_MAX", 1)   # horizontal acceleration for line tracking mode
                self.setParam("MPC_XY_VEL_MAX", 50.0) # max horizontal velocity
                self.setParam("MPC_Z_VEL_MAX_DN", 10) # max descend vel
                self.setParam("MPC_Z_VEL_MAX_UP", 10) # max ascend vel

                self.setMode("AUTO.TAKEOFF")

                rospy.loginfo("Waiting for takeoff to begin...")
                while self.state.mode != "AUTO.TAKEOFF":
                    rospy.loginfo("Awaiting takeoff.")
                    rospy.sleep(0.5)

                rospy.loginfo("Waiting for takeoff to end...")
                while self.state.mode == "AUTO.TAKEOFF":
                    rospy.loginfo("Takeoff is ongoing; awaiting end.")
                    rospy.sleep(0.5)

                # send takeoff mission
                self.wpClearService()
                self.takeoff_or_landing_state = "prep"

                desired_altitude = GROUND_ALTITUDE + MIN_HEIGHT_ABOVE_GROUND + \
                    ALTITUDE_LAYER_INCREMENT * (self.uav_id + 1)

                self.wpSetService(start_index=0, waypoints = [
                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                    mavros_msgs.msg.Waypoint(frame=0, command=16,
                        is_current=True, autocontinue=True,
                        param1=0, # hold time
                        param2=2, # acceptance radius
                        param3=0, # pass radius
                        param4=0, # yaw
                        x_lat=self.gpsPos.latitude,
                        y_long=self.gpsPos.longitude,
                        z_alt=desired_altitude)
                    ])
                while self.state.mode != "AUTO.MISSION":
                    self.setMode("AUTO.MISSION")
                    rospy.sleep(0.5)

            elif req.intermediate=="MOVE":

                desired_altitude = GROUND_ALTITUDE + MIN_HEIGHT_ABOVE_GROUND + \
                    ALTITUDE_LAYER_INCREMENT * (self.uav_id + 1)

                # move to game start position, sorted altitude
                self.wpClearService()
                self.takeoff_or_landing_state = "move"
                self.wpSetService(start_index=0, waypoints = [
                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                    mavros_msgs.msg.Waypoint(frame=0, command=16,
                        is_current=True, autocontinue=True,
                        param1=0, # hold time
                        param2=1, # acceptance radius
                        param3=0, # pass radius
                        param4=0, # yaw
                        x_lat=STARTING_POSITIONS[self.uav_id][0],
                        y_long=STARTING_POSITIONS[self.uav_id][1],
                        z_alt=desired_altitude)
                    ])



            elif req.intermediate == "FINISH":
                # move to start position at standard altitude
                self.wpClearService()
                self.takeoff_or_landing_state = "finish"
                self.wpSetService(start_index=0, waypoints = [
                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                    mavros_msgs.msg.Waypoint(frame=0, command=16,
                        is_current=True, autocontinue=True,
                        param1=0, # hold time
                        param2=1, # acceptance radius
                        param3=0, # pass radius
                        param4=0, # yaw
                        x_lat=STARTING_POSITIONS[self.uav_id][0],
                        y_long=STARTING_POSITIONS[self.uav_id][1],
                        z_alt=GROUND_ALTITUDE + STANDARD_HEIGHT_ABOVE_GROUND)
                    ])


        elif req.command == "LAND":
            if req.intermediate == "PREP":
                self.takeoff_or_landing = "land"

                # set fast parameters
                self.setParam("MPC_ACC_HOR", 1)         # horizontal acceleration for jerk limited trajectory mode
                self.setParam("MPC_ACC_HOR_MAX", 1)     # horizontal acceleration for line tracking mode
                self.setParam("MPC_XY_VEL_MAX", 50.0)     # max horizontal velocity
                self.setParam("MPC_Z_VEL_MAX_DN", 10)    # max descend vel
                self.setParam("MPC_Z_VEL_MAX_UP", 10)    # max ascend vel

                # ascend to sorted altitude
                self.alt_sorted = GROUND_ALTITUDE + MAX_HEIGHT_ABOVE_GROUND - \
                    ALTITUDE_LAYER_INCREMENT*self.land_index
                rospy.loginfo("alt_sorted = {}".format(self.alt_sorted))
                if self.alt_sorted < GROUND_ALTITUDE + MIN_HEIGHT_ABOVE_GROUND:
                    rospy.loginfo("commanded landing altitude too low!!")
                    rospy.logfatal("commanded landing altitude too low!!")
                    sys.exit(-1)

                self.wpClearService()
                self.takeoff_or_landing_state = "prep"
                self.wpSetService(start_index=0, waypoints = [
                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                    mavros_msgs.msg.Waypoint(frame=0, command=16,
                        is_current=True, autocontinue=True,
                        param1=0, # hold time
                        param2=1, # acceptance radius
                        param3=0, # pass radius
                        param4=0, # yaw
                        x_lat=self.gpsPos.latitude,      # latitude
                        y_long=self.gpsPos.longitude,     # longitude
                        z_alt=self.alt_sorted)                 # altitude
                ])
                while self.state.mode != "AUTO.MISSION":
                    self.setMode("AUTO.MISSION")
                    rospy.sleep(0.5)


            elif req.intermediate == "MOVE":
                # move and hold above landing position
                self.wpClearService()
                self.takeoff_or_landing_state = "move"
                self.wpSetService(start_index=0, waypoints = [
                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                    mavros_msgs.msg.Waypoint(frame=0, command=16,
                        is_current=True, autocontinue=True,
                        param1=0, # hold time
                        param2=1, # acceptance radius
                        param3=0, # pass radius
                        param4=0, # yaw
                        x_lat=LANDING_POSITIONS[self.uav_id][0],
                        y_long=LANDING_POSITIONS[self.uav_id][1],
                        z_alt=self.alt_sorted)
                    ])


            elif req.intermediate == "FINISH":
                # set slow parameters for landing
                self.setParam("MPC_TKO_SPEED", 1)
                self.setParam("MPC_Z_VEL_MAX_DN", 1.0)    # max descend vel
                # land
                self.wpClearService()
                self.takeoff_or_landing_state = "finish"
                ## Waypoint mission doesn't work if first waypoint is to land
                self.wpSetService(start_index=0, waypoints = [

                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_WAYPOINT
                    mavros_msgs.msg.Waypoint(frame=0, command=16,
                        is_current=True, autocontinue=True,
                        param1=0, # hold time
                        param2=1, # acceptance radius
                        param3=0, # pass radius
                        param4=0, # yaw
                        x_lat=LANDING_POSITIONS[self.uav_id][0],
                        y_long=LANDING_POSITIONS[self.uav_id][1],
                        z_alt=GROUND_ALTITUDE + MIN_HEIGHT_ABOVE_GROUND),

                    # https://mavlink.io/en/messages/common.html#MAV_CMD_NAV_LAND
                    mavros_msgs.msg.Waypoint(frame=0, command=21, is_current=False, autocontinue=True,
                        param1=ABORT_ALTITUDE,
                        param2=0, # no precision landing
                        param3=0, # NA
                        param4=0, # yaw
                        x_lat=LANDING_POSITIONS[self.uav_id][0],
                        y_long=LANDING_POSITIONS[self.uav_id][1],
                        z_alt=GROUND_ALTITUDE,
                    )])

        return quadditch.srv.AdminCmdResponse(True)




    def setMode(self, mode):
        STARTING_POSITIONS("Setting flight mode to {}".format(mode))
        try:
            self.flightModeService(custom_mode=mode)
        except rospy.ServiceException as e:
            rospy.logerr("service set_mode call failed: %s"%e)


    def setParam(self, paramId, paramValue):
        rospy.loginfo("Setting param {} to {}".format(paramID, paramValue))
        try:
            self.paramSetService(param_id = paramId, value = mavros_msgs.msg.ParamValue(integer=0, real=paramValue))
        except rospy.ServiceException as e:
            rospy.logerr("service set param call failed: %s"%e)


    def setArmed(self, armed = True):
        rospy.loginfo("Setting armed state to {}".format(armed))
        try:
            self.armService(True)
        except rospy.ServiceException as e:
            rospy.logerr("Service arm call failed: %s"%e)


    def on_shutdown_cb(self):
        sys.exit(0)


if __name__ == "__main__":

    rospy.init_node("uav_control", anonymous=True)
    uavID = int(rospy.get_namespace()[-2])
    uav = UAV(uavID)
    rospy.on_shutdown(uav.on_shutdown_cb)

    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    rospy.logwarn("NOT WAITING FOR READY STATE. This may have negative consequences later!!! - Wade")
    # while not uav.ready:
    #     rospy.sleep(1)
    # rospy.loginfo("UAV starting")

    # rospy.loginfo("Waiting for home position estimate")
    # while not uav.home_set:
    #     rospy.sleep(1)

    rospy.loginfo("Ready to fly.")

    while not rospy.is_shutdown():
        uav.pub_alive.publish("/uav"+str(uav.uav_id)) # TODO: move this to a UAV timer
        rospy.sleep(0.5)
        # set offboard mode

