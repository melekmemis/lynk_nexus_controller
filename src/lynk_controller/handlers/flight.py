import rospy
from lynk_controller.handlers.base import BaseCommandHandler
from lynk_controller.interfaces import RESULT_SUCCESS, RESULT_FAILURE
from lynk_nexus.msg import Command

class FlightHandler(BaseCommandHandler):
    """Handles core flight commands."""

    def handle_arming(self, msg: Command):
        try:
            params = self._extract_params(msg)
            arm_req = bool(params.get("arm", False))
            rospy.loginfo(f"[FlightHandler] ARMING | Target: {arm_req}")
            resp = self.mavros.arm(value=arm_req)
            if resp.success:
                self.lynk.send_result(msg, status=RESULT_SUCCESS, message="Arming successful")
            else:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=f"MAVROS Arming failed: {resp.result}")
        except Exception as e:
            rospy.logerr(f"[FlightHandler] Error during arming handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))

    def handle_takeoff(self, msg: Command):
        try:
            params = self._extract_params(msg)
            alt = float(params.get("altitude_m", params.get("alt", 5.0)))
            rospy.loginfo(f"[FlightHandler] TAKEOFF | Resolved Altitude: {alt}m")
            
            if not self._ensure_guided(msg, "TAKEOFF"): return
            if not self._ensure_armed(msg, "TAKEOFF"): return

            rospy.loginfo(f"[FlightHandler] TAKEOFF | Executing takeoff to {alt}m...")
            resp = self.mavros.takeoff(altitude=alt)
            if not resp.success:
                message = "TAKEOFF Fail: MAVROS Takeoff rejected"
                rospy.logerr(f"[FlightHandler] {message}")
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=message)
                return

            rospy.loginfo("[FlightHandler] TAKEOFF | Waiting for altitude...")
            timeout = rospy.Time.now() + rospy.Duration(30.0)
            while self.mavros.altitude < (alt * 0.8) and not rospy.is_shutdown():
                if rospy.Time.now() > timeout:
                    self.lynk.send_result(msg, status=RESULT_FAILURE, message="TAKEOFF Fail: Takeoff timeout")
                    return
                rospy.sleep(0.5)

            rospy.loginfo("[FlightHandler] TAKEOFF | Success")
            self.lynk.send_result(msg, status=RESULT_SUCCESS, message=f"Takeoff successful to {alt}m")
        except Exception as e:
            rospy.logerr(f"[FlightHandler] Error during takeoff handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))

    def handle_set_mode(self, msg: Command):
        try:
            params = self._extract_params(msg)
            mode = str(params.get("mode", "GUIDED")).upper()
            rospy.loginfo(f"[FlightHandler] SET_MODE | Target: {mode}")
            resp = self.mavros.set_mode(mode=mode)
            if resp.mode_sent:
                self.lynk.send_result(msg, status=RESULT_SUCCESS, message=f"Mode set to {mode}")
            else:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=f"Failed to set mode {mode}")
        except Exception as e:
            rospy.logerr(f"[FlightHandler] Error during set_mode handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))

    def handle_land(self, msg: Command):
        try:
            rospy.loginfo("[FlightHandler] Calling MAVROS SetMode: LAND")
            resp = self.mavros.land()
            if resp.mode_sent:
                self.lynk.send_result(msg, status=RESULT_SUCCESS, message="Landing initiated (LAND mode set)")
            else:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message="MAVROS Land mode change failed")
        except Exception as e:
            rospy.logerr(f"[FlightHandler] Error during land handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))

    def handle_rtl(self, msg: Command):
        try:
            rospy.loginfo("[FlightHandler] Calling MAVROS SetMode: RTL")
            resp = self.mavros.rtl()
            if resp.mode_sent:
                self.lynk.send_result(msg, status=RESULT_SUCCESS, message="RTL initiated (RTL mode set)")
            else:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message="MAVROS RTL mode change failed")
        except Exception as e:
            rospy.logerr(f"[FlightHandler] Error during RTL handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))
