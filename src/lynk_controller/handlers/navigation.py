import rospy
from mavros_msgs.msg import GlobalPositionTarget
from lynk_controller.handlers.base import BaseCommandHandler
from lynk_controller.interfaces import RESULT_SUCCESS, RESULT_FAILURE
from lynk_nexus.msg import Command

class NavigationHandler(BaseCommandHandler):
    """Handles navigation commands like GOTO and SMART_GOTO."""

    def handle_smart_goto(self, msg: Command):
        try:
            params = self._extract_params(msg)
            try:
                lat_raw = params.get("lat", params.get("latitude"))
                lon_raw = params.get("lon", params.get("longitude"))
                alt_raw = params.get("alt", params.get("altitude_m"))
                
                if lat_raw is None or lon_raw is None:
                    raise ValueError("Missing required coordinates (lat/lon)")
                
                lat = float(lat_raw)
                lon = float(lon_raw)
                alt = float(alt_raw) if alt_raw is not None else 10.0
                
                if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                    raise ValueError(f"Invalid GPS coordinates: {lat}, {lon}")
                if alt < 0 or alt > 2000:
                    raise ValueError(f"Invalid altitude: {alt}m")
            except (ValueError, TypeError) as e:
                message = f"SMART_GOTO Fail: Invalid parameters - {str(e)}"
                rospy.logerr(f"[NavigationHandler] {message}")
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=message)
                return
            
            rospy.loginfo(f"[NavigationHandler] SMART_GOTO | TARGET: {lat}, {lon} @ {alt}m")

            if not self._ensure_guided(msg, "SMART_GOTO"): return
            if not self._ensure_armed(msg, "SMART_GOTO"): return
            if not self._ensure_takeoff(msg, alt, "SMART_GOTO"): return

            target = GlobalPositionTarget()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = "base_link"
            target.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_REL_ALT
            target.type_mask = (
                GlobalPositionTarget.IGNORE_VX |
                GlobalPositionTarget.IGNORE_VY |
                GlobalPositionTarget.IGNORE_VZ |
                GlobalPositionTarget.IGNORE_AFX |
                GlobalPositionTarget.IGNORE_AFY |
                GlobalPositionTarget.IGNORE_AFZ |
                GlobalPositionTarget.IGNORE_YAW |
                GlobalPositionTarget.IGNORE_YAW_RATE
            )
            target.latitude = lat
            target.longitude = lon
            target.altitude = alt
            
            self.mavros.publish_goto(target)
            rospy.loginfo(f"[NavigationHandler] SMART_GOTO | Published to {lat}, {lon} @ {alt}m")
            self.lynk.send_result(msg, status=RESULT_SUCCESS, message=f"Smart GoTo initiated to {lat}, {lon}")
        except Exception as e:
            rospy.logerr(f"[NavigationHandler] Error during SMART_GOTO handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))
