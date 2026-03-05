import json
import rospy
from typing import Dict, Any
from lynk_controller.interfaces import MavrosInterface, LynkInterface, RESULT_SUCCESS, RESULT_FAILURE
from lynk_nexus.msg import Command

class BaseCommandHandler:
    """Base class for all command handlers."""
    def __init__(self, mavros: MavrosInterface, lynk: LynkInterface):
        self.mavros = mavros
        self.lynk = lynk

    def _extract_params(self, msg: Command) -> Dict[str, Any]:
        """Merges parsed_params_json and params_json into a single dictionary."""
        params = {}
        if msg.parsed_params_json:
            try:
                params.update(json.loads(msg.parsed_params_json))
            except Exception:
                pass
        if msg.params_json:
            try:
                data = json.loads(msg.params_json)
                if isinstance(data, dict):
                    if "json" in data and isinstance(data["json"], str):
                        try:
                            nested = json.loads(data["json"])
                            if isinstance(nested, dict):
                                params.update(nested)
                        except:
                            pass
                    if "params" in data and isinstance(data["params"], dict):
                        params.update(data["params"])
                    else:
                        params.update(data)
            except Exception:
                pass
        return params

    def _ensure_guided(self, msg: Command, cmd_context: str) -> bool:
        if self.mavros.mode != "GUIDED":
            rospy.loginfo(f"[CommandHandler] {cmd_context} | Setting mode to GUIDED (Current: {self.mavros.mode})")
            resp = self.mavros.set_mode(mode="GUIDED")
            if not resp.mode_sent:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=f"{cmd_context} Fail: Could not set GUIDED mode")
                return False
            rospy.sleep(1.0)
        return True

    def _ensure_armed(self, msg: Command, cmd_context: str) -> bool:
        retry_count = 0
        while not self.mavros.armed and retry_count < 3:
            rospy.loginfo(f"[CommandHandler] {cmd_context} | Arming vehicle (Attempt {retry_count + 1})")
            self.mavros.arm(value=True)
            rospy.sleep(2.0)
            if self.mavros.armed:
                break
            retry_count += 1

        if not self.mavros.armed:
            message = f"{cmd_context} Fail: Vehicle failed to arm after retries"
            rospy.logerr(f"[CommandHandler] {message}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=message)
            return False
        return True

    def _ensure_takeoff(self, msg: Command, altitude: float, cmd_context: str) -> bool:
        if self.mavros.altitude < 1.0:
            rospy.loginfo(f"[CommandHandler] {cmd_context} | Vehicle on ground. Taking off to {altitude}m first")
            resp = self.mavros.takeoff(altitude=altitude)
            if not resp.success:
                message = f"{cmd_context} Prep Fail: MAVROS Takeoff rejected"
                rospy.logerr(f"[CommandHandler] {message}")
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=message)
                return False
            
            timeout = rospy.Time.now() + rospy.Duration(30.0)
            while self.mavros.altitude < (altitude * 0.8) and not rospy.is_shutdown():
                if rospy.Time.now() > timeout:
                    self.lynk.send_result(msg, status=RESULT_FAILURE, message=f"{cmd_context} Prep Fail: Takeoff timeout")
                    return False
                rospy.sleep(0.5)
            rospy.loginfo(f"[CommandHandler] {cmd_context} | Takeoff complete")
        return True
