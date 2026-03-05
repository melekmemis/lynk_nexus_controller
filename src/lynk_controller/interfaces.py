import rospy
from typing import Dict, Any
from lynk_nexus.msg import Ack, ResultEnvelope, Command
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointPush, WaypointClear, WaypointSetCurrent, ParamSet
from mavros_msgs.msg import State as MavrosState, GlobalPositionTarget, ParamValue, WaypointList
from sensor_msgs.msg import BatteryState



# LYNK Command Status Constants
RESULT_SUCCESS = 0
RESULT_FAILURE = 1
RESULT_TIMEOUT = 2

class MavrosInterface:
    """Encapsulates MAVROS service calls and state tracking."""
    def __init__(self):
        # State
        self.armed = False
        self.mode = "UNKNOWN"
        self.altitude = 0.0
        self.abs_altitude = 0.0
        self.lat = 0.0
        self.lon = 0.0
        self.current_wp = 0
        self.battery_remaining = 1.0 # 0.0 to 1.0



        # Services
        self._arm_srv = rospy.ServiceProxy("/mavros/cmd/arming", CommandBool)
        self._set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self._takeoff_srv = rospy.ServiceProxy("/mavros/cmd/takeoff", CommandTOL)
        self._land_srv = rospy.ServiceProxy("/mavros/cmd/land", CommandTOL)
        self._mission_clear_srv = rospy.ServiceProxy("/mavros/mission/clear", WaypointClear)
        self._mission_push_srv = rospy.ServiceProxy("/mavros/mission/push", WaypointPush)
        self._mission_set_current_srv = rospy.ServiceProxy("/mavros/mission/set_current", WaypointSetCurrent)
        self._param_set_srv = rospy.ServiceProxy("/mavros/param/set", ParamSet)
        self._wp_sub = rospy.Subscriber("/mavros/mission/waypoints", WaypointList, self._on_mission_wps)
        self._battery_sub = rospy.Subscriber("/mavros/battery", BatteryState, self._on_battery)



        # Publishers
        self._goto_pub = rospy.Publisher("/mavros/setpoint_raw/global", GlobalPositionTarget, queue_size=10)

    def update_state(self, msg: MavrosState):
        self.armed = msg.armed
        self.mode = msg.mode

    def update_gps(self, msg: Any):
        self.altitude = msg.rel_alt_m
        self.abs_altitude = msg.alt_m
        self.lat = msg.lat
        self.lon = msg.lon

    def _on_mission_wps(self, msg: WaypointList):
        for i, wp in enumerate(msg.waypoints):
            if wp.is_current:
                self.current_wp = i
                break

    def _on_battery(self, msg: BatteryState):
        self.battery_remaining = msg.percentage



    def set_mode(self, mode: str):
        return self._set_mode_srv(custom_mode=mode)

    def arm(self, value: bool):
        return self._arm_srv(value=value)

    def takeoff(self, altitude: float):
        return self._takeoff_srv(altitude=altitude)

    def land(self):
        return self._set_mode_srv(custom_mode="LAND")

    def rtl(self):
        return self._set_mode_srv(custom_mode="RTL")

    def mission_clear(self):
        return self._mission_clear_srv()

    def mission_push(self, start_index: int, waypoints: list):
        return self._mission_push_srv(start_index=start_index, waypoints=waypoints)

    def mission_set_current(self, wp_seq: int):
        return self._mission_set_current_srv(wp_seq=wp_seq)

    def param_set(self, param_id: str, value: float):
        """Sets a MAVROS/ArduPilot parameter."""
        p_val = ParamValue()
        # ArduPilot params are usually floats or ints
        # If it's a whole number, we set integer, else real? 
        # Actually ParamValue has both. ArduPilot use real for most.
        p_val.integer = int(value)
        p_val.real = float(value)
        return self._param_set_srv(param_id=param_id, value=p_val)

    def publish_goto(self, target: GlobalPositionTarget):
        self._goto_pub.publish(target)

class LynkInterface:
    """Encapsulates LYNK protocol communications."""
    def __init__(self, vehicle_id: int, namespace: str):
        self.vehicle_id = vehicle_id
        self._ack_pub = rospy.Publisher(f"{namespace}/tx/request/ack", Ack, queue_size=10)
        self._result_pub = rospy.Publisher(f"{namespace}/tx/request/result", ResultEnvelope, queue_size=10)

    def send_ack(self, cmd_msg: Command):
        ack = Ack()
        ack.header.stamp = rospy.Time.now()
        ack.transaction_id = cmd_msg.transaction_id
        ack.command_id = cmd_msg.command_id
        ack.command_name = cmd_msg.command_name
        ack.src_id = self.vehicle_id
        ack.dst_id = cmd_msg.src_id
        ack.ack_id = 1 
        ack.ack_name = "OK"
        self._ack_pub.publish(ack)

    def send_result(self, cmd_msg: Command, status: int, message: str = "", error_code: int = 0):
        res = ResultEnvelope()
        res.tx_id = cmd_msg.transaction_id
        res.result.status = status
        res.result.error_code = error_code
        res.result.message = message
        self._result_pub.publish(res)
