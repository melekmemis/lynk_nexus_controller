import json
import math
import rospy
from mavros_msgs.msg import Waypoint
from lynk_controller.handlers.base import BaseCommandHandler
from lynk_controller.interfaces import RESULT_SUCCESS, RESULT_FAILURE
from lynk_nexus.msg import Command

class MissionHandler(BaseCommandHandler):
    """Handles mission upload and control."""

    def handle_upload(self, msg: Command):
        try:
            params = self._extract_params(msg)
            json_str = params.get("json")
            if not json_str:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message="MISSION_UPLOAD Fail: Missing 'json' parameter")
                return

            payload = json.loads(json_str)
            wp_list = payload.get("waypoints", [])
            replace = payload.get("replace_existing", True)
            width_m = payload.get("width_m")

            if width_m is not None:
                # ArduPilot WPNAV_XTRACK_ERR is in cm
                xtrack_error_cm = float(width_m) * 100.0
                rospy.loginfo(f"[MissionHandler] MISSION | Setting WPNAV_XTRACK_ERR to {xtrack_error_cm} cm")
                self.mavros.param_set("WPNAV_XTRACK_ERR", xtrack_error_cm)

            if replace:
                rospy.loginfo("[MissionHandler] MISSION | Clearing existing mission")
                self.mavros.mission_clear()

            mavros_wps = []
            home_wp = Waypoint()
            home_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
            home_wp.command = 16 # MAV_CMD_NAV_WAYPOINT
            home_wp.is_current = False
            home_wp.autocontinue = True
            if self.mavros.lat != 0 and self.mavros.lon != 0:
                home_wp.x_lat = self.mavros.lat
                home_wp.y_long = self.mavros.lon
            elif wp_list:
                home_wp.x_lat = float(wp_list[0].get("lat", 0.0))
                home_wp.y_long = float(wp_list[0].get("lon", 0.0))
            home_wp.z_alt = 0.0
            mavros_wps.append(home_wp)

            for i, wp in enumerate(wp_list):
                m_wp = Waypoint()
                m_wp.frame = Waypoint.FRAME_GLOBAL_REL_ALT
                m_wp.command = 16 # MAV_CMD_NAV_WAYPOINT
                m_wp.is_current = (i == 0)
                m_wp.autocontinue = True
                m_wp.x_lat = float(wp.get("lat", 0.0))
                m_wp.y_long = float(wp.get("lon", 0.0))
                m_wp.z_alt = float(wp.get("alt", 10.0))
                mavros_wps.append(m_wp)

            rospy.loginfo(f"[MissionHandler] MISSION | Pushing {len(mavros_wps)} points")
            resp = self.mavros.mission_push(start_index=0, waypoints=mavros_wps)

            if resp.success:
                self.lynk.send_result(msg, status=RESULT_SUCCESS, message=f"Mission uploaded ({len(mavros_wps)} points)")
            else:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message="MAVROS Mission Push failed")
        except Exception as e:
            rospy.logerr(f"[MissionHandler] Error during MISSION_UPLOAD: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))

    def handle_control(self, msg: Command):
        try:
            params = self._extract_params(msg)
            action = str(params.get("action", "")).upper()

            if action == "START":
                rospy.loginfo("[MissionHandler] MISSION | Received START command")
                if not self._ensure_guided(msg, "MISSION_START"): return
                if not self._ensure_armed(msg, "MISSION_START"): return
                if not self._ensure_takeoff(msg, 5.0, "MISSION_START"): return

                try:
                    self.mavros.mission_set_current(wp_seq=1)
                except:
                    pass

                rospy.loginfo("[MissionHandler] MISSION | Switching to AUTO mode")
                resp = self.mavros.set_mode(mode="AUTO")
                if resp.mode_sent:
                    self.lynk.send_result(msg, status=RESULT_SUCCESS, message="Mission started (AUTO mode set)")
                else:
                    self.lynk.send_result(msg, status=RESULT_FAILURE, message="MAVROS AUTO mode change failed")
            elif action == "ABORT":
                rospy.loginfo("[MissionHandler] MISSION | Aborting mission (Mode -> GUIDED)")
                self.mavros.set_mode(mode="GUIDED")
                self.lynk.send_result(msg, status=RESULT_SUCCESS, message="Mission aborted")
            elif action == "RESUME":
                # User wants to start from the waypoint that started, i.e., the one *before* the current one.
                target_wp = max(1, self.mavros.current_wp - 1)
                rospy.loginfo(f"[MissionHandler] MISSION | Resuming mission from waypoint {target_wp} (Current was {self.mavros.current_wp})")
                
                if not self._ensure_armed(msg, "MISSION_RESUME"): return
                
                # Explicitly set the current waypoint to the target waypoint
                try:
                    self.mavros.mission_set_current(wp_seq=target_wp)
                except Exception as e:
                    rospy.logwarn(f"[MissionHandler] Failed to set waypoint {target_wp}: {e}")

                rospy.loginfo("[MissionHandler] MISSION | Switching to AUTO mode")
                resp = self.mavros.set_mode(mode="AUTO")
                if resp.mode_sent:
                    self.lynk.send_result(msg, status=RESULT_SUCCESS, message=f"Mission resumed (AUTO mode set, targeting WP {target_wp})")
                else:
                    self.lynk.send_result(msg, status=RESULT_FAILURE, message="MAVROS AUTO mode change failed")
            else:
                self.lynk.send_result(msg, status=RESULT_FAILURE, message=f"Unsupported mission action: {action}")
        except Exception as e:
            rospy.logerr(f"[MissionHandler] Error during mission_control handler: {e}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message=str(e))

# --- Mission Pattern Utility Functions ---

def generate_circle_waypoints(center_lat, center_lon, radius_m, num_points, alt):
    """Generates a list of waypoints forming a circle around a center point."""
    waypoints = []
    lat_deg_to_m = 111320.0
    lon_deg_to_m = lat_deg_to_m * math.cos(math.radians(center_lat))
    
    for i in range(num_points):
        angle = (2 * math.pi * i) / num_points
        off_x = radius_m * math.cos(angle)
        off_y = radius_m * math.sin(angle)
        wp_lat = center_lat + (off_y / lat_deg_to_m)
        wp_lon = center_lon + (off_x / lon_deg_to_m)
        waypoints.append({"lat": wp_lat, "lon": wp_lon, "alt": alt})
    return waypoints

def calc_dist_m(lat1, lon1, lat2, lon2):
    """Approximate distance in meters between two lat/lon points."""
    lat_deg_to_m = 111320.0
    avg_lat = (lat1 + lat2) / 2.0
    lon_deg_to_m = lat_deg_to_m * math.cos(math.radians(avg_lat))
    dy = (lat2 - lat1) * lat_deg_to_m
    dx = (lon2 - lon1) * lon_deg_to_m
    return math.sqrt(dx*dx + dy*dy)

def generate_corridor_waypoints(path, speed, alt):
    """Interpolates waypoints for a corridor mission."""
    interpolated = []
    step_m = 2.0 
    
    for i in range(len(path) - 1):
        p1, p2 = path[i], path[i+1]
        dist = calc_dist_m(p1["lat"], p1["lon"], p2["lat"], p2["lon"])
        num_steps = max(1, int(dist / step_m))
        
        for j in range(num_steps):
            frac = j / float(num_steps)
            interpolated.append({
                "lat": p1["lat"] + (p2["lat"] - p1["lat"]) * frac,
                "lon": p1["lon"] + (p2["lon"] - p1["lon"]) * frac,
                "alt": alt
            })
    interpolated.append({"lat": path[-1]["lat"], "lon": path[-1]["lon"], "alt": alt})
    return interpolated
