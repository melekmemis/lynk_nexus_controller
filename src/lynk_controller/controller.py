#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import threading
from typing import Dict, Any

from lynk_nexus.msg import Command, State, Gps
from mavros_msgs.msg import State as MavrosState

from lynk_controller.interfaces import MavrosInterface, LynkInterface, RESULT_FAILURE
from lynk_controller.handlers import FlightHandler, MissionHandler, NavigationHandler

rospy.loginfo("[VehicleController] DEBUG | controller.py version 3.0 (Modularized) LOADED")

class VehicleController:
    """
    Bridges LYNK Nexus protocol commands to MAVROS/ArduPilot actions.
    Modular design using Interfaces and Handlers.
    """

    def __init__(self, vehicle_id: int = 1):
        self.vehicle_id = vehicle_id
        self.namespace = f"/vehicles/{self.vehicle_id}/lynk"
        
        # Interfaces
        self.mavros = MavrosInterface()
        self.lynk = LynkInterface(self.vehicle_id, self.namespace)

        # Handlers
        self.flight_handler = FlightHandler(self.mavros, self.lynk)
        self.mission_handler = MissionHandler(self.mavros, self.lynk)
        self.nav_handler = NavigationHandler(self.mavros, self.lynk)

        # Safety State
        self.low_battery_triggered = False
        self._safety_timer = rospy.Timer(rospy.Duration(1.0), self._check_battery)


        # Subscribers
        self._cmd_sub = rospy.Subscriber(f"{self.namespace}/rx/command", Command, self._on_command)
        self._state_sub = rospy.Subscriber(f"{self.namespace}/rx/state", State, self._on_lynk_state)
        self._gps_sub = rospy.Subscriber(f"{self.namespace}/rx/gps", Gps, self._on_lynk_gps)
        self._mavros_state_sub = rospy.Subscriber("/mavros/state", MavrosState, self._on_mavros_state)

        rospy.loginfo(f"[VehicleController] Initialized for Vehicle ID: {self.vehicle_id}")

    def _on_mavros_state(self, msg: MavrosState):
        self.mavros.update_state(msg)

    def _on_lynk_state(self, msg: State):
        # We can still track LYNK-side state if needed, 
        # but most logic now relies on MAVROS state directly via MavrosInterface
        pass

    def _on_lynk_gps(self, msg: Gps):
        self.mavros.update_gps(msg)

    def _check_battery(self, event):
        """Monitors battery and triggers RTL if level is below 10%."""
        if self.mavros.battery_remaining < 0.10 and not self.low_battery_triggered:
            if self.mavros.armed:


                rospy.logwarn(f"[VehicleController] LOW BATTERY ALERT | Level: {self.mavros.battery_remaining*100:.1f}%. Triggering RTL!")
                self.mavros.rtl()
                self.low_battery_triggered = True
            else:
                # If not armed, just log once
                rospy.logwarn_once(f"[VehicleController] LOW BATTERY ALERT | Level: {self.mavros.battery_remaining*100:.1f}%")
        elif self.mavros.battery_remaining >= 0.15:
            # Reset trigger if battery is charged above a hysteresis threshold
            self.low_battery_triggered = False

    def _on_command(self, msg: Command):

        """Dispatches LYNK commands to specific handlers."""
        cmd_name = msg.command_name.strip().upper()
        rospy.loginfo(f"[VehicleController] >>> RECV <<< | CMD: '{cmd_name}'")

        # Send Acknowledgment immediately
        self.lynk.send_ack(msg)

        # Dispatch
        if cmd_name == "FLIGHT_ARMING":
            self.flight_handler.handle_arming(msg)
        elif cmd_name == "FLIGHT_TAKEOFF":
            threading.Thread(target=self.flight_handler.handle_takeoff, args=(msg,)).start()
        elif cmd_name == "FLIGHT_SET_MODE":
            self.flight_handler.handle_set_mode(msg)
        elif cmd_name == "FLIGHT_LAND":
            self.flight_handler.handle_land(msg)
        elif cmd_name == "FLIGHT_RTL":
            self.flight_handler.handle_rtl(msg)
        elif cmd_name in ["MISSION_UPLOAD", "FLIGHT_MISSION_UPLOAD"]:
            self.mission_handler.handle_upload(msg)
        elif cmd_name in ["MISSION_CONTROL", "FLIGHT_MISSION_CONTROL"]:
            threading.Thread(target=self.mission_handler.handle_control, args=(msg,)).start()
        elif cmd_name in ["FLIGHT_GOTO", "SMART_GOTO"]:
            threading.Thread(target=self.nav_handler.handle_smart_goto, args=(msg,)).start()
        else:
            rospy.logwarn(f"[VehicleController] Unsupported command: {cmd_name}")
            self.lynk.send_result(msg, status=RESULT_FAILURE, message="Unsupported command")
