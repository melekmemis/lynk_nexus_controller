#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from lynk_controller.controller import VehicleController

def main():
    try:
        rospy.init_node("lynk_nexus_vehicle_controller", anonymous=False)
        
        vehicle_id = rospy.get_param("~vehicle_id", 1)
        
        controller = VehicleController(vehicle_id=vehicle_id)
        
        rospy.loginfo(f"[VehicleControllerNode] Node started for Vehicle ID: {vehicle_id}")
        rospy.spin()
        
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logfatal(f"[VehicleControllerNode] Fatal error: {e}")

if __name__ == "__main__":
    main()
