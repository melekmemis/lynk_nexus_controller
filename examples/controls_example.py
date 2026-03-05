#!/usr/bin/env python3
import sys
import tty
import termios
import json
import math
import lynk_nexus_sdk as lns
from lynk_controller.handlers.mission import generate_circle_waypoints, generate_corridor_waypoints

def getch():
    """Reads a single character from stdin without waiting for Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def prompt_num(prompt_msg, required=True, default=None, min_val=None, max_val=None):
    """Prompts for a number until a valid one is provided."""
    while True:
        try:
            val_str = input(prompt_msg).strip()
            if not val_str:
                if not required: return default
                if default is not None: return default
                print("Error: This field is required.")
                continue
            
            val = float(val_str)
            if min_val is not None and val < min_val:
                print(f"Error: Value must be at least {min_val}.")
                continue
            if max_val is not None and val > max_val:
                print(f"Error: Value must be at most {max_val}.")
                continue
            return val
        except ValueError:
            print("Error: Invalid input. Please enter a number.")
        except KeyboardInterrupt:
            raise KeyboardInterrupt

def main():
    print("--- LYNK Nexus Interactive Keyboard Control ---")
    print("Commands:")
    print("  [g] - Smart GoTo (Manual coordinates required)")
    print("  [t] - Takeoff (5m)")
    print("  [l] - Land")
    print("  [r] - Return to Launch (RTL)")
    print("  [m] - Waypoint Mission (Interactive builder)")
    print("  [q] - Quit")
    print("-----------------------------------------------")

    # Initialize SDK (Assumes Vehicle ID 1)
    try:
        sdk = lns.LynkNexusSDK(vehicle_id=1)
    except Exception as e:
        print(f"Error initializing SDK: {e}")
        return

    try:
        while True:
            char = getch()
            if not char:
                continue
            
            # Detect Ctrl+C (raw mode reads it as \x03)
            if char == '\x03' or char.lower() == 'q':
                print("\nExiting...")
                break
                
            char = char.lower()
            
            if char == 'g':
                print("\n[KEY] 'G' pressed -> Smart GoTo Mode")
                try:
                    lat = prompt_num("Enter Latitude (-90 to 90): ", min_val=-90, max_val=90)
                    lon = prompt_num("Enter Longitude (-180 to 180): ", min_val=-180, max_val=180)
                    alt = prompt_num("Enter Altitude (0 to 2000, default 10.0m): ", required=False, default=10.0, min_val=0, max_val=2000)
                    
                    print(f"Sending Smart GoTo to {lat}, {lon} @ {alt}m...")
                    resp = sdk.flight_goto(lat=lat, lon=lon, alt=alt, wait_for_ack=True)
                    
                    if hasattr(resp, "result") and resp.result.status != 0:
                        print(f"Smart GoTo Failed: {resp.result.message}")
                    elif hasattr(resp, "success") and not resp.success:
                        print(f"Smart GoTo Failed: {resp.message}")
                        
                except KeyboardInterrupt:
                    print("\nCancelled Smart GoTo input.")
                    continue
                
            elif char == 't':
                print("\n[KEY] 'T' pressed -> Takeoff Mode")
                try:
                    alt = prompt_num("Enter Takeoff Altitude (0 to 2000, default 5.0m): ", required=False, default=5.0, min_val=0, max_val=2000)
                    print(f"Sending Takeoff command for {alt}m...")
                    resp = sdk.flight_takeoff(altitude_m=alt, wait_for_ack=True)
                    
                    if hasattr(resp, "result") and resp.result.status != 0:
                        print(f"Takeoff Failed: {resp.result.message}")
                except KeyboardInterrupt:
                    print("\nCancelled Takeoff input.")
                    continue
                
            elif char == 'l':
                print("\n[KEY] 'L' pressed -> Sending Land...")
                sdk.flight_land(wait_for_ack=True)
                
            elif char == 'r':
                print("\n[KEY] 'R' pressed -> Sending RTL...")
                sdk.flight_set_mode(mode='RTL', wait_for_ack=True)

            elif char == 'm':
                print("\n[KEY] 'M' pressed -> Waypoint Mission Builder")
                waypoints = []
                try:
                    print("\nSelect Mission Type:")
                    print("  [1] Manual Waypoints")
                    print("  [2] Circle Pattern")
                    print("  [3] Corridor Route")
                    choice = input("Choice (default 1): ").strip()
                    
                    if choice == '2':
                        print("\n--- Circle Pattern Generator ---")
                        c_lat = prompt_num("  Center Latitude: ", min_val=-90, max_val=90)
                        c_lon = prompt_num("  Center Longitude: ", min_val=-180, max_val=180)
                        radius = prompt_num("  Radius (meters): ", min_val=1)
                        count = int(prompt_num("  Number of Waypoints (default 16): ", min_val=3, default=16))
                        c_alt = prompt_num("  Altitude (meters, default 10.0): ", required=False, default=10.0)
                        
                        waypoints = generate_circle_waypoints(c_lat, c_lon, radius, count, c_alt)
                    elif choice == '3':
                        print("\n--- Corridor Route Generator ---")
                        corridor_path = []
                        while True:
                            print(f"\n  Adding Route Node #{len(corridor_path) + 1} ---")
                            lat = prompt_num("    Latitude: ", min_val=-90, max_val=90)
                            lon = prompt_num("    Longitude: ", min_val=-180, max_val=180)
                            corridor_path.append({"lat": lat, "lon": lon})
                            if input("    Add another node? (y/n, default 'n'): ").strip().lower() != 'y':
                                break
                                
                        speed = prompt_num("  Flight Speed (m/s, default 3.0): ", required=False, default=3.0)
                        alt = prompt_num("  Altitude (meters, default 10.0): ", required=False, default=10.0)
                        width = prompt_num("  Corridor Width (meters, default 5.0): ", required=False, default=5.0)
                        
                        print(f"\nGenerating interpolated waypoints for {width}m corridor...")
                        waypoints = generate_corridor_waypoints(corridor_path, speed, alt)
                        
                        print(f"\nUploading Corridor mission with {len(waypoints)} points...")
                        payload = {
                            "mission_id": 1,
                            "waypoints": waypoints,
                            "width_m": width,
                            "replace_existing": True
                        }
                        resp = sdk.send_command(command_name="MISSION_UPLOAD", params={"json": json.dumps(payload)}, wait_for_ack=True)
                        
                        if hasattr(resp, "result") and resp.result.status != 0:
                            print(f"Mission Upload Failed: {resp.result.message}")
                        else:
                            start = input("\nCorridor uploaded successfully. Start now? (y/n, default 'y'): ").strip().lower()
                            if start != 'n':
                                print("Starting mission (Auto mode)...")
                                payload = {"action": "START"}
                                sdk.send_command(command_name="MISSION_CONTROL", params={"json": json.dumps(payload)}, wait_for_ack=True)
                        continue 
                    else:
                        while True:
                            print(f"\n--- Adding Waypoint #{len(waypoints) + 1} ---")
                            lat = prompt_num("  Latitude (-90 to 90): ", min_val=-90, max_val=90)
                            lon = prompt_num("  Longitude (-180 to 180): ", min_val=-180, max_val=180)
                            alt = prompt_num("  Altitude (0 to 2000, default 10.0m): ", required=False, default=10.0, min_val=0, max_val=2000)
                            
                            waypoints.append({"lat": lat, "lon": lon, "alt": alt})
                            
                            cont = input("\nAdd another waypoint? (y/n, default 'n'): ").strip().lower()
                            if cont != 'y':
                                break
                    
                    if not waypoints:
                        print("No waypoints entered. Mission cancelled.")
                        continue

                    print(f"\nUploading mission with {len(waypoints)} waypoints...")
                    resp = sdk.mission_upload(mission_id=1, waypoints=waypoints, wait_for_ack=True)
                    
                    if hasattr(resp, "result") and resp.result.status != 0:
                        print(f"Mission Upload Failed: {resp.result.message}")
                    else:
                        start = input("\nMission uploaded successfully. Start mission now? (y/n, default 'y'): ").strip().lower()
                        if start != 'n':
                            print("Starting mission (Auto mode)...")
                            # Wrap in JSON for protocol engine compatibility
                            payload = {"action": "START"}
                            sdk.send_command(command_name="MISSION_CONTROL", params={"json": json.dumps(payload)}, wait_for_ack=True)
                            
                except KeyboardInterrupt:
                    print("\nMission building cancelled.")
                    continue

    except KeyboardInterrupt:
        print("\nExiting (Ctrl+C)...")

if __name__ == "__main__":
    main()
