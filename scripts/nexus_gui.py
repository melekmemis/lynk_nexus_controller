#!/usr/bin/env python3
import sys
import json
import math
import traceback
from datetime import datetime

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QPushButton, QLabel, QLineEdit, QTabWidget, QTextEdit, 
    QFormLayout, QGroupBox, QDoubleSpinBox, QSpinBox, QTableWidget,
    QTableWidgetItem, QHeaderView, QMessageBox, QFrame, QScrollArea,
    QStackedWidget, QSplitter
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QSize, QUrl
from PyQt5.QtGui import QFont, QColor, QPalette, QIcon
from PyQt5.QtWebEngineWidgets import QWebEngineView, QWebEnginePage
import os

import lynk_nexus_sdk as lns
from lynk_controller.handlers.mission import generate_circle_waypoints, generate_corridor_waypoints

# --- Styling Constants ---
DARK_PALETTE = """
QMainWindow { background-color: #121212; }
QWidget { background-color: #121212; color: #E0E0E0; font-family: 'Segoe UI', sans-serif; font-size: 11px; }
QGroupBox { border: 1px solid #2C2C2C; border-radius: 4px; margin-top: 1ex; font-weight: bold; }
QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left; padding: 0 3px; }
QLineEdit, QDoubleSpinBox, QSpinBox { background-color: #1E1E1E; border: 1px solid #333333; border-radius: 4px; padding: 2px; color: #FFFFFF; }
QPushButton { background-color: #2D2D2D; border: 1px solid #444444; border-radius: 4px; padding: 5px; font-weight: bold; }
QPushButton:hover { background-color: #3D3D3D; border-color: #555555; }
QPushButton#actionButton { background-color: #1A73E8; color: white; border: none; }
QPushButton#actionButton:hover { background-color: #1557B0; }
QPushButton#dangerButton { background-color: #D93025; color: white; border: none; }
QPushButton#dangerButton:hover { background-color: #B22222; }
QTabWidget::pane { border: 1px solid #2C2C2C; top: -1px; background: #121212; }
QTabBar::tab { background: #1E1E1E; border: 1px solid #2C2C2C; padding: 4px 8px; min-width: 60px; }
QTabBar::tab:selected { background: #1A73E8; color: white; }
QTextEdit { background-color: #000000; border: 1px solid #2C2C2C; font-family: 'Consolas', monospace; color: #00FF00; font-size: 10px; }
QTableWidget { background-color: #1E1E1E; alternate-background-color: #252525; border: 1px solid #2C2C2C; font-size: 10px; }
QHeaderView::section { background-color: #2D2D2D; color: white; padding: 2px; border: 1px solid #121212; }
"""

class WebPage(QWebEnginePage):
    """Custom WebPage to handle console messages as signals."""
    consoleMsg = pyqtSignal(str)

    def javaScriptConsoleMessage(self, level, message, line, sourceID):
        self.consoleMsg.emit(message)

class SDKWorker(QThread):
    """Worker thread for non-blocking SDK calls."""
    result_ready = pyqtSignal(str, object)
    log_message = pyqtSignal(str)

    def __init__(self, sdk, func_name, *args, **kwargs):
        super().__init__()
        self.sdk = sdk
        self.func_name = func_name
        self.args = args
        self.kwargs = kwargs

    def run(self):
        try:
            func = getattr(self.sdk, self.func_name)
            self.log_message.emit(f"Calling {self.func_name} with {self.args} {self.kwargs}...")
            resp = func(*self.args, **self.kwargs)
            self.result_ready.emit(self.func_name, resp)
        except Exception as e:
            self.log_message.emit(f"Error in {self.func_name}: {str(e)}")
            self.result_ready.emit(self.func_name, {"success": False, "message": str(e)})

class NexusGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LYNK Nexus Controller")
        self.setMinimumSize(200, 300)
        self.resize(450, 500)
        self.setStyleSheet(DARK_PALETTE)
        
        self.setup_ui()

        # Initialize SDK
        try:
            self.sdk = lns.LynkNexusSDK(vehicle_id=1)
            self.log("SDK Initialized successfully.")
        except Exception as e:
            self.log(f"SDK Init Error: {e}", "ERROR")
            self.sdk = None

    def setup_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # --- Top Action Bar ---
        top_bar = QHBoxLayout()
        
        self.btn_takeoff = QPushButton("TAKEOFF (5m)")
        self.btn_takeoff.setObjectName("actionButton")
        self.btn_takeoff.clicked.connect(self.cmd_takeoff)
        
        self.btn_land = QPushButton("LAND")
        self.btn_land.setObjectName("dangerButton")
        self.btn_land.clicked.connect(self.cmd_land)
        
        self.btn_rtl = QPushButton("RTL")
        self.btn_rtl.clicked.connect(self.cmd_rtl)
        
        self.btn_abort = QPushButton("ABORT MISSION")
        self.btn_abort.setObjectName("dangerButton")
        self.btn_abort.clicked.connect(self.cmd_abort)

        self.btn_resume = QPushButton("RESUME MISSION")
        self.btn_resume.setObjectName("actionButton")
        self.btn_resume.clicked.connect(self.cmd_resume)

        top_bar.addWidget(self.btn_takeoff)
        top_bar.addWidget(self.btn_land)
        top_bar.addWidget(self.btn_rtl)
        top_bar.addWidget(self.btn_abort)
        top_bar.addWidget(self.btn_resume)
        main_layout.addLayout(top_bar)

        # --- Tabs ---
        self.tabs = QTabWidget()
        
        # Tab 1: Smart GoTo
        self.tab_goto = QWidget()
        self.setup_goto_tab()
        self.tabs.addTab(self.tab_goto, "Smart GoTo")
        
        # Tab 2: Mission Builder
        self.tab_mission = QWidget()
        self.setup_mission_tab()
        self.tabs.addTab(self.tab_mission, "Mission Builder")
        
        main_layout.addWidget(self.tabs)

        # --- Log Console ---
        main_layout.addWidget(QLabel("Logs:"))
        self.log_console = QTextEdit()
        self.log_console.setReadOnly(True)
        self.log_console.setFixedHeight(80)
        main_layout.addWidget(self.log_console)

    def setup_goto_tab(self):
        layout = QVBoxLayout(self.tab_goto)
        
        form_group = QGroupBox("Target Coordinates")
        form_layout = QFormLayout(form_group)
        
        self.goto_lat = QDoubleSpinBox()
        self.goto_lat.setRange(-90.0, 90.0)
        self.goto_lat.setDecimals(7)
        
        self.goto_lon = QDoubleSpinBox()
        self.goto_lon.setRange(-180.0, 180.0)
        self.goto_lon.setDecimals(7)
        
        self.goto_alt = QDoubleSpinBox()
        self.goto_alt.setRange(0, 2000)
        self.goto_alt.setValue(10.0)
        
        form_layout.addRow("Latitude:", self.goto_lat)
        form_layout.addRow("Longitude:", self.goto_lon)
        form_layout.addRow("Altitude (m):", self.goto_alt)
        
        layout.addWidget(form_group)
        
        btn_go = QPushButton("Execute Smart GoTo")
        btn_go.setObjectName("actionButton")
        btn_go.clicked.connect(self.cmd_goto)
        layout.addWidget(btn_go)
        layout.addStretch()

    def setup_mission_tab(self):
        layout = QVBoxLayout(self.tab_mission)
        
        # Mode Selector
        mode_layout = QHBoxLayout()
        self.mission_type = QPushButton("Manual")
        self.mission_type.setCheckable(True)
        self.mission_type.setChecked(True)

        self.map_btn = QPushButton("Map")
        self.map_btn.setCheckable(True)
        
        self.circle_btn = QPushButton("Circle")
        self.circle_btn.setCheckable(True)
        
        self.corridor_btn = QPushButton("Corridor")
        self.corridor_btn.setCheckable(True)
        
        # Mutual exclusion
        def update_tabs(active_btn):
            btns = [self.mission_type, self.map_btn, self.circle_btn, self.corridor_btn]
            for b in btns:
                b.setChecked(b == active_btn)
            self.mission_stack.setCurrentIndex(btns.index(active_btn))

        self.mission_type.clicked.connect(lambda: update_tabs(self.mission_type))
        self.map_btn.clicked.connect(lambda: update_tabs(self.map_btn))
        self.circle_btn.clicked.connect(lambda: update_tabs(self.circle_btn))
        self.corridor_btn.clicked.connect(lambda: update_tabs(self.corridor_btn))

        mode_layout.addWidget(self.mission_type)
        mode_layout.addWidget(self.map_btn)
        mode_layout.addWidget(self.circle_btn)
        mode_layout.addWidget(self.corridor_btn)
        layout.addLayout(mode_layout)

        # Stacked Layout for Mission Types
        self.mission_stack = QStackedWidget()
        
        # 1. Manual WP
        self.manual_pane = QWidget()
        man_layout = QVBoxLayout(self.manual_pane)
        self.wp_table = QTableWidget(0, 3)
        self.wp_table.setHorizontalHeaderLabels(["Lat", "Lon", "Alt"])
        self.wp_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        man_layout.addWidget(self.wp_table)
        
        man_btns = QHBoxLayout()
        add_wp_btn = QPushButton("+ Add Waypoint")
        add_wp_btn.clicked.connect(self.add_manual_wp)
        remove_wp_btn = QPushButton("- Remove Selected")
        remove_wp_btn.clicked.connect(self.remove_selected_wp)
        man_btns.addWidget(add_wp_btn)
        man_btns.addWidget(remove_wp_btn)
        man_layout.addLayout(man_btns)
        
        # 2. Map & List Integrated
        self.map_pane = QWidget()
        map_layout = QVBoxLayout(self.map_pane)
        
        splitter = QSplitter(Qt.Horizontal)
        
        # Map View
        self.map_view = QWebEngineView()
        self.map_page = WebPage()
        self.map_page.consoleMsg.connect(self.handle_map_message)
        self.map_view.setPage(self.map_page)
        map_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "map.html")
        self.map_view.setUrl(QUrl.fromLocalFile(map_path))
        
        # Side List for Map
        self.map_list = QTableWidget(0, 3)
        self.map_list.setHorizontalHeaderLabels(["#", "Lat", "Lon"])
        self.map_list.horizontalHeader().setSectionResizeMode(QHeaderView.Interactive)
        self.map_list.horizontalHeader().setStretchLastSection(True)
        self.map_list.setMinimumWidth(80)
        
        splitter.addWidget(self.map_view)
        splitter.addWidget(self.map_list)
        splitter.setStretchFactor(0, 3)
        splitter.setStretchFactor(1, 1)
        
        map_layout.addWidget(splitter)
        map_layout.addWidget(QLabel("Click map to add points. Markers are numbered by order."))

        # 3. Circle
        self.circle_pane = QWidget()
        circ_layout = QFormLayout(self.circle_pane)
        self.c_lat = QDoubleSpinBox(); self.c_lat.setRange(-90, 90); self.c_lat.setDecimals(7)
        self.c_lon = QDoubleSpinBox(); self.c_lon.setRange(-180, 180); self.c_lon.setDecimals(7)
        self.c_radius = QDoubleSpinBox(); self.c_radius.setRange(1, 1000); self.c_radius.setValue(20)
        self.c_count = QSpinBox(); self.c_count.setRange(3, 100); self.c_count.setValue(16)
        self.c_alt = QDoubleSpinBox(); self.c_alt.setRange(0, 500); self.c_alt.setValue(10)
        circ_layout.addRow("Center Lat:", self.c_lat)
        circ_layout.addRow("Center Lon:", self.c_lon)
        circ_layout.addRow("Radius (m):", self.c_radius)
        circ_layout.addRow("Point Count:", self.c_count)
        circ_layout.addRow("Altitude (m):", self.c_alt)

        # 3. Corridor
        self.corridor_pane = QWidget()
        corr_layout = QVBoxLayout(self.corridor_pane)
        self.corr_table = QTableWidget(0, 2)
        self.corr_table.setHorizontalHeaderLabels(["Lat", "Lon"])
        self.corr_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        corr_layout.addWidget(QLabel("Define Path Nodes:"))
        corr_layout.addWidget(self.corr_table)
        corr_btns = QHBoxLayout()
        add_node = QPushButton("+ Add Node"); add_node.clicked.connect(self.add_corr_node)
        corr_btns.addWidget(add_node)
        corr_layout.addLayout(corr_btns)
        
        self.corr_speed = QDoubleSpinBox(); self.corr_speed.setValue(3.0)
        self.corr_alt = QDoubleSpinBox(); self.corr_alt.setValue(10.0)
        self.corr_width = QDoubleSpinBox(); self.corr_width.setRange(0.1, 50.0); self.corr_width.setValue(2.0)
        
        c_form = QFormLayout()
        c_form.addRow("Velocity (m/s):", self.corr_speed)
        c_form.addRow("Altitude (m):", self.corr_alt)
        c_form.addRow("Width (m):", self.corr_width)
        corr_layout.addLayout(c_form)

        self.mission_stack.addWidget(self.manual_pane)
        self.mission_stack.addWidget(self.map_pane)
        self.mission_stack.addWidget(self.circle_pane)
        self.mission_stack.addWidget(self.corridor_pane)
        layout.addWidget(self.mission_stack)

        # Upload & Start
        upload_btn = QPushButton("UPLOAD MISSION")
        upload_btn.setObjectName("actionButton")
        upload_btn.clicked.connect(self.cmd_upload_mission)
        layout.addWidget(upload_btn)

    def add_manual_wp(self):
        row = self.wp_table.rowCount()
        self.wp_table.insertRow(row)
        for i, val in enumerate([0.0, 0.0, 10.0]):
            self.wp_table.setItem(row, i, QTableWidgetItem(str(val)))

    def add_corr_node(self):
        row = self.corr_table.rowCount()
        self.corr_table.insertRow(row)
        self.corr_table.setItem(row, 0, QTableWidgetItem("0.0"))
        self.corr_table.setItem(row, 1, QTableWidgetItem("0.0"))

    def remove_selected_wp(self):
        row = self.wp_table.currentRow()
        if row < 0: return
        
        item = self.wp_table.item(row, 0)
        if item:
            marker_id = item.data(Qt.UserRole)
            if marker_id:
                # Tell map to remove marker
                self.map_view.page().runJavaScript(f"removeMarker({marker_id})")
        
        self.wp_table.removeRow(row)
        self.log("Removed selected waypoint from list and map")

    # --- Commands ---
    def log(self, text, level="INFO"):
        time_str = datetime.now().strftime("%H:%M:%S")
        self.log_console.append(f"[{time_str}] {level}: {text}")

    def handle_map_message(self, message):
        if message.startswith("SYNC:"):
            try:
                data = json.loads(message[5:])
                self.sync_map_data_to_tables(data)
            except Exception as e:
                self.log(f"Map SYNC error: {e}", "ERROR")

    def sync_map_data_to_tables(self, data):
        # Update both the map_list and the main wp_table
        for table in [self.map_list, self.wp_table]:
            table.setRowCount(0)
            for i, wp in enumerate(data):
                row = table.rowCount()
                table.insertRow(row)
                
                # Column 0: Index or Lat
                if table == self.map_list:
                    table.setItem(row, 0, QTableWidgetItem(str(i + 1)))
                    table.setItem(row, 1, QTableWidgetItem(f"{wp['lat']:.7f}"))
                    table.setItem(row, 2, QTableWidgetItem(f"{wp['lon']:.7f}"))
                else:
                    lat_item = QTableWidgetItem(f"{wp['lat']:.7f}")
                    lat_item.setData(Qt.UserRole, str(wp['id']))
                    table.setItem(row, 0, lat_item)
                    table.setItem(row, 1, QTableWidgetItem(f"{wp['lon']:.7f}"))
                    table.setItem(row, 2, QTableWidgetItem("10.0"))
        
        self.log(f"Synced {len(data)} waypoints from map")

    def run_sdk_worker(self, func_name, *args, **kwargs):
        if not self.sdk:
            self.log("SDK not connected!", "ERROR")
            return
        self.worker = SDKWorker(self.sdk, func_name, *args, **kwargs)
        self.worker.log_message.connect(self.log)
        self.worker.result_ready.connect(self.handle_sdk_result)
        self.worker.start()

    def handle_sdk_result(self, func, resp):
        status = "Success"
        msg = ""
        
        # Normalize response parsing (Sdk responses can be complex objects)
        if hasattr(resp, "result"):
            if resp.result.status != 0:
                status = "FAILED"
                msg = resp.result.message
        elif hasattr(resp, "success") and not resp.success:
            status = "FAILED"
            msg = getattr(resp, "message", "Unknown error")
        elif isinstance(resp, dict) and not resp.get("success", True):
             status = "FAILED"
             msg = resp.get("message", "Error")

        self.log(f"{func} finished: {status} {msg}")
        if status == "FAILED":
            QMessageBox.critical(self, "Command Failed", f"{func} failed:\n{msg}")

    def cmd_takeoff(self):
        self.run_sdk_worker("flight_takeoff", altitude_m=5.0, wait_for_ack=True)

    def cmd_land(self):
        self.run_sdk_worker("flight_land", wait_for_ack=True)

    def cmd_rtl(self):
        self.run_sdk_worker("flight_set_mode", mode='RTL', wait_for_ack=True)

    def cmd_abort(self):
        self.log("Aborting mission...")
        payload = {"action": "ABORT"}
        self.run_sdk_worker("send_command", command_name="MISSION_CONTROL", params={"json": json.dumps(payload)}, wait_for_ack=True)

    def cmd_resume(self):
        self.log("Resuming mission...")
        payload = {"action": "RESUME"}
        self.run_sdk_worker("send_command", command_name="MISSION_CONTROL", params={"json": json.dumps(payload)}, wait_for_ack=True)

    def cmd_goto(self):
        lat = self.goto_lat.value()
        lon = self.goto_lon.value()
        alt = self.goto_alt.value()
        self.run_sdk_worker("flight_goto", lat=lat, lon=lon, alt=alt, wait_for_ack=True)

    def cmd_upload_mission(self):
        waypoints = []
        idx = self.mission_stack.currentIndex()
        
        try:
            width_m = None
            if idx in [0, 1]: # Manual or Map
                for r in range(self.wp_table.rowCount()):
                    waypoints.append({
                        "lat": float(self.wp_table.item(r, 0).text()),
                        "lon": float(self.wp_table.item(r, 1).text()),
                        "alt": float(self.wp_table.item(r, 2).text())
                    })
            elif idx == 2: # Circle
                waypoints = generate_circle_waypoints(
                    self.c_lat.value(), self.c_lon.value(), 
                    self.c_radius.value(), self.c_count.value(), self.c_alt.value()
                )
            elif idx == 3: # Corridor
                path = []
                for r in range(self.corr_table.rowCount()):
                    path.append({
                        "lat": float(self.corr_table.item(r, 0).text()),
                        "lon": float(self.corr_table.item(r, 1).text())
                    })
                if len(path) < 2:
                    raise ValueError("At least 2 nodes required for corridor")
                waypoints = generate_corridor_waypoints(path, self.corr_speed.value(), self.corr_alt.value())
                width_m = self.corr_width.value()
            
            if not waypoints:
                self.log("No waypoints defined!", "WARNING")
                return

            self.log(f"Uploading mission with {len(waypoints)} points...")
            if width_m is not None:
                payload = {
                    "mission_id": 1,
                    "waypoints": waypoints,
                    "replace_existing": True,
                    "width_m": width_m
                }
                self.run_sdk_worker("send_command", command_name="MISSION_UPLOAD", params={"json": json.dumps(payload)}, wait_for_ack=True)
            else:
                self.run_sdk_worker("mission_upload", mission_id=1, waypoints=waypoints, replace_existing=True, wait_for_ack=True)
            
            # Ask to start
            reply = QMessageBox.question(self, "Mission Uploaded", "Start mission now?", QMessageBox.Yes | QMessageBox.No)
            if reply == QMessageBox.Yes:
                self.log("Starting mission execution...")
                payload = {"action": "START"}
                self.run_sdk_worker("send_command", command_name="MISSION_CONTROL", params={"json": json.dumps(payload)}, wait_for_ack=True)

        except Exception as e:
            self.log(f"Mission Preparation Error: {e}", "ERROR")
            QMessageBox.warning(self, "Mission Error", str(e))

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = NexusGUI()
    gui.show()
    sys.exit(app.exec_())
