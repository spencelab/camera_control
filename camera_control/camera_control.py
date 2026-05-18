#!/usr/bin/env python3
# camera_control.py
#
# Minimal ROS2 + Qt GUI for multi-camera control and recording modes.
# - Left: discover camera nodes, list & select
# - Right: common settings + recorder pane
# - Configure: sets parameters on selected nodes
# - Start/Stop: toggles 'recording' param and 'record_mode' on selected nodes
#
# Assumptions (tweak to match your stack):
#   Remote camera nodes expose ROS 2 parameters:
#       width:int, height:int, exposure:int(µs), external_trigger:bool, fps_limit:int
#       state:str (optional; one of "idle","buffering","dumping","streaming")
#       recording:bool (we toggle), record_mode:str in {"ram_raw_avi","ram_mp4","debayerhalf_mp4"}
#
# Tested conceptually with rclpy + PySide6. You can swap PySide6 → PyQt6 if you prefer.
#
# Run:
#   ros2 run <your_pkg> camera_control.py   (or)  python3 camera_control.py
#
# Deps:
#   pip/conda: PySide6
#   ROS2: rclpy
#
# Notes:
#   - We integrate ROS spinning via a QTimer so the UI stays responsive.
#   - Discovery pattern is simple: node names starting with "cam". Adjust CAM_NAME_PREDICATE.

import sys
import asyncio
from typing import List, Tuple, Dict, Optional

from PySide6 import QtCore, QtWidgets, QtGui

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.task import Future
from rcl_interfaces.srv import GetParameters, SetParameters
from rcl_interfaces.msg import ParameterValue


# --------------------------
# Config / constants
# --------------------------
DISCOVERY_INTERVAL_MS = 1000
SPIN_INTERVAL_MS = 10

# A simple predicate; adjust to match your naming (or use a dedicated topic/service to enumerate).
def CAM_NAME_PREDICATE(name: str) -> bool:
    return name.startswith("cam")

class AsyncParametersClient:
    """Simple async parameter client for one remote node."""
    def __init__(self, parent_node: Node, remote_name: str):
        self.node = parent_node
        self.remote_name = remote_name
        self.cli_get = self.node.create_client(GetParameters, f"{remote_name}/get_parameters")
        self.cli_set = self.node.create_client(SetParameters, f"{remote_name}/set_parameters")

    async def wait_for_service(self, timeout_sec=2.0):
        await self.cli_get.wait_for_service(timeout_sec=timeout_sec)
        await self.cli_set.wait_for_service(timeout_sec=timeout_sec)

    async def get_parameters(self, names):
        await self.wait_for_service()
        req = GetParameters.Request(names=names)
        future = self.cli_get.call_async(req)
        resp = await future
        vals = []
        for v in resp.values:
            if v.type == Parameter.Type.NOT_SET:
                vals.append(None)
            else:
                vals.append(Parameter(name='', type_=v.type, value=v.bool_value if v.type == Parameter.Type.BOOL
                                      else v.integer_value if v.type == Parameter.Type.INTEGER
                                      else v.double_value if v.type == Parameter.Type.DOUBLE
                                      else v.string_value))
        return vals

    async def set_parameters(self, params):
        await self.wait_for_service()
        req = SetParameters.Request()
        req.parameters = [p.to_parameter_msg() for p in params]
        future = self.cli_set.call_async(req)
        resp = await future
        return resp.results

# --------------------------
# ROS helper wrappers
# --------------------------
class CameraControlNode(Node):
    def __init__(self):
        super().__init__("camera_control")
        self._param_clients: Dict[Tuple[str, str], AsyncParametersClient] = {}
        self.get_logger().info("camera_control node started")

    def list_nodes(self) -> List[Tuple[str, str]]:
        # Returns list of (name, namespace)
        return self.get_node_names_and_namespaces()

    def get_param_client(self, name: str, namespace: str) -> AsyncParametersClient:
        key = (name, namespace)
        if key not in self._param_clients:
            full_name = namespace.rstrip("/") + "/" + name if namespace not in ("", "/") else name
            # AsyncParametersClient takes a NodeName or Node object; use NodeName via remote_name kw
            self._param_clients[key] = AsyncParametersClient(
                self,
                remote_name=full_name
            )
        return self._param_clients[key]

    async def try_get_string_param(self, name: str, namespace: str, param_name: str) -> Optional[str]:
        client = self.get_param_client(name, namespace)
        try:
            value = await client.get_parameters([param_name])
            if value and value[0] is not None and value[0].type_ != Parameter.Type.NOT_SET:
                return value[0].value
        except Exception as e:
            self.get_logger().debug(f"get_param {name}{namespace}:{param_name} failed: {e}")
        return None

    async def set_params_bulk(self, nodes: List[Tuple[str, str]], params: List[Parameter]) -> Dict[Tuple[str, str], bool]:
        results: Dict[Tuple[str, str], bool] = {}
        tasks = []
        for (n, ns) in nodes:
            client = self.get_param_client(n, ns)
            tasks.append((n, ns, client.set_parameters(params)))
        # run concurrently
        for (n, ns, fut) in tasks:
            try:
                res = await fut
                ok = all(out.successful for out in res)
                results[(n, ns)] = ok
            except Exception as e:
                self.get_logger().warn(f"SetParameters failed for {ns}/{n}: {e}")
                results[(n, ns)] = False
        return results


# --------------------------
# Qt GUI
# --------------------------
class CameraTable(QtWidgets.QTableWidget):
    COL_CAMERA = 0
    COL_NAMESPACE = 1
    COL_STATE = 2

    def __init__(self):
        super().__init__(0, 3)
        self.setHorizontalHeaderLabels(["Camera", "Namespace", "State"])
        self.horizontalHeader().setStretchLastSection(True)
        self.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.setSelectionMode(QtWidgets.QAbstractItemView.MultiSelection)
        self.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.verticalHeader().setVisible(False)

    def set_rows(self, rows: List[Tuple[str, str, str]]):
        self.setRowCount(0)
        for cam, ns, state in rows:
            r = self.rowCount()
            self.insertRow(r)
            self.setItem(r, self.COL_CAMERA, QtWidgets.QTableWidgetItem(cam))
            self.setItem(r, self.COL_NAMESPACE, QtWidgets.QTableWidgetItem(ns))
            self.setItem(r, self.COL_STATE, QtWidgets.QTableWidgetItem(state))

    def selected_nodes(self) -> List[Tuple[str, str]]:
        res = []
        for idx in self.selectionModel().selectedRows():
            name = self.item(idx.row(), self.COL_CAMERA).text()
            ns   = self.item(idx.row(), self.COL_NAMESPACE).text()
            res.append((name, ns))
        return res


class RightPane(QtWidgets.QWidget):
    configure_clicked = QtCore.Signal()
    start_clicked = QtCore.Signal()
    stop_clicked = QtCore.Signal()

    def __init__(self):
        super().__init__()

        # --- Camera Settings group ---
        self.width_spin   = QtWidgets.QSpinBox()
        self.height_spin  = QtWidgets.QSpinBox()
        self.exposure_spin = QtWidgets.QSpinBox()
        self.ext_trig_chk = QtWidgets.QCheckBox("External hardware trigger")
        self.fps_limit_spin = QtWidgets.QSpinBox()

        self.width_spin.setRange(64, 8192)
        self.height_spin.setRange(32, 4320)
        self.exposure_spin.setRange(1, 10000000)  # µs
        self.exposure_spin.setSingleStep(100)
        self.fps_limit_spin.setRange(0, 1000)
        self.fps_limit_spin.setValue(0)
        self.fps_limit_spin.setToolTip("0 = unlimited (no software limiter)")

        cam_form = QtWidgets.QFormLayout()
        cam_form.addRow("Width", self.width_spin)
        cam_form.addRow("Height", self.height_spin)
        cam_form.addRow("Exposure (µs)", self.exposure_spin)
        cam_form.addRow("", self.ext_trig_chk)
        cam_form.addRow("FPS limit (soft)", self.fps_limit_spin)

        cam_group = QtWidgets.QGroupBox("Camera Settings (applies to all selected)")
        cam_group.setLayout(cam_form)

        # --- Recorder group ---
        self.mode_combo = QtWidgets.QComboBox()
        # Map user labels → internal param strings
        self.mode_combo.addItem("RAM ringbuffer → uncompressed AVI", "ram_raw_avi")
        self.mode_combo.addItem("RAM ringbuffer → processed MP4", "ram_mp4")
        self.mode_combo.addItem("Streaming → debayerHalf MP4 w/ timestamps", "debayerhalf_mp4")

        self.configure_btn = QtWidgets.QPushButton("Configure")
        self.start_btn = QtWidgets.QPushButton("Start")
        self.stop_btn = QtWidgets.QPushButton("Stop")

        rec_layout = QtWidgets.QFormLayout()
        rec_layout.addRow("Record mode", self.mode_combo)

        btns = QtWidgets.QHBoxLayout()
        btns.addWidget(self.configure_btn)
        btns.addWidget(self.start_btn)
        btns.addWidget(self.stop_btn)

        rec_group = QtWidgets.QGroupBox("Recorder")
        rec_v = QtWidgets.QVBoxLayout()
        rec_v.addLayout(rec_layout)
        rec_v.addLayout(btns)
        rec_group.setLayout(rec_v)

        # Assemble right pane
        v = QtWidgets.QVBoxLayout()
        v.addWidget(cam_group)
        v.addWidget(rec_group)
        v.addStretch(1)
        self.setLayout(v)

        # signals
        self.configure_btn.clicked.connect(self.configure_clicked.emit)
        self.start_btn.clicked.connect(self.start_clicked.emit)
        self.stop_btn.clicked.connect(self.stop_clicked.emit)

    def get_camera_params(self) -> Dict[str, Parameter]:
        return {
            "width": Parameter("width", Parameter.Type.INTEGER, self.width_spin.value()),
            "height": Parameter("height", Parameter.Type.INTEGER, self.height_spin.value()),
            "exposure": Parameter("exposure", Parameter.Type.INTEGER, self.exposure_spin.value()),
            "external_trigger": Parameter("external_trigger", Parameter.Type.BOOL, self.ext_trig_chk.isChecked()),
            "fps_limit": Parameter("fps_limit", Parameter.Type.INTEGER, self.fps_limit_spin.value()),
        }

    def get_record_mode_param(self) -> Parameter:
        internal = self.mode_combo.currentData()
        return Parameter("record_mode", Parameter.Type.STRING, str(internal))


class MainWindow(QtWidgets.QWidget):
    def __init__(self, ros_node: CameraControlNode):
        super().__init__()
        self.setWindowTitle("camera_control")
        self.resize(1100, 650)
        self.ros = ros_node

        # Left pane
        self.discover_btn = QtWidgets.QPushButton("Discover cameras")
        self.table = CameraTable()

        left_v = QtWidgets.QVBoxLayout()
        left_v.addWidget(self.discover_btn)
        left_v.addWidget(self.table)

        left = QtWidgets.QWidget()
        left.setLayout(left_v)

        # Right pane
        self.right = RightPane()

        # Layout
        splitter = QtWidgets.QSplitter()
        splitter.setOrientation(QtCore.Qt.Horizontal)
        splitter.addWidget(left)
        splitter.addWidget(self.right)
        splitter.setStretchFactor(0, 2)
        splitter.setStretchFactor(1, 3)

        top = QtWidgets.QVBoxLayout()
        top.addWidget(splitter)
        self.setLayout(top)

        # Connect
        self.discover_btn.clicked.connect(self.on_discover)
        self.right.configure_clicked.connect(self.on_configure)
        self.right.start_clicked.connect(self.on_start)
        self.right.stop_clicked.connect(self.on_stop)

        # timers
        self.disc_timer = QtCore.QTimer(self)
        self.disc_timer.setInterval(DISCOVERY_INTERVAL_MS)
        self.disc_timer.timeout.connect(self.refresh_states)
        self.disc_timer.start()

        # immediately populate once
        self.on_discover()

    def current_rows(self) -> List[Tuple[str, str, str]]:
        """Return rows currently shown: (name, namespace, state)"""
        rows = []
        for r in range(self.table.rowCount()):
            name = self.table.item(r, CameraTable.COL_CAMERA).text()
            ns   = self.table.item(r, CameraTable.COL_NAMESPACE).text()
            state = self.table.item(r, CameraTable.COL_STATE).text()
            rows.append((name, ns, state))
        return rows

    def on_discover(self):
        # scan ROS graph
        nodes = self.ros.list_nodes()
        cams = [(n, ns) for (n, ns) in nodes if CAM_NAME_PREDICATE(n)]
        # fill with "unknown" state, then refresh asynchronously
        self.table.set_rows([(n, ns, "unknown") for (n, ns) in cams])
        # kick a refresh pass
        self.refresh_states()

    def refresh_states(self):
        # async fetch 'state' param on each visible camera row
        rows = self.current_rows()
        if not rows:
            return
        loop = asyncio.get_event_loop()
        for (name, ns, _) in rows:
            loop.create_task(self._update_one_state(name, ns))

    async def _update_one_state(self, name: str, ns: str):
        state = await self.ros.try_get_string_param(name, ns, "state")
        # Update cell if still present
        for r in range(self.table.rowCount()):
            if (self.table.item(r, CameraTable.COL_CAMERA).text() == name and
                self.table.item(r, CameraTable.COL_NAMESPACE).text() == ns):
                self.table.item(r, CameraTable.COL_STATE).setText(state if state else "unknown")
                break

    def _selected_or_warn(self) -> List[Tuple[str, str]]:
        sel = self.table.selected_nodes()
        if not sel:
            QtWidgets.QMessageBox.information(self, "No cameras selected", "Select one or more cameras in the left table.")
        return sel

    def on_configure(self):
        sel = self._selected_or_warn()
        if not sel:
            return
        params_map = self.right.get_camera_params()
        params = list(params_map.values())

        async def do():
            okmap = await self.ros.set_params_bulk(sel, params)
            failed = [f"{ns}/{n}" for (n, ns), ok in okmap.items() if not ok]
            if failed:
                QtWidgets.QMessageBox.warning(self, "Configure", "Some nodes failed:\n" + "\n".join(failed))
            else:
                QtWidgets.QMessageBox.information(self, "Configure", "Parameters applied.")

        asyncio.get_event_loop().create_task(do())

    def on_start(self):
        sel = self._selected_or_warn()
        if not sel:
            return
        mode_param = self.right.get_record_mode_param()
        start_param = Parameter("recording", Parameter.Type.BOOL, True)
        params = [mode_param, start_param]

        async def do():
            okmap = await self.ros.set_params_bulk(sel, params)
            failed = [f"{ns}/{n}" for (n, ns), ok in okmap.items() if not ok]
            if failed:
                QtWidgets.QMessageBox.warning(self, "Start", "Some nodes failed:\n" + "\n".join(failed))
            else:
                QtWidgets.QMessageBox.information(self, "Start", "Recording started (param-based).")

        asyncio.get_event_loop().create_task(do())

    def on_stop(self):
        sel = self._selected_or_warn()
        if not sel:
            return
        stop_param = Parameter("recording", Parameter.Type.BOOL, False)
        params = [stop_param]

        async def do():
            okmap = await self.ros.set_params_bulk(sel, params)
            failed = [f"{ns}/{n}" for (n, ns), ok in okmap.items() if not ok]
            if failed:
                QtWidgets.QMessageBox.warning(self, "Stop", "Some nodes failed:\n" + "\n".join(failed))
            else:
                QtWidgets.QMessageBox.information(self, "Stop", "Recording stopped.")

        asyncio.get_event_loop().create_task(do())


# --------------------------
# App/bootstrap
# --------------------------
def main():
    # ROS init
    rclpy.init(args=None)

    # Qt app
    app = QtWidgets.QApplication(sys.argv)

    # ROS node
    ros_node = CameraControlNode()

    # integrate ROS spin into Qt via timer
    def spin_once():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
    spin_timer = QtCore.QTimer()
    spin_timer.timeout.connect(spin_once)
    spin_timer.start(SPIN_INTERVAL_MS)

    # Asyncio integration (Qt’s event loop is already running; use default loop)
    if sys.platform.startswith("win"):
        # On Windows, ProactorEventLoop is default for Py3.8+. Usually fine.
        pass

    win = MainWindow(ros_node)
    win.show()

    ret = app.exec()

    # cleanup
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(ret)


if __name__ == "__main__":
    main()

