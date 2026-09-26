#!/usr/bin/env python3

import threading
import time

import cv2
from diagnostic_msgs.msg import DiagnosticArray
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_srvs.srv import Trigger


WINDOW = "Stereo ArUco Calibration"
BUTTONS = {
    "start": (20, 198, 145, 246),
    "stop": (160, 198, 285, 246),
    "reset": (300, 198, 425, 246),
    "solve": (440, 198, 565, 246),
    "save": (580, 198, 705, 246),
}


class CalibrationGui(Node):
    def __init__(self):
        super().__init__("stereo_calibration_gui")
        self._lock = threading.Lock()
        self._preview = None
        self._status = {
            "state": "WAITING",
            "accepted_samples": "0",
            "minimum_samples": "10",
            "maximum_samples": "40",
            "last_decision": "waiting_for_status",
        }
        self._feedback = "Waiting for calibration backend..."
        # Node uses ``_clients`` internally to track ROS client entities. Keep
        # the GUI's name-to-client lookup separate or the executor will iterate
        # these dictionary keys as though they were client objects.
        self._service_clients = {
            name: self.create_client(Trigger, f"/stereo_calibration/{name}")
            for name in BUTTONS
        }
        self.create_subscription(
            CompressedImage,
            "/stereo_calibration/preview/compressed",
            self._preview_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            DiagnosticArray,
            "/stereo_calibration/status",
            self._status_callback,
            10,
        )

    def _preview_callback(self, message):
        data = np.frombuffer(message.data, dtype=np.uint8)
        image = cv2.imdecode(data, cv2.IMREAD_COLOR)
        if image is not None:
            with self._lock:
                self._preview = image

    def _status_callback(self, message):
        if not message.status:
            return
        values = {item.key: item.value for item in message.status[0].values}
        with self._lock:
            self._status = values

    def call(self, name):
        client = self._service_clients[name]
        if not client.service_is_ready():
            with self._lock:
                self._feedback = f"Service /stereo_calibration/{name} is unavailable"
            return
        future = client.call_async(Trigger.Request())

        def completed(result_future):
            try:
                response = result_future.result()
                text = ("OK: " if response.success else "FAILED: ") + response.message
            except Exception as exception:  # noqa: BLE001
                text = f"Service error: {exception}"
            with self._lock:
                self._feedback = text

        future.add_done_callback(completed)

    def snapshot(self):
        with self._lock:
            image = None if self._preview is None else self._preview.copy()
            return image, dict(self._status), self._feedback


def _float(status, key):
    try:
        value = float(status.get(key, "nan"))
        return value if np.isfinite(value) else None
    except ValueError:
        return None


def _bar(panel, label, value, y, color):
    value = 0.0 if value is None else max(0.0, min(1.0, value))
    cv2.putText(panel, label, (20, y + 17), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (230, 230, 230), 1, cv2.LINE_AA)
    x0, x1 = 150, 430
    cv2.rectangle(panel, (x0, y), (x1, y + 20), (85, 85, 85), 1)
    cv2.rectangle(panel, (x0 + 1, y + 1),
                  (x0 + 1 + int((x1 - x0 - 2) * value), y + 19), color, -1)
    cv2.putText(panel, f"{value * 100:3.0f}%", (440, y + 17),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1, cv2.LINE_AA)


def _button(panel, name, enabled):
    x0, y0, x1, y1 = BUTTONS[name]
    color = (55, 130, 55) if enabled else (70, 70, 70)
    cv2.rectangle(panel, (x0, y0), (x1, y1), color, -1)
    cv2.rectangle(panel, (x0, y0), (x1, y1), (185, 185, 185), 1)
    label = name.upper()
    size = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0]
    cv2.putText(panel, label, ((x0 + x1 - size[0]) // 2, y0 + 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                (245, 245, 245) if enabled else (150, 150, 150), 1, cv2.LINE_AA)


def draw(preview, status, feedback):
    if preview is None:
        preview = np.full((540, 1280, 3), 25, dtype=np.uint8)
        cv2.putText(preview, "Waiting for synchronized preview...", (80, 270),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (180, 180, 180), 2, cv2.LINE_AA)
    width = max(preview.shape[1], 760)
    if preview.shape[1] != width:
        canvas = np.zeros((preview.shape[0], width, 3), dtype=np.uint8)
        canvas[:, :preview.shape[1]] = preview
        preview = canvas
    panel = np.full((265, width, 3), 32, dtype=np.uint8)

    state = status.get("state", "WAITING")
    samples = status.get("accepted_samples", "0")
    minimum = status.get("minimum_samples", "10")
    maximum = status.get("maximum_samples", "40")
    common = status.get("common_marker_count", "0")
    left_count = status.get("left_marker_count", "0")
    right_count = status.get("right_marker_count", "0")
    delta = status.get("pair_delta_ms", "nan")
    left_error = status.get("left_reprojection_error_px", "nan")
    right_error = status.get("right_reprojection_error_px", "nan")
    decision = status.get("last_decision", "waiting")
    sampling_state = status.get("sampling_state", "WAITING")
    stability = _float(status, "stability_progress")

    state_color = (80, 220, 80) if state in {"CAPTURING", "SOLVED"} else (0, 190, 255)
    cv2.putText(panel, f"State: {state}", (20, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.75, state_color, 2, cv2.LINE_AA)
    cv2.putText(panel, f"Samples: {samples}/{maximum}  (solve >= {minimum})",
                (260, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.62, (235, 235, 235), 1, cv2.LINE_AA)
    cv2.putText(panel,
                f"Markers L/R/common: {left_count}/{right_count}/{common}    pair dt: {delta} ms",
                (20, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(panel, f"PnP error L/R: {left_error}/{right_error} px",
                (20, 84), cv2.FONT_HERSHEY_SIMPLEX, 0.57, (220, 220, 220), 1, cv2.LINE_AA)

    _bar(panel, "Samples", _float(status, "sample_progress"), 96, (70, 180, 70))
    _bar(panel, "Position", _float(status, "position_coverage"), 120, (180, 135, 40))
    _bar(panel, "Distance", _float(status, "distance_coverage"), 144, (180, 135, 40))
    _bar(panel, "Rotation", _float(status, "rotation_coverage"), 168, (180, 135, 40))

    can_solve = status.get("can_solve", "false") == "true"
    can_save = status.get("can_save", "false") == "true"
    for name in BUTTONS:
        enabled = name not in {"solve", "save"} or (can_solve if name == "solve" else can_save)
        _button(panel, name, enabled)

    stability_percent = 0.0 if stability is None else stability * 100.0
    cv2.putText(panel, f"Sampling: {sampling_state}  {stability_percent:.0f}%", (730, 84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (210, 210, 210), 1, cv2.LINE_AA)
    cv2.putText(panel, f"Decision: {decision}", (730, 112), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 210, 255), 1, cv2.LINE_AA)
    if "stereo_rms_px" in status:
        cv2.putText(panel,
                    f"RMS {status['stereo_rms_px']} px   epi mean/max "
                    f"{status.get('mean_epipolar_error_px', '?')}/"
                    f"{status.get('max_epipolar_error_px', '?')} px",
                    (730, 142), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (100, 230, 100), 1, cv2.LINE_AA)
        cv2.putText(panel, f"Baseline: {status.get('baseline_m', '?')} m",
                    (730, 170), cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                    (100, 230, 100), 1, cv2.LINE_AA)
    cv2.putText(panel, feedback[:100], (730, 218), cv2.FONT_HERSHEY_SIMPLEX,
                0.48, (210, 210, 210), 1, cv2.LINE_AA)
    cv2.putText(panel, "Keys: SPACE start/stop | R reset | C solve | S save | Q quit",
                (20, 260), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (155, 155, 155), 1, cv2.LINE_AA)
    return np.vstack((preview, panel))


def main():
    rclpy.init()
    node = CalibrationGui()
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    cv2.namedWindow(WINDOW, cv2.WINDOW_AUTOSIZE)

    def mouse(event, x, y, _flags, _parameter):
        if event != cv2.EVENT_LBUTTONUP:
            return
        preview, status, _ = node.snapshot()
        preview_height = 540 if preview is None else preview.shape[0]
        panel_y = y - preview_height
        for name, (x0, y0, x1, y1) in BUTTONS.items():
            if x0 <= x <= x1 and y0 <= panel_y <= y1:
                if name == "solve" and status.get("can_solve", "false") != "true":
                    return
                if name == "save" and status.get("can_save", "false") != "true":
                    return
                node.call(name)
                return

    cv2.setMouseCallback(WINDOW, mouse)
    try:
        while rclpy.ok():
            preview, status, feedback = node.snapshot()
            cv2.imshow(WINDOW, draw(preview, status, feedback))
            key = cv2.waitKey(30) & 0xFF
            if key in (ord("q"), 27):
                break
            if key == ord("r"):
                node.call("reset")
            elif key == ord("c"):
                node.call("solve")
            elif key == ord("s"):
                node.call("save")
            elif key == ord(" "):
                node.call("stop" if status.get("state") == "CAPTURING" else "start")
            time.sleep(0.01)
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

