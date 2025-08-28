#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge
import numpy as np
from collections import deque

class ArucoDetectorNode(Node):
    def __init__(self):
        super().__init__('aruco_detector_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')

        self.declare_parameter('marker_length_m', 0.038)
        self.declare_parameter('id_left', 13)
        self.declare_parameter('id_right', 14)

        self.declare_parameter('x_thresh_cm', 5.0)
        self.declare_parameter('theta_thresh_deg', 5.0)
        self.declare_parameter('x_close_cm', 1.0)
        self.declare_parameter('theta_close_deg', 2.0)
        self.declare_parameter('target_theta_deg', 0.0)
        self.declare_parameter('camera_x_offset_cm', 4.0)

        self.declare_parameter('min_cmd_interval', 0.5)

        self.declare_parameter('target_z_cm', 49.0)
        self.declare_parameter('z_close_cm', 1.0)
        self.declare_parameter('approach_k', 0.6)
        self.declare_parameter('min_step_cm', 3.0)
        self.declare_parameter('max_step_cm', 12.0)
        self.declare_parameter('max_total_cm', 120.0)

        self.declare_parameter('insert_total_cm', 17.0)
        self.declare_parameter('insert_step_cm', 5.0)
        self.declare_parameter('lift_small_cm', 3.0)
        self.declare_parameter('retract_extra_cm', 0.0)

        self.declare_parameter('reacquire_miss_n', 3)
        self.declare_parameter('reacquire_hit_n', 2)

        self.declare_parameter('cmd_timeout_s', 6.0)
        self.declare_parameter('max_cmd_retries', 1)

        gp = self.get_parameter
        self.image_topic         = str(gp('image_topic').value)
        self.camera_info_topic   = str(gp('camera_info_topic').value)
        self.marker_length_m     = float(gp('marker_length_m').value)
        self.id_left             = int(gp('id_left').value)
        self.id_right            = int(gp('id_right').value)

        self.x_thresh_cm         = float(gp('x_thresh_cm').value)
        self.theta_thresh_deg    = float(gp('theta_thresh_deg').value)
        self.x_close_cm          = float(gp('x_close_cm').value)
        self.theta_close_deg     = float(gp('theta_close_deg').value)
        self.target_theta_deg    = float(gp('target_theta_deg').value)
        self.camera_x_offset_cm  = float(gp('camera_x_offset_cm').value)

        self.min_cmd_interval    = float(gp('min_cmd_interval').value)

        self.target_z_cm         = float(gp('target_z_cm').value)
        self.z_close_cm          = float(gp('z_close_cm').value)
        self.approach_k          = float(gp('approach_k').value)
        self.min_step_cm         = float(gp('min_step_cm').value)
        self.max_step_cm         = float(gp('max_step_cm').value)
        self.max_total_cm        = float(gp('max_total_cm').value)

        self.insert_total_cm     = float(gp('insert_total_cm').value)
        self.insert_step_cm      = float(gp('insert_step_cm').value)
        self.lift_small_cm       = float(gp('lift_small_cm').value)
        self.retract_extra_cm    = float(gp('retract_extra_cm').value)

        self.reacquire_miss_n    = int(gp('reacquire_miss_n').value)
        self.reacquire_hit_n     = int(gp('reacquire_hit_n').value)

        self.cmd_timeout_s       = float(gp('cmd_timeout_s').value)
        self.max_cmd_retries     = int(gp('max_cmd_retries').value)

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.history = HistoryPolicy.KEEP_LAST

        self.sub_cam  = self.create_subscription(Image, self.image_topic, self.on_image, qos)
        self.sub_info = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_info, 10)

        self.pub_status = self.create_publisher(String, '/align_status', 10)
        self.pub_error  = self.create_publisher(Vector3, '/align_error', 10)
        self.pub_cmd    = self.create_publisher(String, '/align_cmd', 10)
        self.pub_cycle_done = self.create_publisher(String, '/align_cycle_done', 10)

        self.sub_serial_state = self.create_subscription(String, '/serial_sender/state', self.on_serial_state, 10)

        self.state = 'IDLE'
        self.last_cmd = ''
        self.last_cmd_time = self.get_clock().now() - Duration(seconds=self.min_cmd_interval)

        self.last_x = None
        self.last_theta = None
        self.last_z = None

        self.z_hist = deque(maxlen=5)

        self.total_approach_cm = 0.0
        self.total_insert_cm   = 0.0

        self.pending_cmd   = None
        self.pending_since = None
        self.retry_count   = 0

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        if hasattr(cv2.aruco, 'DetectorParameters'):
            self.aruco_params = cv2.aruco.DetectorParameters()
            self.use_new_api = True
        else:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
            self.use_new_api = False

        if self.use_new_api and hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        else:
            self.detector = None

        self.miss_frames = 0
        self.hit_frames  = 0

        self.get_logger().info(
            f'ArucoDetectorNode ready | image={self.image_topic}, info={self.camera_info_topic}'
        )

    def on_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float32).reshape(3, 3)
            self.dist_coeffs   = np.array(msg.d, dtype=np.float32)
            self.get_logger().info('CameraInfo received')

    def _clear_timeout(self):
        self.pending_cmd = None
        self.pending_since = None
        self.retry_count = 0
        self.last_cmd = ''

    def on_serial_state(self, msg: String):
        txt = msg.data.strip()
        if txt.startswith('RX:'):
            txt = txt[3:].strip()

        if txt in ('rotate_done', 'translate_done', 'forward_done',
                   'insert_done', 'lift_done', 'backward_done'):
            self._clear_timeout()

        if txt == 'rotate_done':
            self.get_logger().info('[FSM] rotate_done → WAIT_X')
            self.publish_state('WAIT_X')
            return

        if txt == 'translate_done':
            self.get_logger().info('[FSM] translate_done → IDLE')
            self.publish_state('IDLE')
            return

        if txt == 'forward_done':
            self.get_logger().info('[FSM] forward_done → APPROACH')
            self.publish_state('APPROACH')
            return

        if txt == 'insert_done':
            self._proceed_insert_or_lift()
            return

        if txt == 'lift_done':
            self.publish_state('RETRACT')
            total_forward = self.total_approach_cm + self.total_insert_cm + self.retract_extra_cm
            self.send_cmd(f'backward_cm:{total_forward:.2f}')
            self._mark_cmd_time()
            return

        if txt == 'backward_done':
            self.get_logger().info('[FSM] backward_done → DONE')
            self.total_approach_cm = 0.0
            self.total_insert_cm   = 0.0
            self._clear_timeout()
            self.publish_state('DONE')
            self.pub_cycle_done.publish(String(data='align_cycle_done'))
            return

        if txt.startswith('ERR'):
            self.get_logger().warn(f'[SERIAL]{txt}')
            self.publish_state('HOLD')

    def _check_timeout(self):
        if not self.pending_cmd or not self.pending_since:
            return

        dt = (self.get_clock().now() - self.pending_since).nanoseconds * 1e-9
        if dt < self.cmd_timeout_s:
            return

        self.get_logger().warn(f'[TIMEOUT] {self.pending_cmd} (>{self.cmd_timeout_s:.1f}s)')

        if self.retry_count < self.max_cmd_retries:
            self.retry_count += 1
            self.get_logger().warn(f'[RETRY {self.retry_count}/{self.max_cmd_retries}] {self.pending_cmd}')
            self.send_cmd(self.pending_cmd, force=True)
            self._mark_cmd_time()
            return

        self.get_logger().error(f'[ABORT] {self.pending_cmd} no ack → rollback to IDLE')
        self._clear_timeout()
        self.last_cmd_time = self.get_clock().now() - Duration(seconds=self.min_cmd_interval)
        self.publish_state('IDLE')

    def on_image(self, msg: Image):
        if self.camera_matrix is None:
            return

        self._check_timeout()

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        if self.detector is not None:
            corners, ids, _ = self.detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.aruco_params)

        ids_list = [] if ids is None else ids.flatten().tolist()
        ids_set  = set(ids_list)
        needed   = {self.id_left, self.id_right}
        have_both = needed.issubset(ids_set)

        vision_required_states = ('IDLE', 'APPROACH', 'ALIGN_THETA', 'ALIGN_X', 'WAIT_X', 'REACQUIRE')
        if not have_both and self.state in vision_required_states:
            self.miss_frames += 1; self.hit_frames = 0
            if self.miss_frames >= self.reacquire_miss_n and self.state in ('IDLE', 'APPROACH'):
                if self.state != 'REACQUIRE':
                    self.get_logger().warn(f"[FSM] miss {self.miss_frames} → REACQUIRE")
                    self.publish_state('REACQUIRE')
            return
        else:
            if have_both:
                self.hit_frames += 1; self.miss_frames = 0
                if self.state == 'REACQUIRE' and self.hit_frames >= self.reacquire_hit_n:
                    self.get_logger().info(f"[FSM] hit {self.hit_frames} → IDLE")
                    self.publish_state('IDLE')

        if self.state in ('INSERT', 'LIFT', 'RETRACT', 'DONE', 'HOLD'):
            return

        if not have_both:
            return

        rvecs, tvecs, found_ids = [], [], []
        for c, i in zip(corners, ids_list):
            if i in needed:
                rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
                    c, self.marker_length_m, self.camera_matrix, self.dist_coeffs
                )
                rvecs.append(rvec[0][0]); tvecs.append(tvec[0][0]); found_ids.append(i)
        if len(found_ids) < 2:
            return

        if found_ids[0] == self.id_left:
            t_left, r_left  = tvecs[0], rvecs[0]
            t_right, r_right = tvecs[1], rvecs[1]
        else:
            t_right, r_right = tvecs[0], rvecs[0]
            t_left,  r_left  = tvecs[1], rvecs[1]
        if float(t_left[0]) > float(t_right[0]):
            t_left, t_right = t_right, t_left
            r_left, r_right = r_right, r_left

        zL, zR = float(t_left[2]), float(t_right[2])
        w1 = 1.0 / zL if zL != 0 else 0.0
        w2 = 1.0 / zR if zR != 0 else 0.0
        t_mid = (w1 * t_left + w2 * t_right) / (w1 + w2)

        x_cm = float(t_mid[0] * 100.0) - self.camera_x_offset_cm
        z_cm = float(t_mid[2] * 100.0)

        theta_deg = self._compute_theta_from_two_markers_deg(t_left, t_right)
        theta_err = self._normalize_angle_deg(theta_deg - self.target_theta_deg)

        err_msg = Vector3(); err_msg.x = x_cm; err_msg.y = 0.0; err_msg.z = z_cm
        self.pub_error.publish(err_msg)

        self.last_x = x_cm; self.last_theta = theta_err
        self.z_hist.append(z_cm)
        z_smooth = float(np.median(self.z_hist))
        self.last_z = z_smooth

        if self.state == 'WAIT_X':
            if abs(self.last_x) > self.x_thresh_cm:
                self.publish_state('ALIGN_X')
                self.send_cmd(f'align_x:{self.last_x:.2f}')
                self._mark_cmd_time()
            else:
                self.publish_state('IDLE')
            return

        if self.state not in ('IDLE', 'APPROACH'):
            return
        if self.last_cmd and self._too_soon():
            return

        if abs(theta_err) > self.theta_thresh_deg:
            self.publish_state('ALIGN_THETA')
            self.send_cmd(f'align_theta:{theta_err:.2f}')
            self._mark_cmd_time()
            return

        if abs(x_cm) > self.x_thresh_cm:
            self.publish_state('ALIGN_X')
            self.send_cmd(f'align_x:{x_cm:.2f}')
            self._mark_cmd_time()
            return

        if abs(theta_err) <= self.theta_close_deg and abs(x_cm) <= self.x_close_cm:
            if abs(z_smooth - self.target_z_cm) <= self.z_close_cm:
                self.get_logger().info('[FSM] z target reached → start INSERT')
                self.total_insert_cm = 0.0
                self.publish_state('INSERT')
                self._proceed_insert_or_lift()
                return

            dz = z_smooth - self.target_z_cm
            step = self._clamp(self.approach_k * abs(dz), self.min_step_cm, self.max_step_cm)

            if self.total_approach_cm + step > self.max_total_cm:
                self.get_logger().warn('[APPROACH] max_total_cm 초과 → HOLD')
                self.publish_state('HOLD')
                return

            self.publish_state('APPROACH')
            self.send_cmd(f'forward_cm:{step:.2f}')
            self.total_approach_cm += step
            self._mark_cmd_time()
            return

        self.publish_state('HOLD')

    def _proceed_insert_or_lift(self):
        if self.total_insert_cm >= self.insert_total_cm - 1e-3:
            self.publish_state('LIFT')
            self.send_cmd(f'lift_up_cm:{self.lift_small_cm:.2f}')
            self._mark_cmd_time()
        else:
            step = min(self.insert_step_cm, self.insert_total_cm - self.total_insert_cm)
            self.publish_state('INSERT')
            self.send_cmd(f'insert_cm:{step:.2f}')
            self.total_insert_cm += step
            self._mark_cmd_time()

    def _too_soon(self):
        return (self.get_clock().now() - self.last_cmd_time).nanoseconds * 1e-9 < self.min_cmd_interval

    def _mark_cmd_time(self):
        self.last_cmd_time = self.get_clock().now()

    def _compute_theta_from_two_markers_deg(self, p1, p2):
        dx = float(p2[0] - p1[0]); dz = float(p2[2] - p1[2])
        theta_deg = float(np.degrees(np.arctan2(dz, dx)))
        return self._normalize_angle_deg(theta_deg)

    def _normalize_angle_deg(self, angle):
        if angle > 180.0: angle -= 360.0
        elif angle < -180.0: angle += 360.0
        return angle

    def publish_state(self, state: str):
        if self.state != state:
            self.state = state
            self.pub_status.publish(String(data=self.state))

    def send_cmd(self, cmd: str, *, force: bool = False):
        if not force and cmd == self.last_cmd:
            return
        self.last_cmd = cmd
        self.pub_cmd.publish(String(data=cmd))
        self.get_logger().info(f'[CMD] {cmd}')
        self.pending_cmd = cmd
        self.pending_since = self.get_clock().now()
        self.retry_count = 0

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))


def main():
    rclpy.init()
    node = ArucoDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
