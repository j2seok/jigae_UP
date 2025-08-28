import math
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

class TGTFilter(Node):
    def __init__(self):
        super().__init__('tgt_cmd_filter')

        # ---------------- 파라미터 ----------------
        self.map_frame   = self.declare_parameter('map_frame', 'map').get_parameter_value().string_value
        self.base_frame  = self.declare_parameter('base_frame', 'base_link').get_parameter_value().string_value
        self.plan_topic  = self.declare_parameter('plan_topic', '/plan').get_parameter_value().string_value

        self.cmd_in_topic  = self.declare_parameter('cmd_in', '/cmd_vel_raw').get_parameter_value().string_value  # 변동점
        self.cmd_out_topic = self.declare_parameter('cmd_out', '/cmd_vel').get_parameter_value().string_value      # 변동점

        self.lookahead     = self.declare_parameter('lookahead', 0.8).get_parameter_value().double_value          # 변동점
        self.enter_tol     = math.radians(self.declare_parameter('heading_tol_enter_deg', 8.0).get_parameter_value().double_value)  # 변동점
        self.exit_tol      = math.radians(self.declare_parameter('heading_tol_exit_deg', 4.0).get_parameter_value().double_value)    # 변동점
        self.min_hold_time = self.declare_parameter('min_hold_time', 0.4).get_parameter_value().double_value       # 변동점
        self.min_go_distance = self.declare_parameter('min_go_distance', 0.12).get_parameter_value().double_value  # 변동점
        self.heading_lpf_alpha = self.declare_parameter('heading_lpf_alpha', 0.2).get_parameter_value().double_value  # 변동점

        self.vx_cap = self.declare_parameter('vx_cap', 0.6).get_parameter_value().double_value
        self.wz_cap = self.declare_parameter('wz_cap', 1.5).get_parameter_value().double_value
        self.lin_deadband = self.declare_parameter('lin_deadband', 0.02).get_parameter_value().double_value
        self.ang_deadband = self.declare_parameter('ang_deadband', 0.02).get_parameter_value().double_value

        # 변동점: 횡방향(크로스-트랙) 오차 보호장치
        self.use_cross_track_guard = self.declare_parameter('use_cross_track_guard', True).get_parameter_value().bool_value
        self.cross_track_max_m = self.declare_parameter('cross_track_max_m', 0.30).get_parameter_value().double_value

        # ---------------- TF / 상태 ----------------
        self.tf_buf = Buffer()
        self.tf_lst = TransformListener(self.tf_buf, self)
        self.last_plan = None
        self.prev_desired_heading = None

        # 변동점: 상태기계
        self.mode = 'ROTATE'  # 'ROTATE' or 'GO'
        self.last_switch_t = self.get_clock().now()
        self.go_start_x = None
        self.go_start_y = None

        # ---------------- I/O ----------------
        self.sub_plan = self.create_subscription(Path, self.plan_topic, self.on_plan, 1)
        self.sub_cmd  = self.create_subscription(Twist, self.cmd_in_topic, self.on_cmd, 10)
        self.pub_cmd  = self.create_publisher(Twist, self.cmd_out_topic, 10)

        self.get_logger().info('TGT filter ready (turn–go–turn).')

    # ---------------- 콜백: 경로 ----------------
    def on_plan(self, msg: Path):
        self.last_plan = msg

    # ---------------- 유틸: 현재 포즈 ----------------
    def get_robot_pose(self):
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.base_frame,
                                              rclpy.time.Time(), Duration(seconds=0.1))
            x = tf.transform.translation.x
            y = tf.transform.translation.y
            q = tf.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            return x, y, yaw
        except Exception as e:
            self.get_logger().warn(f'TF lookup failed: {e}')
            return None

    # ---------------- 유틸: 경로 기반 목표 헤딩 ----------------
    def desired_heading_from_path(self, x, y):
        if not self.last_plan or len(self.last_plan.poses) < 2:
            return None

        # lookahead 만큼 누적 길이를 따라가며 목표점 찾기
        accum = 0.0
        prev = None
        for ps in self.last_plan.poses:
            px = ps.pose.position.x
            py = ps.pose.position.y
            if prev is None:
                prev = (px, py)
                continue
            seg = math.hypot(px - prev[0], py - prev[1])
            accum += seg
            if accum >= self.lookahead:
                dx, dy = (px - x), (py - y)
                if abs(dx) < 1e-6 and abs(dy) < 1e-6:
                    break
                des = math.atan2(dy, dx)
                # 변동점: 목표 헤딩 저역통과(래핑 고려)
                self.prev_desired_heading = self._ang_lpf(self.prev_desired_heading, des, self.heading_lpf_alpha)
                return self.prev_desired_heading
            prev = (px, py)

        # 경로가 짧으면 마지막 점 기준
        px = self.last_plan.poses[-1].pose.position.x
        py = self.last_plan.poses[-1].pose.position.y
        des = math.atan2(py - y, px - x)
        self.prev_desired_heading = self._ang_lpf(self.prev_desired_heading, des, self.heading_lpf_alpha)
        return self.prev_desired_heading

    # ---------------- 유틸: 각도 LPF ----------------
    def _ang_lpf(self, prev, new, alpha):
        if prev is None:
            return new
        err = math.atan2(math.sin(new - prev), math.cos(new - prev))
        return prev + alpha * err

    # ---------------- 유틸: 경로에 대한 횡오차 ----------------
    def cross_track_error(self, x, y):
        if not self.last_plan or len(self.last_plan.poses) < 2:
            return 0.0
        min_d = float('inf')
        pts = [(p.pose.position.x, p.pose.position.y) for p in self.last_plan.poses]
        for i in range(len(pts) - 1):
            x1, y1 = pts[i]; x2, y2 = pts[i+1]
            # 세그먼트 (x1,y1)-(x2,y2)와 점(x,y) 거리
            dx, dy = x2 - x1, y2 - y1
            if dx == 0.0 and dy == 0.0:
                d = math.hypot(x - x1, y - y1)
            else:
                t = ((x - x1)*dx + (y - y1)*dy) / (dx*dx + dy*dy)
                t = max(0.0, min(1.0, t))
                projx = x1 + t*dx; projy = y1 + t*dy
                d = math.hypot(x - projx, y - projy)
            if d < min_d:
                min_d = d
        return min_d

    # ---------------- 콜백: cmd_vel_raw ----------------
    def on_cmd(self, msg: Twist):
        # 기본 캡/데드밴드
        lin = max(min(msg.linear.x,  self.vx_cap), -self.vx_cap)
        ang = max(min(msg.angular.z, self.wz_cap), -self.wz_cap)
        if abs(lin) < self.lin_deadband: lin = 0.0
        if abs(ang) < self.ang_deadband: ang = 0.0

        pose = self.get_robot_pose()
        if pose is None or self.last_plan is None:
            out = Twist(); out.linear.x = lin; out.angular.z = ang
            self.pub_cmd.publish(out)
            return

        x, y, yaw = pose
        des = self.desired_heading_from_path(x, y)
        if des is None:
            out = Twist(); out.linear.x = lin; out.angular.z = ang
            self.pub_cmd.publish(out)
            return

        # 헤딩 오차
        err = math.atan2(math.sin(des - yaw), math.cos(des - yaw))
        now = self.get_clock().now()
        hold_ok = (now - self.last_switch_t).nanoseconds * 1e-9 >= self.min_hold_time

        # GO 모드 최소 전진거리
        def go_distance_ok():
            if self.go_start_x is None: return True
            return math.hypot(x - self.go_start_x, y - self.go_start_y) >= self.min_go_distance

        # 변동점: 횡오차 보호장치 (경로에서 너무 벗어나면 강제 ROTATE)
        if self.use_cross_track_guard:
            e_y = self.cross_track_error(x, y)
            if e_y > self.cross_track_max_m:
                self.mode = 'ROTATE'
                self.last_switch_t = now
                self.go_start_x, self.go_start_y = None, None

        # 변동점: 히스테리시스 상태기계
        if self.mode == 'ROTATE':
            if hold_ok and abs(err) < self.enter_tol:
                self.mode = 'GO'
                self.last_switch_t = now
                self.go_start_x, self.go_start_y = x, y
        else:  # GO
            if hold_ok and go_distance_ok() and abs(err) > self.exit_tol:
                self.mode = 'ROTATE'
                self.last_switch_t = now
                self.go_start_x, self.go_start_y = None, None

        # 변동점: 모드별 최종 출력
        out = Twist()
        if self.mode == 'ROTATE':
            out.linear.x = 0.0
            base_w = max(abs(ang), self.ang_deadband)
            out.angular.z = math.copysign(base_w, err)
        else:
            out.angular.z = 0.0
            out.linear.x = lin if lin != 0.0 else max(self.lin_deadband, 0.05)
        self.pub_cmd.publish(out)

def main():
    rclpy.init()
    node = TGTFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
