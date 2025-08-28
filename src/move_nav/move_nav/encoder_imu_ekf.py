import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion, TransformStamped, Twist
import serial
import math
import time
from tf2_ros import TransformBroadcaster
import json  # 변동점: JSON 전송을 위한 import

class SerialIMUEncoderNode(Node):
    def __init__(self):
        super().__init__('serial_imu_encoder_node')
        self.ser = serial.Serial('/dev/ttyTHS1', 115200, timeout=1)

        self.tf_broadcaster = TransformBroadcaster(self)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.timer = self.create_timer(0.05, self.timer_callback)  # 20Hz

        # 누적 변수
        self.x = 0.0
        self.y = 0.0
        self.theta_imu = 0.0
        self.theta_enc = 0.0
        self.theta = 0.0
        self.a = 0

        # 거리 및 회전 계산용
        self.last_l = 0
        self.last_r = 0
        self.prev_time = time.time()
        self.WHEEL_CIRCUM_CM = 13.0
        self.COUNTS_PER_REV = 1465.84
        self.TICK_TO_CM = self.WHEEL_CIRCUM_CM / self.COUNTS_PER_REV
        self.WHEEL_BASE = 0.375  # 바퀴 간격 (m)

        # 자이로 바이어스 보정
        self.gyro_bias_z = 0.0
        self.bias_initialized = False
        self.bias_sample_count = 0
        self.bias_sample_target = 200
        self.bias_sample_sum = 0.0

        # 변동점: cmd_vel 수신 + 아두이노로 전송 준비
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.get_logger().info("[CMD_VEL] Subscriber registered on /cmd_vel")
        self.last_cmd_time = time.time()          # 변동점: 워치독용
        self.cmd_timeout = 0.6                    # 변동점: s, 일정 시간 cmd_vel 없으면 정지 명령 전송
        self.sent_stop_once = False               # 변동점: 중복 전송 방지 플래그

    # 변동점: 상단에 이미 import json 되어 있으면 그대로 사용
    # from geometry_msgs.msg import Twist ... (생략)

    def cmd_vel_callback(self, msg: Twist):
        vx = float(msg.linear.x)
        wz = float(msg.angular.z)
        self.get_logger().info(f"[CMD_VEL] linear.x={vx:.3f} m/s, angular.z={wz:.3f} rad/s")
        # 변동점: JSON 라인으로 전송 (아두이노와 포맷 일치)
        payload = {"vx": vx, "wz": wz}
        line = json.dumps(payload) + "\n"
        try:
            self.ser.write(line.encode('ascii'))
        except Exception as e:
            self.get_logger().error(f"[SERIAL WRITE ERR] {e}")
        self.last_cmd_time = time.time()
        self.sent_stop_once = False


    def timer_callback(self):
        # 변동점: 타임아웃 시 정지 JSON 전송
        if (time.time() - self.last_cmd_time) > self.cmd_timeout and not self.sent_stop_once:
            try:
                self.ser.write(b'{"vx":0,"wz":0}\n')
                self.get_logger().debug("[WATCHDOG] sent stop JSON")
            except Exception as e:
                self.get_logger().warn(f"[WATCHDOG WRITE ERR] {e}")
            self.sent_stop_once = True


        # 변동점: 블로킹 방지 – 수신 버퍼 없으면 바로 리턴 (readline() 대기 방지)
        if self.ser.in_waiting == 0:
            return

        line = self.ser.readline().decode(errors="ignore").strip()
        if not line:
            return

        try:
            # 변동점: '&'는 비트연산자 → 논리연산자 'and' 로 수정 (버그 가능성)
            if (self.bias_initialized == True and self.a == 0):
                print(f"[RAW] {line}", flush=True)
                self.a = 1

            enc_part, imu_part = line.split(';')
            enc_fields = enc_part.replace('ENC:', '').split(',')
            if len(enc_fields) == 4:
                l1, r1, l2, r2 = map(int, enc_fields)
            else:
                self.get_logger().warn(f"엔코더 데이터 이상: {enc_fields}")
                return

            imu_fields = imu_part.replace('IMU:', '').split(',')
            if len(imu_fields) == 9:
                ax, ay, az, gx, gy, gz, mx, my, mz = map(float, imu_fields)
            else:
                self.get_logger().warn(f"IMU 데이터 이상: {imu_fields}")
                return

            # 바이어스 수집 단계
            if not self.bias_initialized:
                self.bias_sample_sum += gz
                self.bias_sample_count += 1
                if self.bias_sample_count >= self.bias_sample_target:
                    self.gyro_bias_z = self.bias_sample_sum / self.bias_sample_target
                    self.bias_initialized = True
                    self.get_logger().info(f"[GYRO BIAS INIT] gyro_bias_z={self.gyro_bias_z:.4f} deg/s")
                return

            now = time.time()
            dt = now - self.prev_time
            self.prev_time = now

            gz_corr_deg = gz - self.gyro_bias_z
            gz_corr_rad = math.radians(gz_corr_deg)
            self.theta_imu += gz_corr_rad * dt

            delta_l = (l1 - self.last_l) * self.TICK_TO_CM * 0.01
            delta_r = (r1 - self.last_r) * self.TICK_TO_CM * 0.01
            self.last_l = l1
            self.last_r = r1
            delta_theta_enc = (delta_r - delta_l) / self.WHEEL_BASE
            self.theta_enc += delta_theta_enc

            theta_fused = self.theta_imu
            self.theta = -1 * (math.atan2(math.sin(theta_fused), math.cos(theta_fused)))

            d = (delta_l + delta_r) / 2.0
            self.x += d * math.cos(self.theta)
            self.y += d * math.sin(self.theta)

            imu_msg = Imu()
            stamp = self.get_clock().now().to_msg()
            imu_msg.header.stamp = stamp
            imu_msg.header.frame_id = 'base_link'
            imu_msg.linear_acceleration.x = ax
            imu_msg.linear_acceleration.y = ay
            imu_msg.linear_acceleration.z = az
            imu_msg.angular_velocity.x = math.radians(gx)
            imu_msg.angular_velocity.y = math.radians(gy)
            imu_msg.angular_velocity.z = gz_corr_rad
            imu_msg.orientation = self.euler_to_quaternion(0.0, 0.0, self.theta)
            self.imu_pub.publish(imu_msg)

            odom_msg = Odometry()
            odom_msg.header.stamp = stamp
            odom_msg.header.frame_id = "odom"
            odom_msg.child_frame_id = "base_link"
            odom_msg.pose.pose.position.x = self.x
            odom_msg.pose.pose.position.y = self.y
            odom_msg.pose.pose.position.z = 0.0
            odom_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, self.theta)
            self.odom_pub.publish(odom_msg)

            t = TransformStamped()
            t.header.stamp = stamp
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            q = self._quat_from_euler(0.0 , 0.0, self.theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]
            self.tf_broadcaster.sendTransform(t)

        except Exception as e:
            self.get_logger().error(f"[Parsing Error] {e}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx, qy, qz, qw = self._quat_from_euler(roll, pitch, yaw)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def _quat_from_euler(self, roll, pitch, yaw):
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        return (qx, qy, qz, qw)

def main(args=None):
    rclpy.init(args=args)
    node = SerialIMUEncoderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
