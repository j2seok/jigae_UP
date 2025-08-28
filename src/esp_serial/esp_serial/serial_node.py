#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, threading, time, json

class ESPSerialNode(Node):
    def __init__(self):
        super().__init__('esp_serial_node')

        # params (변경 없으면 기본값 사용)
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value

        # 시리얼 오픈
        self.ser = serial.Serial(port, baud, timeout=1)

        # ROS pub/sub
        self.publisher_ = self.create_publisher(String, 'esp32/data', 10)
        self.subscription = self.create_subscription(
            String, 'esp32/write', self.write_callback, 10
        )

        # 수신/송신 스레드
        self._running = True
        self.read_thread = threading.Thread(target=self.read_serial, daemon=True)
        self.read_thread.start()
        self.write_thread = threading.Thread(target=self.write_loop, daemon=True)
        self.write_thread.start()

        self.get_logger().info(f"ESPSerialNode started. Listening on {port} @ {baud}")

    def read_serial(self):
        while rclpy.ok() and self._running:
            try:
                raw = self.ser.readline()  # '\n' 기준
                if not raw:
                    continue
                line = raw.decode('utf-8', errors='ignore').strip()
                if not line:
                    continue

                # 그대로 토픽으로 뿌림(디버깅/내부 연계용)
                self.publisher_.publish(String(data=line))
                self.get_logger().info(f"[RX] {line}")

                # (옵션) 다운링크 명령 자동 ACK 테스트
                # try:
                #     obj = json.loads(line)
                #     if isinstance(obj, dict) and 'command' in obj:
                #         ack = {"t":"ack","req_id":obj.get("req_id"),"ok":True,
                #                "note":"accepted","ts":int(time.time()*1000)}
                #         self.ser.write((json.dumps(ack, separators=(',',':'))+'\n').encode('utf-8'))
                #         self.get_logger().info(f"[TX] {ack}")
                # except Exception:
                #     pass

            except Exception as e:
                self.get_logger().error(f"Serial read error: {e}")
                time.sleep(0.2)

    def write_callback(self, msg: String):
        try:
            self.ser.write((msg.data + '\n').encode('utf-8'))
            self.get_logger().info(f"[TX] {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Serial write error: {e}")

    def write_loop(self):
        x, y, heading = 0.0, 0.0, 0.0
        while rclpy.ok() and self._running:
            try:
                now_ms = int(time.time() * 1000)
                dummy = {
                    "t": "telemetry",
                    "x": round(x, 3),
                    "y": round(y, 3),
                    "heading": round(heading % 360.0, 2),
                    "ts": now_ms
                }
                line = json.dumps(dummy, separators=(',', ':')) + '\n'
                self.ser.write(line.encode('utf-8'))
                self.get_logger().info(f"[TX] {line.strip()}")

                # 더미 궤적
                x += 0.1
                y += 0.05
                heading += 5.0

                time.sleep(1.0)  # 1Hz
            except Exception as e:
                self.get_logger().error(f"Serial auto-write error: {e}")
                time.sleep(1.0)

    def destroy_node(self):
        # 깨끗한 종료
        self._running = False
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ESPSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
