import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
import serial
import threading

class SerialSenderNode(Node):
    def __init__(self):
        super().__init__('serial_sender_node')
        self.declare_parameter('port', '/dev/ttyTHS1')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = int(self.get_parameter('baud').value)
        self.ser = serial.Serial(port, baud, timeout=0.02)

        self.sub = self.create_subscription(StringMsg, '/align_cmd', self.on_cmd, 10)
        self.pub_state = self.create_publisher(StringMsg, '/serial_sender/state', 10)

        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        self.get_logger().info(f'Serial open on {port} @ {baud}')

    def on_cmd(self, msg: StringMsg):
        cmd = msg.data.strip()
        if not cmd:
            return
        try:
            self.ser.write((cmd + '\n').encode('utf-8'))
            self.pub_state.publish(StringMsg(data=f'TX:{cmd}'))
        except Exception as e:
            self.pub_state.publish(StringMsg(data=f'ERR:{e}'))

    def _rx_loop(self):
        buf = b''
        while True:
            try:
                data = self.ser.read(64)
                if data:
                    buf += data
                    if b'\n' in buf:
                        line, buf = buf.split(b'\n', 1)
                        text = line.decode(errors='ignore').strip()
                        self.pub_state.publish(StringMsg(data=text.strip()))
                else:
                    rclpy.spin_once(self, timeout_sec=0.0)
            except Exception as e:
                self.pub_state.publish(StringMsg(data=f'ERR_RX:{e}'))
                break

def main():
    rclpy.init()
    node = SerialSenderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
