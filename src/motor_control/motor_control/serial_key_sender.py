import rclpy
from rclpy.node import Node
import serial

class RosSerialCommand(Node):
    def __init__(self):
        super().__init__('ros_serial_command')
        self.ser = serial.Serial('/dev/ttyTHS1', 115200)
        self.get_logger().info("시리얼 연결됨 /dev/ttyTHS1")

        self.run_input_loop()

    def run_input_loop(self):
        while rclpy.ok():
            cmd = input("명령어 입력 (qweasd... or f10/l90/255 등): ").strip()
            if cmd == "": continue
            self.ser.write(cmd.encode())
            self.ser.write(b'\n')  # 아두이노에서 readStringUntil('\n')로 읽을 수 있게
            self.get_logger().info(f"보낸 명령어: {cmd}")

def main(args=None):
    rclpy.init(args=args)
    node = RosSerialCommand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.ser.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
