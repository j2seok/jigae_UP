def main(args=None):
    rclpy.init(args=args)
    node = ESPSerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
