import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import tf_transformations

class StaticTFPublisher(Node):
    def __init__(self):
        super().__init__('static_tf_node')
        self.broadcaster = StaticTransformBroadcaster(self)
        self.publish_static_transforms()

    def publish_static_transforms(self):
        transforms = []

        # base_link → laser
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'base_link'
        t1.child_frame_id = 'laser'
        t1.transform.translation.x = 0.0
        t1.transform.translation.y = 0.0
        t1.transform.translation.z = 0.1
        q1 = tf_transformations.quaternion_from_euler(0, 0, 0)
        t1.transform.rotation.x = q1[0]
        t1.transform.rotation.y = q1[1]
        t1.transform.rotation.z = q1[2]
        t1.transform.rotation.w = q1[3]
        transforms.append(t1)

        # odom → base_link
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'odom'
        t2.child_frame_id = 'base_link'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        q2 = tf_transformations.quaternion_from_euler(0, 0, 0)
        t2.transform.rotation.x = q2[0]
        t2.transform.rotation.y = q2[1]
        t2.transform.rotation.z = q2[2]
        t2.transform.rotation.w = q2[3]
        transforms.append(t2)

        self.broadcaster.sendTransform(transforms)
        self.get_logger().info("Static transforms published with timestamp.")

def main():
    rclpy.init()
    node = StaticTFPublisher()
    rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
