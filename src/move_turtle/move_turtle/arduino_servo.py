import rclpy
import serial
from rclpy.node import Node
from std_msgs.msg import String
from tf2_ros import Buffer, TransformBroadcaster, TransformListener
from geometry_msgs.msg import TransformStamped, Twist
import tf_transformations
import math

class Arduino_servo(Node):
    def __init__(self):
        super().__init__("arduino_servo")
        self.create_subscription(String, "servo", self.sub_callback, 10)
        self.ser = serial.Serial('/dev/ttyUSB0', 115200)
        self.tf_broadcaster = TransformBroadcaster(self)

    def sub_callback(self, msg: String):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "base_link"
        t.child_frame_id = "flag"
        t.transform.translation.x = 0.025  
        t.transform.translation.y = -0.05
        t.transform.translation.z = 0.14

        # msg.data : move180
        theta = int(msg.data[4:7])
        q = tf_transformations.quaternion_from_euler(0.0, -math.pi*theta/180, 0.0)
        self.get_logger().info(f"msg : {msg.data} q[1] : {q[1]}")
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

        byte_msg = (msg.data+'\n').encode('utf-8')
        self.ser.write(byte_msg)
        self.get_logger().info(msg.data)

def main():
    rclpy.init()
    node = Arduino_servo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.destroy_node()

if __name__ == "__main__":
    main()