import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray


class EncoderAnglesSim(Node):
    def __init__(self) -> None:
        super().__init__("encoder_angles_sim")
        self.declare_parameter("left_joint_name", "wheel_left_joint")
        self.declare_parameter("right_joint_name", "wheel_right_joint")
        self.left_joint_name = self.get_parameter("left_joint_name").get_parameter_value().string_value
        self.right_joint_name = self.get_parameter("right_joint_name").get_parameter_value().string_value

        self.publisher = self.create_publisher(Float32MultiArray, "/encoders/angles", 10)
        self.subscription = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_state_callback,
            10,
        )

    @staticmethod
    def wrap_degrees(angle_rad: float) -> float:
        angle_deg = math.degrees(angle_rad)
        return ((angle_deg + 180.0) % 360.0) - 180.0

    def joint_state_callback(self, msg: JointState) -> None:
        try:
            left_index = msg.name.index(self.left_joint_name)
            right_index = msg.name.index(self.right_joint_name)
        except ValueError:
            return

        if len(msg.position) <= max(left_index, right_index):
            return

        encoder_msg = Float32MultiArray()
        encoder_msg.data = [
            self.wrap_degrees(msg.position[left_index]),
            self.wrap_degrees(msg.position[right_index]),
        ]
        self.publisher.publish(encoder_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EncoderAnglesSim()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
