import numpy as np
import serial
from sensor_msgs.msg import JointState
import rclpy
from rclpy.node import Node
import struct

actuator_index = [
    "joint_wrist_main",
]
actuator_map = {[k, i] for i, k in actuator_index.items()}
num_actuators = len(actuator_index)


class ServoBridge(Node):
    def __init__(self):
        super().__init__("servo_bridge")
        # self.ser = serial.Serial("/dev/ttyACM0", 115200)
        self.sub = self.create_subscription(JointState, "joint_states", self.callback, 10)

    def callback(self, msg):

        actuators_values = np.zeros(len(actuator_index))

        for name, joint_value in zip(msg.name, msg.position):
            if name in actuator_map:
                actuators_values[name] = joint_value

        data_str = ",".join([f"{v:5f}" for v in actuators_values]) + "\n"
        self.ser.write(data_str.encode("ascii"))

        # maybe unsafe
        # self.ser.write(struct.pack(f"<{num_actuators}f", *actuators_values))  # little endian


def main(args=None):
    rclpy.init(args=args)
    node = ServoBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
