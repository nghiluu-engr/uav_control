#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import serial

ros_qos = 10

class ServoTriggerNode(Node):
    def __init__(self):
        super().__init__('servo_trigger_node')
        print('start trigger servo')

        self.servo = serial.Serial(
            '/dev/ttyUSB1',
            115200,
            timeout=1
        )

        #sub
        self.servo_trigger_sub = self.create_subscription(Bool,
                                                          '/mission/servo_trigger',
                                                          self.servo_trigger_callback,
                                                          ros_qos)

        #flag
        self.servo_trigger_requested = False


    def servo_trigger_callback(self, msg: Bool):
        if msg.data and not self.servo_trigger_requested:
            self.servo_trigger_requested = msg.data
            self.servo.write(b'1\n')
            self.servo.flush()
            print('send 1')
        else:
            self.servo_trigger_requested = False
            self.servo.write(b'0\n')
            self.servo.flush()
            print('send 0')


def main(args = None):
    rclpy.init(args = args)
    node = ServoTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.servo.close()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()