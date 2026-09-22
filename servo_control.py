#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import Jetson.GPIO as gpio

ros_qos = 10

class ServoTriggerNode(Node):
    def __init__(self):
        super().__init__('servo_trigger_node')
        print('start trigger servo')

        gpio.setmode(gpio.BOARD) #match pin name with header on hardware
        SERVO_0_PIN = 32
        SERVO_1_PIN = 33

        self.servo_0 = gpio.PWM(SERVO_0_PIN, 50)
        self.servo_1 = gpio.PWM(SERVO_1_PIN, 50)


        self.duty_cycle = 0.0
        self.zero_degree = 5.0

        self.servo_0.start(self.zero_degree)
        self.servo_1.start(self.zero_degree)

        #sub
        self.servo_trigger_sub = self.create_subscription(Bool,
                                                          '/mission/servo_trigger',
                                                          self.servo_trigger_callback,
                                                          ros_qos)


    def servo_trigger_callback(self, msg: Bool):
        if msg.data:
            self.set_servo_angle(0, 90)
            self.set_servo_angle(1, 90)
        else:
            self.set_servo_angle(0, 0)
            self.set_servo_angle(1, 0)            


    def angle_to_duty(self, angle):
        self.duty_cycle = self.zero_degree + (angle / 180.0) * self.zero_degree


    def set_servo_angle(self, servo, angle):
        self.angle_to_duty(angle)
        if servo == 0:
            self.servo_0.ChangeDutyCycle(self.duty_cycle)
        if servo == 1:
            self.servo_1.ChangeDutyCycle(self.duty_cycle)



def main(args = None):
    rclpy.init(args = args)
    node = ServoTriggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.servo_0.stop()
        node.servo_1.stop()
        gpio.cleanup()

        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()