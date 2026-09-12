#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import (VehicleStatus, VehicleCommand, 
                          OffboardControlMode, VehicleLocalPosition, TrajectorySetpoint)
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool

px4_qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                         durability = DurabilityPolicy.TRANSIENT_LOCAL,
                         history = HistoryPolicy.KEEP_LAST,
                         depth = 1)

ros_qos = 10


class OffboardControlNode(Node):
    def __init__(self):
        super().__init__('offboard_control_node')

        #parameter
        self.declare_parameter("default_target_x", 0.0)
        self.declare_parameter("default_target_y", 0.0)
        self.declare_parameter("default_target_z", -5.0)

        #value
        self.default_target_x = float(self.get_parameter('default_target_x').value)
        self.default_target_y = float(self.get_parameter('default_target_y').value)
        self.default_target_z = float(self.get_parameter('default_target_z').value)

        self.target_x = self.default_target_x
        self.target_y = self.default_target_y
        self.target_z = self.default_target_z

        self.local_x = 0
        self.local_y = 0
        self.local_z = 0

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.vehicle_nav_state = None

        #timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    #pub

        #position
        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode,
                                                               '/fmu/in/offboard_control_mode',
                                                               px4_qos)

        self.target_position_pub = self.create_publisher(TrajectorySetpoint,
                                                        '/fmu/in/trajectory_setpoint',
                                                        px4_qos)

        self.current_position_pub = self.create_publisher(PoseStamped,
                                                          '/mission/current_position',
                                                          ros_qos)

        #velocity
        self.current_velocity_pub = self.create_publisher(TwistStamped,
                                                          '/mission/current_velocity',
                                                          ros_qos)

        #vehicle command
        self.vehicle_command_pub = self.create_publisher(VehicleCommand,
                                                         '/fmu/in/vehicle_command',
                                                         px4_qos)





    #sub

        #position
        self.target_position_sub = self.create_subscription(PoseStamped,
                                                            '/mission/target_position',
                                                            self.target_position_callback,
                                                            ros_qos)

        self.local_position_sub = self.create_subscription(VehicleLocalPosition,
                                                           '/fmu/out/vehicle_local_position_v1',
                                                           self.local_position_callback,
                                                           px4_qos)

        #velocity
        self.target_velocity_sub = self.create_subscription(TwistStamped,
                                                            '/mission/target_velocity',
                                                            self.target_velocity_callback,
                                                            ros_qos)

        self.current_velocity_sub = self.create_subscription(VehicleLocalPosition,
                                                           '/fmu/out/vehicle_local_position_v1',
                                                           self.current_velocity_callback,
                                                           px4_qos)

        
        #vehicle status
        self.land_requested_sub = self.create_subscription(Bool,
                                                           '/mission/land',
                                                           self.land_requested_callback,
                                                           ros_qos)


        self.vehicle_status_sub = self.create_subscription(VehicleStatus,
                                                           '/fmu/out/vehicle_status_v4',
                                                           self.vehicle_status_callback,
                                                           px4_qos)

        #flag
        self.set_point_counter = 0
        self.armed_sent = False

        self.land_requested = False
        self.land_command_sent = False

        self.lock_position_requested = False
        self.lock_position_sent = False

    #timer
    def timer_callback(self):

        if not self.land_requested:
            self.land_command_sent = False
            self.offboard_control_callback()
            self.set_point_pub()
        
        if self.set_point_counter < 10:
            self.set_point_counter += 1


        if self.set_point_counter == 10 and not self.armed_sent:
            self.take_off()
            self.arm()
            self.armed_sent = True
            print('arming.......')
            print('take off.........')
    

        if self.land_requested and not self.land_command_sent:
            self.land()
            if self.vehicle_nav_state == 18:
                self.land_command_sent = True
                print('landing......')
            else:
                print('trying to land......')
            print(self.vehicle_nav_state)


    #position
    def target_position_callback(self, msg: PoseStamped):
        self.target_x = float(msg.pose.position.x)
        self.target_y = float(msg.pose.position.y)
        self.target_z = float(msg.pose.position.z)

    def local_position_callback(self, msg: VehicleLocalPosition):
        self.local_x = msg.x
        self.local_y = msg.y
        self.local_z = msg.z

        current_position_msg = PoseStamped()
        current_position_msg.pose.position.x = self.local_x
        current_position_msg.pose.position.y = self.local_y
        current_position_msg.pose.position.z = self.local_z
        current_position_msg.header.stamp = self.get_clock().now().to_msg()
        self.current_position_pub.publish(current_position_msg)

    #velocity
    def target_velocity_callback(self, msg: TwistStamped):
        self.target_vx = float(msg.twist.linear.x)
        self.target_vy = float(msg.twist.linear.y)
        self.target_vz = float(msg.twist.linear.z)

    def current_velocity_callback(self, msg: VehicleLocalPosition):
        self.current_vx = msg.vx
        self.current_vy = msg.vy
        self.current_vz = msg.vz

        current_velocity_msg = TwistStamped()
        current_velocity_msg.twist.linear.x = self.current_vx
        current_velocity_msg.twist.linear.y = self.current_vy
        current_velocity_msg.twist.linear.z = self.current_vz
        current_velocity_msg.header.stamp = self.get_clock().now().to_msg()
        self.current_velocity_pub.publish(current_velocity_msg)


    #vehicle status & command
    def land_requested_callback(self, msg: Bool):
        if not self.land_requested and msg.data:
            self.land_requested = msg.data

    def offboard_control_callback(self):
        offboard_msg = OffboardControlMode()
        offboard_msg.position = True
        offboard_msg.velocity = True
        offboard_msg.acceleration = False
        offboard_msg.attitude = False
        offboard_msg.body_rate = False
        offboard_msg.thrust_and_torque = False
        offboard_msg.direct_actuator = False

        offboard_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.offboard_control_mode_pub.publish(offboard_msg)


    def set_point_pub(self):
        set_point_msg = TrajectorySetpoint()
        set_point_msg.position = [self.target_x, self.target_y, self.target_z]
        set_point_msg.velocity = [self.target_vx, self.target_vy, self.target_vz]
        set_point_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.target_position_pub.publish(set_point_msg)


    def vehicle_command_callback(self, msg, command, param1 = 0.0, param2 = 0.0, param3 = 0.0):
        command_msg = msg
        command_msg.command = command
        command_msg.param1 = param1
        command_msg.param2 = param2
        command_msg.param3 = param3

        command_msg.target_system = 1
        command_msg.target_component = 1
        command_msg.source_system = 1
        command_msg.source_component = 1
        command_msg.from_external = True

        command_msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_pub.publish(command_msg)


    def vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_nav_state = msg.nav_state


    def arm(self):
        self.vehicle_command_callback(VehicleCommand(), 
                                      VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                                      param1 = 1.0)

    def disarm(self):
        self.vehicle_command_callback(VehicleCommand(), 
                                        VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 
                                        param1 = 0.0)

    def take_off(self):
        self.vehicle_command_callback(VehicleCommand(), 
                                        VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
                                        param1 = 1.0, 
                                        param2 = 6.0)    

    def land(self):
        self.vehicle_command_callback(VehicleCommand(), 
                                        VehicleCommand.VEHICLE_CMD_NAV_LAND)

    def return_to_home(self):
        self.vehicle_command_callback(VehicleCommand(), 
                                        VehicleCommand.VEHICLE_CMD_NAV_RETURN_TO_LAUNCH)

def main(args = None):
    rclpy.init(args = args)
    node = OffboardControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()