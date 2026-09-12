#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from geometry_msgs.msg import PoseStamped, TwistStamped
from std_msgs.msg import Bool
from enum import Enum
import math

class MissionState(Enum):
    WAIT_FOR_POSITION = 0
    TAKE_OFF = 1
    MISSION = 2
    LANDING_SEARCH = 3
    DONE = 4

ros_qos = 10

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')

        #parameters
        self.declare_parameter('take_off_altitude', -5.0)
        self.declare_parameter('hold_count_required', 5.0)
        self.declare_parameter('waypoint',[0.0, 0.0, -5.0,
                                           -5.0, 0.0, -5.0,
                                           0.0, -5.0, -5.0])


        #value
        self.take_off_altitude = self.get_parameter('take_off_altitude').value
        self.hold_count_required = self.get_parameter('hold_count_required').value
        self.waypoint = self.get_parameter('waypoint').value

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 0.0

        self.distance_x = 0.0
        self.distance_y = 0.0
        self.distance_z = 0.0

        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0

        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0

        self.time_start_hover = 0.0
        self.current_time = 0.0

        self.state = MissionState.WAIT_FOR_POSITION

        self.waypoint_counter = 0

        self.max_speed = 1.0
        self.max_accel = 0.5
        self.ramp_speed = 0.0

        #timer
        self.timer = self.create_timer(0.1, self.timer_callback)

    #pub

        #position
        self.target_position_pub = self.create_publisher(PoseStamped,
                                                    '/mission/target_position',
                                                    ros_qos)
        #velocity
        self.target_velocity_pub = self.create_publisher(TwistStamped,
                                                        '/mission/target_velocity',
                                                        ros_qos)
        #vehicle status
        self.land_pub = self.create_publisher(Bool,
                                            '/mission/land',
                                            ros_qos)

        self.lock_position_pub = self.create_publisher(Bool,
                                                        '/mission/lock_position',
                                                        ros_qos)

        #sub
        self.current_position_sub = self.create_subscription(PoseStamped,
                                                            '/mission/current_position',
                                                            self.current_position_callback,
                                                            ros_qos)

        self.current_velocity_sub = self.create_subscription(TwistStamped,
                                                            '/mission/current_velocity',
                                                            self.current_velocity_callback,
                                                            ros_qos)

    #main
    def timer_callback(self):
        if self.state != MissionState.LANDING_SEARCH:
            self.target_position_publisher(self.target_x, self.target_y, self.target_z)
            print(f'target position: x: {self.target_x}, y: {self.target_y}, z: {self.target_z}')
            print(f'current position: x: {self.current_x}, y: {self.current_y}, z: {self.current_z}')

            self.velocity_control()

            self.target_velocity_publisher(self.target_vx, self.target_vy, self.target_vz)
            print(f'target velocity: vx: {self.target_vx}, vy: {self.target_vy}, vz: {self.target_vz}')
            print(f'current velocity: x: {self.current_vx}, y: {self.current_vy}, z: {self.current_vz}')

        if self.state == MissionState.WAIT_FOR_POSITION:
            self.state = MissionState.TAKE_OFF

        if self.state == MissionState.TAKE_OFF:
            self.take_off_handle()

        if self.state == MissionState.MISSION:
            self.handle_mission()

        if self.state == MissionState.LANDING_SEARCH:
            self.land_publisher(True)
            self.state = MissionState.DONE

        
    #state
    def take_off_handle(self):
        dz = self.take_off_altitude - self.current_z
        dist = abs(dz)

        if dist <= 0.1 * abs(self.take_off_altitude):
            self.target_x = 0.0
            self.target_y = 0.0
            self.target_z = self.take_off_altitude

            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_vz = 0.0

            self.ramp_speed = 0.0
            self.state = MissionState.MISSION

        else:
            self.target_x = self.current_x
            self.target_y = self.current_y
            self.target_z = self.current_z

            dt = 0.1
            desired_speed = min(self.max_speed, self.max_speed)  
            self.ramp_speed = min(desired_speed, self.ramp_speed + self.max_accel * dt)

            self.target_vx = 0.0
            self.target_vy = 0.0
            self.target_vz = -self.ramp_speed  

    def handle_mission(self):
        self.waypoint_tracking()

    #mission
    def waypoint_tracking(self):
        if self.waypoint_counter >= 3:
            self.state = MissionState.LANDING_SEARCH
            return

        elif self.time_start_hover != 0.0:
            self.hover()

        else:
            wx = self.waypoint[3 * self.waypoint_counter]
            wy = self.waypoint[3 * self.waypoint_counter + 1]
            wz = self.waypoint[3 * self.waypoint_counter + 2]

            dx = wx - self.current_x
            dy = wy - self.current_y
            dz = wz - self.current_z
            dist = math.sqrt(dx**2 + dy**2 + dz**2)
   
            if math.isclose(dist, 0.0, abs_tol=0.7):
                print(f'\033[92m WAYPOINT {self.waypoint_counter + 1} ARRIVED\033[0m')

                self.target_x = wx
                self.target_y = wy
                self.target_z = wz
                self.ramp_speed = 0.0   # reset ramp cho lần bay tiếp theo

                self.hover()

            else:
                self.target_x = self.current_x
                self.target_y = self.current_y
                self.target_z = self.current_z

                dt = 0.1
                desired_speed = self.max_speed if dist > 2.0 else self.max_speed / 2
                self.ramp_speed = min(desired_speed, self.ramp_speed + self.max_accel * dt)

                self.target_vx = self.ramp_speed * dx / dist
                self.target_vy = self.ramp_speed * dy / dist
                self.target_vz = self.ramp_speed * dz / dist


    def hover(self):
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_vz = 0.0

        if self.time_start_hover == 0.0:
            self.time_start_hover = self.get_clock().now().nanoseconds / 1e9
            print('start hovering: ', self.time_start_hover)
        else:
            self.current_time = self.get_clock().now().nanoseconds / 1e9
            print(f'\033[94m current time: \033[0m', self.current_time)

            if self.current_time - self.time_start_hover >= 0.9 * self.hold_count_required:
                self.time_start_hover = 0.0                  
                self.waypoint_counter += 1


    def velocity_control(self):
        if abs(self.current_vx) > self.max_speed or \
            abs(self.current_vy) > self.max_speed or \
            abs(self.current_vz) > self.max_speed:

            if self.target_vx < 0.0 or self.target_x < 0.0:
                self.current_vx = - self.max_speed
            else: self.current_vx = self.max_speed

            if self.target_vy < 0.0 or self.target_y < 0.0:
                self.current_vy = - self.max_speed
            else: self.current_vy = self.max_speed

            if self.target_vz < 0.0 or self.target_z < 0.0:
                self.current_vz = - self.max_speed
            else: self.current_vz = self.max_speed

    #pub
    def target_position_publisher(self, x, y, z):
        target_position_msg = PoseStamped()
        target_position_msg.pose.position.x = x
        target_position_msg.pose.position.y = y
        target_position_msg.pose.position.z = z
        target_position_msg.header.stamp = self.get_clock().now().to_msg()
        self.target_position_pub.publish(target_position_msg)

    def target_velocity_publisher(self, vx, vy, vz):
        target_velocity_msg = TwistStamped()
        target_velocity_msg.twist.linear.x = vx
        target_velocity_msg.twist.linear.y = vy
        target_velocity_msg.twist.linear.z = vz
        target_velocity_msg.header.stamp = self.get_clock().now().to_msg()
        self.target_velocity_pub.publish(target_velocity_msg)

    def land_publisher(self, landing_state: Bool):
        land_msg = Bool()
        land_msg.data = landing_state
        self.land_pub.publish(land_msg)

    def lock_position_publisher(self, lock_position_state: Bool):
        lock_position_msg = Bool()
        lock_position_msg.data = lock_position_state
        self.lock_position_pub.publish(lock_position_msg)


    #sub
    def current_position_callback(self, msg: PoseStamped):
        self.current_x = msg.pose.position.x
        self.current_y = msg.pose.position.y
        self.current_z = msg.pose.position.z
        

    def current_velocity_callback(self, msg: TwistStamped):
        self.current_vx = msg.twist.linear.x
        self.current_vy = msg.twist.linear.y
        self.current_vz = msg.twist.linear.z
        

def main(args = None):
    rclpy.init(args = args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()