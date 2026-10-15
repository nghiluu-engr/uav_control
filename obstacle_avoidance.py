#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import LaserScan
from px4_msgs.msg import VehicleAttitude
from geometry_msgs.msg import PoseStamped
import math
import numpy as np

lidar_qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                         durability = DurabilityPolicy.VOLATILE,
                         history = HistoryPolicy.KEEP_LAST,
                         depth = 1)

px4_qos = QoSProfile(reliability = ReliabilityPolicy.BEST_EFFORT,
                         durability = DurabilityPolicy.TRANSIENT_LOCAL,
                         history = HistoryPolicy.KEEP_LAST,
                         depth = 1)

ros_qos = 10

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        print('start obstacle_avoidance_node')

        #value
        self.lidar_ranges = []
        self.lidar_data = []
        self.current_angle = None
        self.angle_min = None
        self.angle_increment = None
        self.range_min = None
        self.range_max = None

        self.Quaternion_frd_to_ned = []

        self.sector_width = math.radians(5)
        self.number_of_sectors = 0

        self.block_distance = 3.0
        self.drone_radius = 0.5 #m
        self.safety_margin = 1.0 #m

        self.blocked_sectors = []

        self.goal_north = 0.0
        self.goal_east = 0.0
        self.current_north = 0.0
        self.current_east = 0.0
        self.goal_bearing = 0.0

        self.safe_x = 0.0
        self.safe_y = 0.0
        self.safe_z = 0.0

        #flag
        self.obstacle_detected = False

        #timer
        self.timer = self.create_timer(0.2, self.timer_callback)

        #pub
        self.safe_waypoint_pub = self.create_publisher(PoseStamped,
                                                       '/mission/safe_waypoint',
                                                       ros_qos)

        #sub
        self.lidar_data_sub = self.create_subscription(LaserScan,
                                                         '/scan',
                                                         self.lidar_data_callback,
                                                         lidar_qos)

        self.vehicle_attitude_sub = self.create_subscription(VehicleAttitude,
                                                             '/fmu/out/vehicle_attitude',
                                                             self.vehicle_attitude_callback,
                                                             px4_qos)

        self.current_position_sub = self.create_subscription(PoseStamped,
                                                            '/mission/current_position',
                                                            self.current_position_callback,
                                                            ros_qos)

        self.target_position_sub = self.create_subscription(PoseStamped,
                                                            '/mission/target_position',
                                                            self.target_position_callback,
                                                            ros_qos)
#main
    def timer_callback(self):
        self.lidar_data_filter()

        points_north, points_east = self.scan_to_ned()
        self.detect_obstacle(points_north, points_east)

        if self.detect_obstacle(points_north, points_east) is None:
            self.safe_x = self.current_north
            self.safe_y = self.current_east

        if self.obstacle_detected:
            print('obstacle detected...........')
            self.find_safe_waypoint()
            print('safe waypoint found......')

        else:
            self.safe_x = self.goal_north
            self.safe_y = self.goal_east

        self.safe_waypoint_publisher()
        print(f'safe waypoint: x: {self.safe_x}, y: {self.safe_y}')


#sub
    def vehicle_attitude_callback(self, msg: VehicleAttitude):
        self.Quaternion_frd_to_ned = msg.q

    def lidar_data_callback(self, msg: LaserScan):
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment
        self.lidar_ranges = msg.ranges
        self.range_min = msg.range_min
        self.range_max = msg.range_max

#scan process

    #filter lidar data and get angle from lidar ranges
    def lidar_data_filter(self):
        self.obstacle_detected = False
        self.lidar_data = []

        for i, distance in enumerate(self.lidar_ranges): # (index, distance)
            if not math.isfinite(distance): # remove nan, inf+, inf-
                continue

            if not (self.range_min < distance < self.range_max): # limit range
                continue

            self.lidar_data.append((self.get_angle(i), distance)) # angle (rad), distance (m)

        print(f'lidar data: {self.lidar_data}')

    def get_angle(self, index):
        return self.angle_min + index * self.angle_increment

    def quat_to_rotmat(self): 
        w, x, y, z = self.Quaternion_frd_to_ned

        quat_norm = math.sqrt(x*x + y*y + z*z + w*w)
        if not math.isclose(quat_norm, 1.0, abs_tol=1e-3):
            raise ValueError(f'ERROR: quat_norm = {quat_norm}')

        rotmat = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                           [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                           [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])
        return rotmat

    def scan_to_ned(self):
        if self.lidar_data is None:
            return

        #divide lidar data into 2 arrays
        beam_angles = np.array([angle for angle, _ in self.lidar_data]) # get angles in lidar data
        beam_distances = np.array([distance for _, distance in self.lidar_data]) # get distances in lidar data

        #flu -> frd
        points_frd = np.stack([beam_distances * np.cos(beam_angles), # get forward
                               - beam_distances * np.sin(beam_angles), # get right
                               np.zeros_like(beam_angles)], # lidar 2d -> down = 0
                              axis = 1) #[x, y, z]

        #frd -> ned
        rotmat_body_to_ned = self.quat_to_rotmat() 
        points_ned = points_frd @ rotmat_body_to_ned.T

        #get north, east
        points_north = points_ned[:, 0]
        points_east = points_ned[:, 1]
        return points_north, points_east

#vhf+

    def detect_obstacle(self, points_north, points_east):
        self.number_of_sectors = int(2 * math.pi / self.sector_width) 
        safety_radius = self.drone_radius + self.safety_margin
        self.blocked_sectors = [False] * self.number_of_sectors

        for north, east in zip(points_north, points_east):
            drone_to_obstacle = math.sqrt(north * north + east * east)

            if drone_to_obstacle < 1e-3 or drone_to_obstacle > self.block_distance:
                continue

            if drone_to_obstacle <= safety_radius:
                return 

            # north = 0 rad
            obstacle_bearing = math.atan2(east, north)

            half_avoidance_angle = math.asin(min(safety_radius / drone_to_obstacle, 1.0))

            # find avoidance range
            first_avoidance_sector = int((obstacle_bearing - half_avoidance_angle) // self.sector_width)
            last_avoidance_sector = int((obstacle_bearing + half_avoidance_angle) // self.sector_width)

            # mark blocked sectors
            for sector_index in range (first_avoidance_sector, last_avoidance_sector + 1):
                self.blocked_sectors[sector_index % self.number_of_sectors] = True

        self.obstacle_detected = any(self.blocked_sectors)

        return self.obstacle_detected

    def find_valleys(self):
        free_sectors = [not blocked for blocked in self.blocked_sectors]
        valleys = []

        if not any(free_sectors): return valleys
        if all(free_sectors):
            valleys = [(0, self.number_of_sectors)]
            return valleys

        shift = free_sectors.index(False)
        rotated_list = free_sectors[shift:] + free_sectors[:shift]

        i = 0
        while i < self.number_of_sectors:
            if rotated_list[i]:
                start = i
                while i < self.number_of_sectors and rotated_list[i]:
                    i += 1
                length = i - start
                valleys.append(((start + shift) % self.number_of_sectors, length))
            else:
                i += 1

        return valleys

    def find_candidate_directions(self):
        narrow_valley_width = 8
        min_valley_sectors = 3
        margin_sectors = 4

        candidate_directions = []

        goal_bearing = math.atan2(self.goal_east - self.current_east, self.goal_north - self.current_north) % (2 * math.pi)

        def angle_diff(a, b):
            return abs(math.atan2(math.sin(a - b), math.cos(a - b)))

        for start_valley, length_valley in self.find_valleys():

            if length_valley == self.number_of_sectors:
                candidate_directions.append(goal_bearing)
                continue

            if length_valley <= min_valley_sectors:
                continue

            elif length_valley <= narrow_valley_width:
                candidate_sector = start_valley + length_valley / 2

            else:
                start_angle = start_valley * self.sector_width
                end_angle = (start_valley + length_valley) * self.sector_width

                if angle_diff(start_angle, goal_bearing) <= angle_diff(end_angle, goal_bearing):
                    candidate_sector = start_valley + margin_sectors
                else:
                    candidate_sector = start_valley + length_valley - margin_sectors

            candidate_angle = (candidate_sector * self.sector_width) % (2 * math.pi)
            candidate_directions.append(candidate_angle)

        return candidate_directions


    def choose_safe_angle(self):
        goal_bearing = math.atan2(self.goal_east - self.current_east, self.goal_north - self.current_north) % (2 * math.pi)
        candidate_directions = self.find_candidate_directions()

        safe_angle = float('inf')
        i = 0
        for i in range(len(candidate_directions)):
            if abs(goal_bearing - candidate_directions[i]) < safe_angle:
                safe_angle = candidate_directions[i]
                i += 1

        return safe_angle

    def find_safe_waypoint(self):
        goal_distance = math.sqrt((self.goal_north - self.current_north) * (self.goal_north - self.current_north) 
                                  + (self.goal_east - self.current_east) * (self.goal_east - self.current_east))
        
        safe_angle = self.choose_safe_angle()
        if safe_angle is None:
            self.safe_x = self.current_north
            self.safe_y = self.current_east
            return 

        safe_distance = min(goal_distance, 1.0)

        self.safe_x = self.current_north + safe_distance * math.cos(safe_angle)
        self.safe_y = self.current_east + safe_distance * math.sin(safe_angle)


#timing
#health check
#sub
    def target_position_callback(self, msg: PoseStamped):
        self.goal_north = msg.pose.position.x
        self.goal_east = msg.pose.position.y

    def current_position_callback(self, msg: PoseStamped):
        self.current_north = msg.pose.position.x
        self.current_east = msg.pose.position.y

#pub
    def safe_waypoint_publisher(self):
        safe_waypoint_msg = PoseStamped()
        safe_waypoint_msg.pose.position.x = self.safe_x
        safe_waypoint_msg.pose.position.y = self.safe_y
        safe_waypoint_msg.pose.position.z = self.safe_z
        safe_waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        self.safe_waypoint_pub.publish(safe_waypoint_msg)


def main(args = None):
    rclpy.init(args = args)
    node = ObstacleAvoidanceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()