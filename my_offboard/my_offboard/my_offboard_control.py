#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
import math

class MultiWaypointLand(Node):
    def __init__(self):
        super().__init__('multi_waypoint_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos_profile)

        self.interrupt_sub = self.create_subscription(String, '/interrupt_waypoint', self.interrupt_cb, 10)

        self.vehicle_status = VehicleStatus()
        self.local_pos = VehicleLocalPosition()
        self.interrupt_wp = None
        self.setpoint_counter = 0

        # 라벨 매핑 테이블
        self.label_map = {
            'S00': [0.0, 0.0, -2.5],
            'S01': [0.0, 12.65, -2.5],
            'S02': [0.0, 25.3, -2.5],
            'S03': [0.0, 37.95, -2.5],
            'S04': [0.0, 50.6, -2.5],
            'S05': [0.0, 63.25, -2.5],
            'S10': [-10.03, 0.0, -2.5],
            'S11': [-10.03, 12.65, -2.5],
            'S12': [-10.03, 25.3, -2.5],
            'S13': [-10.03, 37.95, -2.5],
            'S14': [-10.03, 50.6, -2.5],
            'S15': [-10.03, 63.25, -2.5],
            'S20': [-20.06, 0.0, -2.5],
            'S21': [-20.06, 12.65, -2.5],
            'S22': [-20.06, 25.3, -2.5],
            'S23': [-20.06, 37.95, -2.5],
            'S24': [-20.06, 50.6, -2.5],
            'S25': [-20.06, 63.25, -2.5],
            'S30': [-30.09, 0.0, -2.5],
            'S31': [-30.09, 12.65, -2.5],
            'S32': [-30.09, 25.3, -2.5],
            'S33': [-30.09, 37.95, -2.5],
            'S34': [-30.09, 50.6, -2.5],
            'S35': [-30.09, 63.25, -2.5],
            'S40': [-40.12, 0.0, -2.5],
            'S41': [-40.12, 12.65, -2.5],
            'S42': [-40.12, 25.3, -2.5],
            'S43': [-40.12, 37.95, -2.5],
            'S44': [-40.12, 50.6, -2.5],
            'S45': [-40.12, 63.25, -2.5],
            'S50': [-50.14, 0.0, -2.5],
            'S51': [-50.14, 12.65, -2.5],
            'S52': [-50.14, 25.3, -2.5],
            'S53': [-50.14, 37.95, -2.5],
            'S54': [-50.14, 50.6, -2.5],
            'S55': [-50.14, 63.25, -2.5],
            'S60': [-60.17, 0.0, -2.5],
            'S61': [-60.17, 12.65, -2.5],
            'S62': [-60.17, 25.3, -2.5],
            'S63': [-60.17, 37.95, -2.5],
            'S64': [-60.17, 50.6, -2.5],
            'S65': [-60.17, 63.25, -2.5],
            'S70': [-70.2, 0.0, -2.5],
            'S71': [-70.2, 12.65, -2.5],
            'S72': [-70.2, 25.3, -2.5],
            'S73': [-70.2, 37.95, -2.5],
            'S74': [-70.2, 50.6, -2.5],
            'S75': [-70.2, 63.25, -2.5],
            'S80': [-80.23, 0.0, -2.5],
            'S81': [-80.23, 12.65, -2.5],
            'S82': [-80.23, 25.3, -2.5],
            'S83': [-80.23, 37.95, -2.5],
            'S84': [-80.23, 50.6, -2.5],
            'S85': [-80.23, 63.25, -2.5],
            'S90': [-90.26, 0.0, -2.5],
            'S91': [-90.26, 12.65, -2.5],
            'S92': [-90.26, 25.3, -2.5],
            'S93': [-90.26, 37.95, -2.5],
            'S94': [-90.26, 50.6, -2.5],
            'S95': [-90.26, 63.25, -2.5],
}

        self.timer = self.create_timer(0.1, self.timer_cb)

    def interrupt_cb(self, msg):
        label = msg.data.strip().upper()
        if label in self.label_map:
            self.interrupt_wp = self.label_map[label]
            self.get_logger().info(f"[INTERRUPT] {label} 요청 수신 → {self.interrupt_wp}")
            self.setpoint_counter = 0
        else:
            self.get_logger().warn(f"[INTERRUPT] 알 수 없는 라벨: {label}")

    def status_cb(self, msg):
        self.vehicle_status = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg

    def timer_cb(self):
        if not self.interrupt_wp:
            return

        now = self.get_clock().now().nanoseconds // 1000
        goal = self.interrupt_wp

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = now
        offboard_mode.position = True
        self.mode_pub.publish(offboard_mode)

        dx = goal[0] - self.local_pos.x
        dy = goal[1] - self.local_pos.y
        yaw_rad = math.atan2(dy, dx)

        sp = TrajectorySetpoint()
        sp.timestamp = now
        sp.position[0] = goal[0]
        sp.position[1] = goal[1]
        sp.position[2] = goal[2]
        sp.yaw = yaw_rad
        self.sp_pub.publish(sp)

        if self.setpoint_counter == 10:
            self.send_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.send_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("[MODE] ARM + OFFBOARD sent")

        self.setpoint_counter += 1

        if self.check_goal_reached(goal):
            self.get_logger().info(f"[REACHED] 목표 도달: {goal}")
            self.interrupt_wp = None

    def check_goal_reached(self, goal):
        dx = self.local_pos.x - goal[0]
        dy = self.local_pos.y - goal[1]
        dz = self.local_pos.z - goal[2]
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if self.setpoint_counter < 50:
            return False
        self.get_logger().info(f"[CHECK] 거리: {dist:.2f} m")
        return dist < 1.5

    def send_command(self, command, **params):
        now = self.get_clock().now().nanoseconds // 1000
        msg = VehicleCommand()
        msg.timestamp = now
        msg.command = command
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MultiWaypointLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()