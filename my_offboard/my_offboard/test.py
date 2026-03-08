#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition, VehicleStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

# TF 관련
import tf2_ros
import geometry_msgs.msg

# RViz Interactive Marker 관련
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker


class MultiWaypointLand(Node):
    def __init__(self):
        super().__init__('multi_waypoint_land')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # PX4 Publishers
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        self.mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.sp_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)

        # PX4 Subscribers
        self.local_pos_sub = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.local_pos_cb, qos_profile)
        self.status_sub = self.create_subscription(VehicleStatus, '/fmu/out/vehicle_status', self.status_cb, qos_profile)

        # Interrupt waypoint topic
        self.interrupt_sub = self.create_subscription(String, '/interrupt_waypoint', self.interrupt_cb, 10)

        # TF Broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # RViz Interactive Marker Server
        self.server = InteractiveMarkerServer(self, "waypoint_markers")

        self.vehicle_status = VehicleStatus()
        self.local_pos = VehicleLocalPosition()
        self.interrupt_wp = None
        self.setpoint_counter = 0

        # Waypoints (축약본, 원래 건 그대로 사용 가능)
        self.waypoints = {
            'A0':  [10.0, 8.5, -2.5],
            'A1':  [2.0, 8.0, -2.5],
            'A2':  [-5.696, 8.0, -2.5],
            'A3':  [14.361, 16.0, -2.5],
            'A4':  [6.338, 16.0, -2.5],
            'A5':  [-1.685, 16.0, -2.5],
            'A6':  [-9.708, 16.0, -2.5],
            'A7':  [18.373, 24.0, -2.5],
            'A8':  [10.35, 24.0, -2.5],
            'A9':  [2.327, 24.0, -2.5],
            'A10': [-5.696, 24.0, -2.5],
            'A11': [-13.719, 24.0, -2.5],
            'A12': [14.361, 32.0, -2.5],
            'A13': [6.338, 32.0, -2.5],
            'A14': [-1.685, 32.0, -2.5],
            'A15': [-9.708, 32.0, -2.5],
            'A16': [10.35, 40.0, -2.5],
            'A17': [2.327, 40.0, -2.5],
            'A18': [-5.696, 40.0, -2.5],
            'B0':  [-59.67, 10.18, -2.5],
            'B1':  [-67.693, 10.18, -2.5],
            'B2':  [-75.716, 10.18, -2.5],
            'B3':  [-55.659, 18.203, -2.5],
            'B4':  [-63.682, 18.203, -2.5],
            'B5':  [-71.705, 18.203, -2.5],
            'B6':  [-79.728, 18.203, -2.5],
            'B7':  [-51.647, 26.226, -2.5],
            'B8':  [-59.67, 26.226, -2.5],
            'B9':  [-67.693, 26.226, -2.5],
            'B10': [-75.716, 26.226, -2.5],
            'B11': [-83.739, 26.226, -2.5],
            'B12': [-55.659, 34.248, -2.5],
            'B13': [-63.682, 34.248, -2.5],
            'B14': [-71.705, 34.248, -2.5],
            'B15': [-79.728, 34.248, -2.5],
            'B16': [-59.67, 42.271, -2.5],
            'B17': [-67.693, 42.271, -2.5],
            'B18': [-75.716, 42.271, -2.5],
            'C0':  [-56.219, 60.072, -2.5],
            'C1':  [-64.242, 60.072, -2.5],
            'C2':  [-72.266, 60.072, -2.5],
            'C3':  [-52.208, 68.095, -2.5],
            'C4':  [-60.231, 68.095, -2.5],
            'C5':  [-68.254, 68.095, -2.5],
            'C6':  [-76.277, 68.095, -2.5],
            'C7':  [-48.196, 76.118, -2.5],
            'C8':  [-56.219, 76.118, -2.5],
            'C9':  [-64.242, 76.118, -2.5],
            'C10': [-72.266, 76.118, -2.5],
            'C11': [-80.289, 76.118, -2.5],
            'C12': [-52.208, 84.141, -2.5],
            'C13': [-60.231, 84.141, -2.5],
            'C14': [-68.254, 84.141, -2.5],
            'C15': [-76.277, 84.141, -2.5],
            'C16': [-56.219, 92.164, -2.5],
            'C17': [-64.242, 92.164, -2.5],
            'C18': [-72.266, 92.164, -2.5],
            'E0':  [-158.945, 65.658, -2.5],
            'E1':  [-166.968, 65.658, -2.5],
            'E2':  [-174.991, 65.658, -2.5],
            'E3':  [-154.934, 73.681, -2.5],
            'E4':  [-162.957, 73.681, -2.5],
            'E5':  [-170.98, 73.681, -2.5],
            'E6':  [-179.003, 73.681, -2.5],
            'E7':  [-150.922, 81.704, -2.5],
            'E8':  [-158.945, 81.704, -2.5],
            'E9':  [-166.968, 81.704, -2.5],
            'E10': [-174.991, 81.704, -2.5],
            'E11': [-183.014, 81.704, -2.5],
            'E12': [-154.934, 89.726, -2.5],
            'E13': [-162.957, 89.726, -2.5],
            'E14': [-170.98, 89.726, -2.5],
            'E15': [-179.003, 89.726, -2.5],
            'E16': [-158.945, 97.749, -2.5],
            'E17': [-166.968, 97.749, -2.5],
            'E18': [-174.991, 97.749, -2.5],
            'D0':  [-162.418, 15.396, -2.5],
            'D1':  [-170.441, 15.396, -2.5],
            'D2':  [-178.464, 15.396, -2.5],
            'D3':  [-158.407, 23.419, -2.5],
            'D4':  [-166.43, 23.419, -2.5],
            'D5':  [-174.453, 23.419, -2.5],
            'D6':  [-182.476, 23.419, -2.5],
            'D7':  [-154.395, 31.442, -2.5],
            'D8':  [-162.418, 31.442, -2.5],
            'D9':  [-170.441, 31.442, -2.5],
            'D10': [-178.464, 31.442, -2.5],
            'D11': [-186.487, 31.442, -2.5],
            'D12': [-158.407, 39.465, -2.5],
            'D13': [-166.43, 39.465, -2.5],
            'D14': [-174.453, 39.465, -2.5],
            'D15': [-182.476, 39.465, -2.5],
            'D16': [-162.418, 47.488, -2.5],
            'D17': [-170.441, 47.488, -2.5],
            'D18': [-178.464, 47.488, -2.5],
            'F0':  [-232.227, 20.333, -2.5],
            'F1':  [-240.25, 20.333, -2.5],
            'F2':  [-248.273, 20.333, -2.5],
            'F3':  [-228.215, 28.356, -2.5],
            'F4':  [-236.238, 28.356, -2.5],
            'F5':  [-244.261, 28.356, -2.5],
            'F6':  [-252.284, 28.356, -2.5],
            'F7':  [-224.204, 36.379, -2.5],
            'F8':  [-232.227, 36.379, -2.5],
            'F9':  [-240.25, 36.379, -2.5],
            'F10': [-248.273, 36.379, -2.5],
            'F11': [-256.296, 36.379, -2.5],
            'F12': [-228.215, 44.402, -2.5],
            'F13': [-236.238, 44.402, -2.5],
            'F14': [-244.261, 44.402, -2.5],
            'F15': [-252.284, 44.402, -2.5],
            'F16': [-232.227, 52.424, -2.5],
            'F17': [-240.25, 52.424, -2.5],
            'F18': [-248.273, 52.424, -2.5],
        }

        # RViz 마커 생성
        self.create_interactive_markers()

        # 0.1초마다 실행
        self.timer = self.create_timer(0.1, self.timer_cb)

    # -----------------------
    # Interactive Marker 생성
    # -----------------------
    def create_interactive_markers(self):
        for label, pos in self.waypoints.items():
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name = label
            int_marker.description = label
            int_marker.pose.position.x = pos[0]
            int_marker.pose.position.y = pos[1]
            int_marker.pose.position.z = 0.0
            int_marker.scale = 1.0

            # 시각화용 구체
            sphere_marker = Marker()
            sphere_marker.type = Marker.SPHERE
            sphere_marker.scale.x = 0.5
            sphere_marker.scale.y = 0.5
            sphere_marker.scale.z = 0.5
            sphere_marker.color.r = 1.0
            sphere_marker.color.g = 0.5
            sphere_marker.color.b = 0.0
            sphere_marker.color.a = 1.0

            control = InteractiveMarkerControl()
            control.always_visible = True
            control.markers.append(sphere_marker)
            int_marker.controls.append(control)

            # 클릭 이벤트 처리
            control = InteractiveMarkerControl()
            control.interaction_mode = InteractiveMarkerControl.BUTTON
            int_marker.controls.append(control)

            self.server.insert(int_marker, self.marker_feedback)

        self.server.applyChanges()

    def marker_feedback(self, feedback):
        label = feedback.marker_name
        self.get_logger().info(f"[RVIZ] 마커 클릭됨: {label}")
        self.publish_interrupt_waypoint(label)

    def publish_interrupt_waypoint(self, label):
        pub = self.create_publisher(String, '/interrupt_waypoint', 10)
        msg = String()
        msg.data = label
        pub.publish(msg)

    # -----------------------
    # PX4 Callback
    # -----------------------
    def interrupt_cb(self, msg):
        label = msg.data.strip().upper()
        if label in self.waypoints:
            self.interrupt_wp = self.waypoints[label]
            self.get_logger().info(f"[INTERRUPT] {label} 요청 수신 → {self.interrupt_wp}")
            self.setpoint_counter = 0
        else:
            self.get_logger().warn(f"[INTERRUPT] 알 수 없는 라벨: {label}")

    def status_cb(self, msg):
        self.vehicle_status = msg

    def local_pos_cb(self, msg):
        self.local_pos = msg

    # -----------------------
    # 주기 실행
    # -----------------------
    def timer_cb(self):
        # 드론 위치 TF 발행
        self.publish_tf("drone_base_link", self.local_pos.x, self.local_pos.y, self.local_pos.z)

        # 모든 waypoint TF 발행
        for label, pos in self.waypoints.items():
            self.publish_tf(label, pos[0], pos[1], pos[2])

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
            self.get_logger().info("[MODE] OFFBOARD sent")

        self.setpoint_counter += 1

        if self.check_goal_reached(goal):
            self.get_logger().info(f"[REACHED] 목표 도달: {goal}")
            self.interrupt_wp = None

    # -----------------------
    # TF 발행
    # -----------------------
    def publish_tf(self, child_frame, x, y, z):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "map"
        t.child_frame_id = child_frame
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(t)

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
