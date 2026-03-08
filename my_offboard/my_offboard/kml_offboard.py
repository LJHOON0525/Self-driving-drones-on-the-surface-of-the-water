#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, Marker
from std_msgs.msg import String

class InteractiveWaypointPublisher(Node):
    def __init__(self):
        super().__init__('interactive_waypoint_publisher')
        self.publisher = self.create_publisher(String, '/interrupt_waypoint', 10)
        self.server = InteractiveMarkerServer(self, "waypoint_markers")

        self.waypoints = {
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

        self.create_interactive_markers()

    def create_interactive_markers(self):
        for name, pos in self.waypoints.items():
            int_marker = InteractiveMarker()
            int_marker.header.frame_id = "map"
            int_marker.name = name
            int_marker.description = name
            int_marker.pose.position.x = pos[0]
            int_marker.pose.position.y = pos[1]
            int_marker.pose.position.z = 0.5

            control = InteractiveMarkerControl()
            control.interaction_mode = InteractiveMarkerControl.BUTTON
            control.always_visible = True

            marker = Marker()
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.z = 1.2
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 1.0
            marker.text = name

            control.markers.append(marker)
            int_marker.controls.append(control)

            self.server.insert(int_marker)  # ✅ 콜백 없이 insert
            self.server.setCallback(name, self.process_feedback)  # ✅ 콜백은 따로 등록

        self.server.applyChanges()

    def process_feedback(self, feedback):
        label = feedback.marker_name
        msg = String()
        msg.data = label
        self.publisher.publish(msg)
        self.get_logger().info(f"[CLICK] {label} → /interrupt_waypoint 퍼블리시")

def main(args=None):
    rclpy.init(args=args)
    node = InteractiveWaypointPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()