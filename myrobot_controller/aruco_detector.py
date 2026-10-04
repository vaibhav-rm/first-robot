#!/usr/bin/env python3
"""ArUco detector: detects DICT_4X4_50 markers, draws borders, chains missions.

Subscribes:
  /camera/image_raw (sensor_msgs/Image) - forward RGB from depth camera
Publishes:
  /Nav2_coordinates (geometry_msgs/Point) - next goal decoded from marker id
  /aruco/annotated_image (sensor_msgs/Image) - image with borders drawn

Mission chain (marker face positions in simple_obstacles.world):
  id=1 at (2.5, 1.5)   -> publishes approach pose of marker 2 (-2.0, -0.8)
  id=2 at (-2.5, -1.0) -> publishes approach pose of marker 3 (1.2, -2.2)
  id=3 at (1.5, -2.8)  -> mission complete (no publish)

Each marker is detected once (processed_markers set) to avoid re-publishing.
Borders are drawn with cv2.aruco.drawDetectedMarkers (visible on
/aruco/annotated_image and in logs).
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
try:
    import cv2.aruco as aruco
except ImportError:  # pragma: no cover
    aruco = None


class ArucoDetector(Node):
    def __init__(self):
        super().__init__('aruco_detector')
        self.bridge = CvBridge()

        if aruco is None:
            self.get_logger().error('cv2.aruco not available! Install opencv-contrib-python')
            raise RuntimeError('cv2.aruco missing')

        # ArUco dictionary (markers generated as DICT_4X4_50, ids 1..3)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        try:
            self.aruco_params = aruco.DetectorParameters()
            # New API (opencv-contrib >= 4.7): ArucoDetector object
            self._detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)
        except AttributeError:
            self.aruco_params = aruco.DetectorParameters_create()
            self._detector = None

        # Marker id -> next-waypoint (x, y) in map frame.
        # Approach poses are set ~0.5m in front of each board so Nav2 can
        # observe the next marker with the forward camera.
        self.marker_missions = {
            1: (-2.0, -0.8),   # seen at (2.5,1.5) -> go towards marker 2
            2: (1.2, -2.2),    # seen at (-2.5,-1.0) -> go towards marker 3
            3: None,           # final marker at (1.5,-2.8) -> mission complete
        }

        self.processed_markers = set()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        self.coord_pub = self.create_publisher(Point, '/Nav2_coordinates', 10)
        self.annot_pub = self.create_publisher(Image, '/aruco/annotated_image', 10)

        self.get_logger().info('ArUco Detector initialized (DICT_4X4_50, ids 1-3)')

    def _detect(self, gray):
        """Handle both legacy and new OpenCV ArUco APIs."""
        if self._detector is not None:
            corners, ids, rejected = self._detector.detectMarkers(gray)
        else:
            corners, ids, rejected = aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params)
        return corners, ids, rejected

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'CvBridge error: {e}')
            return

        if cv_image is None or cv_image.size == 0:
            self.get_logger().warn('Received empty image frame.')
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        try:
            corners, ids, _ = self._detect(gray)
        except Exception as e:
            self.get_logger().error(f'Marker detection error: {e}')
            return

        if ids is not None:
            # Draw green borders + ids around each detected marker
            aruco.drawDetectedMarkers(cv_image, corners, ids)

            for marker_id in ids.flatten().tolist():
                marker_id = int(marker_id)
                if marker_id not in self.processed_markers and marker_id in self.marker_missions:
                    mission = self.marker_missions[marker_id]
                    if mission is not None:
                        self.get_logger().info(
                            f'Detected marker id={marker_id}! Publishing next goal {mission} to /Nav2_coordinates')
                        goal_msg = Point(x=float(mission[0]), y=float(mission[1]), z=0.0)
                        self.coord_pub.publish(goal_msg)
                    else:
                        self.get_logger().info(
                            f'Detected final marker id={marker_id}! Mission complete.')
                    self.processed_markers.add(marker_id)

        try:
            self.annot_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, 'bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    detector = ArucoDetector()
    rclpy.spin(detector)
    detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
