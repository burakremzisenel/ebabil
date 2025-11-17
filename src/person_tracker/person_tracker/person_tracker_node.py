import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, BoundingBox2D


class PersonTrackerNode(Node):
    def __init__(self):
        super().__init__('person_tracker')

        # Parameter: welches Kameratopic?
        self.declare_parameter(
            'image_topic',
            '/cam_front/camera/color/image_raw'
        )
        image_topic = self.get_parameter('image_topic').get_parameter_value().string_value

        # Subscriber für Farb-Bild
        self.image_sub = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10
        )

        # Publisher für Detektionen
        self.detections_pub = self.create_publisher(
            Detection2DArray,
            'person_detections',
            10
        )

        self.get_logger().info(
            f'PersonTrackerNode gestartet. '
            f'Image-Topic: {image_topic}, Output: person_detections'
        )

    def image_callback(self, msg: Image):
        # Wenn kein valides Bild (z.B. width = 0), nichts tun
        if msg.width == 0 or msg.height == 0:
            return

        # Dummy-Detektion: eine "Person" zentral im Bild
        bbox = BoundingBox2D()
        bbox.center.position.x = float(msg.width) / 2.0
        bbox.center.position.y = float(msg.height) / 2.0
        bbox.size_x = float(msg.width) * 0.3
        bbox.size_y = float(msg.height) * 0.5

        det = Detection2D()
        det.header = msg.header
        det.bbox = bbox
        det.id = 'person_1'  # später z.B. Tracking-ID vom Tracker

        array_msg = Detection2DArray()
        array_msg.header = msg.header
        array_msg.detections.append(det)

        self.detections_pub.publish(array_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PersonTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()