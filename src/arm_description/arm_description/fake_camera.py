import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class FakeCamera(Node):
    def __init__(self):
        super().__init__('fake_camera')
        w, h, f = 640, 480, 500.0          # f=500 at 640 wide ~ 65 deg horizontal FOV
        self.img_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', 10)

        self.img = Image(height=h, width=w, encoding='rgb8', step=w * 3,
                         data=bytes(w * h * 3))           # all black
        self.info = CameraInfo(height=h, width=w, distortion_model='plumb_bob')
        self.info.d = [0.0] * 5
        self.info.k = [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0]
        self.info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.info.p = [f, 0.0, w / 2, 0.0, 0.0, f, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        self.create_timer(0.1, self.tick)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        for m in (self.img, self.info):
            m.header.stamp = stamp
            m.header.frame_id = 'camera_optical_frame'
        self.img_pub.publish(self.img)
        self.info_pub.publish(self.info)


def main():
    rclpy.init()
    rclpy.spin(FakeCamera())


if __name__ == '__main__':
    main()
