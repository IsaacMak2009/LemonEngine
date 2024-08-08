import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from loguru import logger


class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Assuming the camera is at index 0
        self.is_active = False
        if self.image_pub.get_num_connections() > 0:
            self.is_active = True

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            rate.sleep()

            self.is_active = self.image_pub.get_num_connections() > 0
            if not self.is_active:
                if self.cap.isOpened():
                    self.cap.release()
                    logger.info("No subscriber. Disabling camera.")
                continue

            if not self.cap.isOpened():
                self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)
                logger.info("Found subscriber(s). Re-opening camera.")

            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(msg)
                # cv2.imshow('frame', frame)
                # if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
                #     break

        self.cap.release()


if __name__ == '__main__':
    try:
        camera_publisher = CameraPublisher()
        camera_publisher.run()
    except rospy.ROSInterruptException:
        pass