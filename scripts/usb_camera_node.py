import cv2
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher:
    def __init__(self):
        rospy.init_node('camera_publisher')
        self.bridge = CvBridge()
        self.image_pub = rospy.Publisher('/camera/color/image_raw', Image, queue_size=10)
        self.cap = cv2.VideoCapture(0, cv2.CAP_V4L2)  # Assuming the camera is at index 0

    def run(self):
        rate = rospy.Rate(30)  # 30 Hz
        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
                self.image_pub.publish(msg)
                # cv2.imshow('frame', frame)
                # if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
                #     break
            rate.sleep()

        self.cap.release()


if __name__ == '__main__':
    try:
        camera_publisher = CameraPublisher()
        camera_publisher.run()
    except rospy.ROSInterruptException:
        pass