import rospy
import cv2
from LemonEngine.sensors import Camera

def main():
    cam1 = Camera("/camera/rgb/image_raw", "bgr8")
    cam2 = Camera("/camera/depth/image_raw", "passthrough")
    width, height = cam1.width, cam1.height
    cx, cy = width // 2, height // 2

    while not rospy.is_shutdown():
        frame = cam1.get_frame()
        depth_frame = cam2.get_frame()
        depth_frame = cv2.resize(depth_frame, (width, height))
        depth = depth_frame[cy, cx]

        frame = cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        frame = cv2.putText(frame, f"{depth}", (cx, cy), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

        cv2.imshow("depth", depth_frame)
        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    rospy.init_node('test_camera', anonymous=True)
    main()