import rospy
import cv2
from LemonEngine.sensors import Camera

def main():
    cam = Camera("/camera/color/image_raw")
    writer = cv2.VideoWriter(
        "/home/doge/Downloads/output0.mp4",
        cv2.VideoWriter_fourcc(*'mp4v'),
        20, (cam.width, cam.height),
    )

    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        image = frame.copy()
        writer.write(frame.copy())

        cv2.putText(image, f"frame: {cam.get_frame_cnt()}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.imshow("result", image)
        if cv2.waitKey(1) & 0xFF in [27, ord('q')]:
            break
    writer.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    rospy.init_node('demo', anonymous=True)
    main()
