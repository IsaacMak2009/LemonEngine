import rospy
import cv2
import os
from loguru import logger
from LemonEngine.sensors import Camera
from LemonEngine.ai import openvino as ov

def main():
    cam = Camera("/camera/color/image_raw", "bgr8")
    model = ov.YoloDetect("yolov9s", min_conf=0.8, device_name="GPU")
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        results = model.predict(frame).tolist()
        for result in results:
            x1, y1, x2, y2, conf, label = map(int, result)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    os.environ["OMP_NUM_THREADS"] = "8"  # HINT: for some machine, lower thread value may get better performance
    rospy.init_node('test_yolo_det', anonymous=True)
    main()