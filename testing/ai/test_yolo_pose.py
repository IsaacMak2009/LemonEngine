import rospy
import cv2
import os
from loguru import logger
from LemonEngine.sensors import Camera
from LemonEngine.ai import openvino as ov

def main():
    cam = Camera("/camera/color/image_raw", "bgr8")
    model = ov.YoloPose(min_conf=0.8, device_name="GPU")
    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        rate.sleep()
        frame = cam.get_frame()
        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        bboxs, keypoints = model.predict(rgb_frame)
        for bbox, keypoint in zip(bboxs, keypoints):
            x1, y1, x2, y2, conf, label = map(int, bbox)
            cv2.putText(frame, f"{label}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)

            for kpt in keypoint:
                x, y, conf = map(int, kpt)
                cv2.circle(frame, (x, y), 3, (0, 255, 0), -1)
        

        cv2.imshow("frame", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

if __name__ == '__main__':
    os.environ["OMP_NUM_THREADS"] = "8"  # HINT: for some machine, lower thread value may get better performance
    rospy.init_node('test_yolo_det', anonymous=True)
    main()