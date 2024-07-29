import numpy as np

import rospy
import cv2
from LemonEngine.sensors import Camera
from LemonEngine.utils.vision import *
from loguru import logger

PATH = "../datas/snap3.npy"

def main():
    cam = Camera("/camera/depth/image_raw", "passthrough")
    total = 20
    out_frame = []

    for i in range(total):
        logger.info(f"Snapping [{i+1}/{total}]...")
        rospy.sleep(0.5)
        frame = cam.get_frame().astype(np.int32)
        out_frame.append(frame)
        result = np.median(out_frame, axis=0)
        result /= 50
        result = result.astype(np.uint8)
        cv2.imshow("frame", result)
        cv2.waitKey(1)
    cv2.waitKey(0)
    result = np.median(out_frame, axis=0)

    np.save(PATH, result)
    cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('test_camera', anonymous=True)
    main()