import cv2
import numpy as np

bahama_blue = np.uint8([[[147, 96, 0]]])

conv = cv2.cvtColor(bahama_blue, cv2.COLOR_BGR2HSV)
print(conv)
