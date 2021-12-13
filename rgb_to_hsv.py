import cv2
import numpy as np

rgb = [37, 150, 190]
# The BGR value of the book
bgr = np.uint8([[[190, 150, 37]]])
hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
print(hsv)
