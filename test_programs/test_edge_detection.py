import numpy as np
import cv2
from matplotlib import pyplot as plt

def create_image():
    height = 480
    width = 640
    blank_image = np.zeros((height, width, 3), np.uint8)
    return blank_image

def mask_frame(frame):

    # blurred = cv2.medianBlur(frame, 5)
    blurred = frame
    # cv2.imshow('blurred', blurred)

    # Better Edge detection
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

    lower_red = np.array([0, 50, 50])
    upper_red = np.array([10, 255, 255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([170, 50, 50])
    upper_red = np.array([180, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    mask = mask0 + mask1

    output_gray = gray.copy()
    output_gray[np.where(mask == 0)] = 0

    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    new_contours = []

    for contour in contours:
        if cv2.contourArea(contour) > 1000:
            new_contours.append(contour)
        else:
            x, y, w, h = cv2.boundingRect(contour)
            for col in range(x, x + w):
                for row in range(y, y + h):
                    blurred[row][col] = [0, 0, 0]
            # for pixel in contour:
            #     row = pixel[0][1]
            #     col = pixel[0][0]
                # blurred[row][col] = [0, 0, 0]

    blank = create_image()

    for row in range(480):
        for col in range(640):
            if mask[row][col]:
                blank[row][col] = blurred[row][col]

    # for new_c in new_contours:
    #     x, y, w, h = cv2.boundingRect(new_c)
    #     for col in range(x, x + w):
    #         for row in range(y, y + h):
    #             blank[row][col] = blurred[row][col]

    # cv2.drawContours(blank, new_contours, -1, 255, 3)

    return blank

def main():
    img = cv2.imread('overlap.png', 0)
    # edges = cv2.Canny(img, 100, 200)
    mask = mask_frame(cv2.imread('overlap.png'))
    cv2.imshow('mask', mask)
    edges = cv2.Canny(mask, 50, 100)
    # plt.subplot(121), plt.imshow(mask_frame(img), cmap='gray')
    # plt.title('Contour Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(121), plt.imshow(img, cmap='gray')
    plt.title('Original Image'), plt.xticks([]), plt.yticks([])
    plt.subplot(122), plt.imshow(edges, cmap='gray')
    plt.title('Edge Image'), plt.xticks([]), plt.yticks([])
    plt.show()


if __name__ == "__main__":
    main()
