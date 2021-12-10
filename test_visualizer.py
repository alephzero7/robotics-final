import cv2
import time
import numpy as np
from IPython.display import HTML, Image

from matplotlib import pyplot as plt
from matplotlib import animation
from typing import List, Tuple

# Define globals
image_size = (640, 480)
file_name = 'video_sample.avi'

def mask_frame(frame):
    # lb = (120, 0, 220)
    # lb = (145, 3, 250)
    # ub = (155, 40, 255)
    # ub = (180, 60, 255)

    blurred = cv2.medianBlur(frame, 5)

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
    maxim_row = 0
    minim_row = 1000

    for contour in contours:
        if cv2.contourArea(contour) > 1000:
            new_contours.append(contour)
        else:
            for pixel in contour:
                row = pixel[0][1]
                col = pixel[0][0]
                output_gray[row][col] = 0

    cv2.drawContours(output_gray, new_contours, -1, 255, 3)

    # cv2.imshow('Frame', output_gray)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    return output_gray

def load_video(name):
    frames = []
    cap = cv2.VideoCapture(name)
    fps = cap.get(cv2.CAP_PROP_FPS)
    print("Reading Frames")
    while cap.isOpened():
        ret, frame = cap.read()

        if ret == False:
            break
        else:
            frames.append(frame)

    cap.release()

    # We'll change this to do the contouring instead
    I = np.zeros([image_size[1], image_size[0], len(frames)])
    print("Processing Frames")
    for t in range(len(frames)):
        if t % 10 == 0:
            print(t)
        # I[:, :, t] = cv2.cvtColor(cv2.resize(
        #     frames[t], image_size), cv2.COLOR_BGR2GRAY) / 255
        smaller_image = cv2.resize(frames[t], image_size)
        I[:, :, t] = mask_frame(smaller_image)
        # cv2.resize(frame, (640, 480))

    return I, fps

def create_gif(I, output_dir, frame_rate=20):
    fig, ax = plt.subplots()
    im = ax.imshow(I[:, :, 0], cmap='gray')

    def init():
        return [im]

    def animate(i):
        ax.clear()
        im = ax.imshow(I[:, :, i], cmap='gray')

        return [im]

    anim = animation.FuncAnimation(
        fig, animate, init_func=init, frames=I.shape[2], interval=100, blit=True)

    anim.save(output_dir, writer='pillow', fps=frame_rate)
    Image(url=output_dir)


def main():

    camera = 0
    cap = cv2.VideoCapture(camera)
    while(cap.isOpened()):

        ret, frame = cap.read()

        elapsed = time.time() - t
        if(elapsed > 5.0):

            print('Capturing.....')

            if ret:
                # cv2.imshow('frame',frame)
                # if(check_contours(frame)):
                #     current_y = adjust_position(cf, current_y)
                pass

        if(elapsed > 10.0):
            break

    cap.release()

def build_gif():

    plt.rcParams['animation.ffmpeg_path'] = '/usr/bin/ffmpeg'
    print('Loading video!')
    I, frame_rate = load_video(file_name)

    print('Making GIF!')
    create_gif(I, './anim2.gif', frame_rate)
    print('GIF made!')
    return


if __name__ == "__main__":
    build_gif()
    # main()
