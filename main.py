# Code adapted from: https://github.com/bitcraze/crazyflie-lib-python/blob/master/examples/autonomousSequence.py

import time
# CrazyFlie imports:
import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
from cflib.crazyflie.syncCrazyflie import SyncCrazyflie
from cflib.crazyflie.syncLogger import SyncLogger
from cflib.positioning.position_hl_commander import PositionHlCommander
import numpy as np
import cv2
import matplotlib.pyplot as plt

# DECLARE CONSTANTS AND IMPORTS

course_width = 1.4
# course_width = 1.0 # FAKE
group_number = 16

# Possibly try 0, 1, 2 ...
camera = 1

DRONE_HEIGHT = 0.4  # The height in meters
X_ADJUST = 0.05
Y_ADJUST = 0.05  # The amount to shift Y in meters
OBSTACLE_PART_LENGTH = 2.8  # The length of the obstacle portion of the course
PIPE_DIAMETER = 0.1


def mask_frame(frame):
    blurred = cv2.medianBlur(frame, 5)

    # Better Edge detection
    hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
    gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)

    # Do the red mask
    lower_red = np.array([0, 40, 40])
    upper_red = np.array([20, 255, 255])
    mask0 = cv2.inRange(hsv, lower_red, upper_red)

    lower_red = np.array([160, 40, 40])
    upper_red = np.array([190, 255, 255])
    mask1 = cv2.inRange(hsv, lower_red, upper_red)

    mask = mask0 + mask1

    output_gray = gray.copy()
    output_gray[np.where(mask == 0)] = 0

    contours, _ = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    new_contours = []

    for contour in contours:
        if cv2.contourArea(contour) > 500:
            new_contours.append(contour)
        else:
            for pixel in contour:
                row = pixel[0][1]
                col = pixel[0][0]
                output_gray[row][col] = 0

    cv2.drawContours(output_gray, new_contours, -1, 255, 3)

    return output_gray


## Some helper functions:
## -----------------------------------------------------------------------------------------

# Ascend and hover:
def set_PID_controller(cf):
    # Set the PID Controller:
    print('Initializing PID Controller')
    cf.param.set_value('stabilizer.controller', '1')
    cf.param.set_value('kalman.resetEstimation', '1')
    time.sleep(0.1)
    cf.param.set_value('kalman.resetEstimation', '0')
    time.sleep(2)
    return


# Ascend and hover:
def ascend_and_hover(cf):
    # Ascend:
    STEP_COUNT = 10
    for y in range(int(DRONE_HEIGHT * STEP_COUNT)):
        cf.commander.send_hover_setpoint(0, 0, 0, y / STEP_COUNT)
        time.sleep(0.1)
    # Hover at 1.0 meters:
    for _ in range(20):
        cf.commander.send_hover_setpoint(0, 0, 0, DRONE_HEIGHT)
        time.sleep(0.1)
    return


def findGreatestContour(contours):
    largest_area = 0
    largest_contour_index = -1
    i = 0
    total_contours = len(contours)

    while i < total_contours:
        area = cv2.contourArea(contours[i])
        if area > largest_area:
            largest_area = area
            largest_contour_index = i
        i += 1

    return largest_area, largest_contour_index


def check_contours(frame):
    # Do the contour detection on the input frame
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    # Compute
    mask0 = cv2.inRange(hsv, np.array([0, 40, 40]), np.array([20, 255, 255]))
    mask1 = cv2.inRange(hsv, np.array([160, 40, 40]), np.array([190, 255, 255]))
    test_mask = mask0 + mask1
    contours, _ = cv2.findContours(test_mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    largest_area, largest_contour_index = findGreatestContour(contours)

    if largest_area > 20000:
        print(largest_area)

        largest_contour = contours[largest_contour_index]
        center = np.mean(largest_contour, axis=0)  # Should give [x, y]
        #         print(center)
        if (240 <= center[0][0] <= 320):
            return True, -Y_ADJUST
        elif (320 <= center[0][0] <= 400):
            return True, Y_ADJUST
        else:
            return False, 0
    return False, 0


# Follow the setpoint sequence trajectory:
def adjust_position(cf, current_y, y_adjust, current_x):
    #     print('Adjusting position')

    current_y = current_y + y_adjust
    position = [current_x, current_y, DRONE_HEIGHT, 0.0]

    #     print('Setting position {}'.format(position))
    for i in range(10):
        cf.commander.send_position_setpoint(position[0],
                                            position[1],
                                            position[2],
                                            position[3])
        time.sleep(0.01)

    # cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    return current_y


def move_forward(cf, current_x, x_adjust, current_y):
    #     print('Move Forward')

    current_x = current_x + x_adjust
    position = [current_x, current_y, DRONE_HEIGHT, 0.0]

    #     print('Setting position {}'.format(position))
    for i in range(10):
        cf.commander.send_position_setpoint(position[0],
                                            position[1],
                                            position[2],
                                            position[3])
        time.sleep(0.01)

    # cf.commander.send_stop_setpoint()
    # Make sure that the last packet leaves before the link is closed
    # since the message queue is not flushed before closing
    time.sleep(0.1)
    return current_x


# Hover, descend, and stop all motion:
def hover_and_descend(cf):
    print('Descending:')
    # Hover at DRONE_HEIGHT meters:
    for _ in range(30):
        cf.commander.send_hover_setpoint(0, 0, 0, DRONE_HEIGHT)
        time.sleep(0.1)
    # Descend:
    for y in range(25):
        cf.commander.send_hover_setpoint(0, 0, 0, (DRONE_HEIGHT - y / 25))
        time.sleep(0.1)
    # Stop all motion:
    for i in range(10):
        cf.commander.send_stop_setpoint()
        time.sleep(0.1)
    return


## -----------------------------------------------------------------------------------------

## The following code is the main logic that is executed when this script is run

## -----------------------------------------------------------------------------------------
# Set the URI the Crazyflie will connect to
uri = f'radio://0/{group_number}/2M'

# Initialize all the CrazyFlie drivers:
cflib.crtp.init_drivers(enable_debug_driver=False)

# Scan for Crazyflies in range of the antenna:
print('Scanning interfaces for Crazyflies...')
available = cflib.crtp.scan_interfaces()

# List local CrazyFlie devices:
print('Crazyflies found:')
for i in available:
    print(i[0])

# Check that CrazyFlie devices are available:
if len(available) == 0:
    print('No Crazyflies found, cannot run example')
else:
    ## Ascent to hover; run the sequence; then descend from hover:
    # Use the CrazyFlie corresponding to team number:
    with SyncCrazyflie(uri, cf=Crazyflie(rw_cache='./cache')) as scf:
        # Get the Crazyflie class instance:
        cf = scf.cf
        current_y = 0.0
        current_x = 0.0

        # Initialize and ascend:
        t = time.time()
        elapsed = time.time() - t
        ascended_bool = 0
        override_y_adjust = 0
        camera = 1

        # store previous adjust value
        y_adjust_prev = 0
        # store x pos when drone last moved left or right
        x_pos_prev = 0.0

        cap = cv2.VideoCapture(camera)
        while (cap.isOpened()):

            ret, frame = cap.read()

            elapsed = time.time() - t
            if (elapsed > 5.0):

                #                 print('Capturing.....')

                if ret:

                    # Show camera

                    plt.figure()
                    plt.imshow(mask_frame(frame))

                    b, g, r = cv2.split(frame)  # get b,g,r
                    rgb_frame = cv2.merge([r, g, b])  # switch it to rgbplt.figure()
                    plt.figure()
                    plt.imshow(rgb_frame)
                    #                     continue
                    # Drone starts by ascending
                    if (ascended_bool == 0):
                        set_PID_controller(cf)
                        ascend_and_hover(cf)
                        ascended_bool = 1
                    else:

                        # If object to move around
                        suc, y_adjust = check_contours(frame)

                        if (suc):

                            new_y = current_y + y_adjust

                            # If at edge, move towards middle
                            if override_y_adjust != 0:
                                print('Moving w/ override_y_adjust:', override_y_adjust)
                                current_y = adjust_position(cf, current_y, override_y_adjust, current_x)

                                # Stop overriding once obstacle passes middle of camera
                                # so that override direction matches that of y_adjust
                                if override_y_adjust * y_adjust > 0:
                                    override_y_adjust = 0

                            # Left out of bounds
                            elif new_y > course_width / 2:
                                override_y_adjust = -Y_ADJUST
                            # Right out of bounds
                            elif new_y < -course_width / 2:
                                override_y_adjust = Y_ADJUST

                            # check to see if we have moved recently to left or right
                            # if we haven't moved sufficiently enough forward to clear an obstacle
                            # cannot go back in opposite direction
                            delta_x = current_x - x_pos_prev
                            if y_adjust * y_adjust_prev < 0 and (delta_x < 0.50):
                                print(
                                    'y_adjust of {} and y_adjust_prev of {} are in opposite directions'.format(y_adjust,
                                                                                                               y_adjust_prev))
                                print('current_x - x_pos_prev = {}'.format(delta_x))
                                print('moving by - y_adjust instead')
                                current_y = adjust_position(cf, current_y, -y_adjust, current_x)

                            # Move away from obstacle
                            else:
                                current_y = adjust_position(cf, current_y, y_adjust, current_x)

                                # store y_adjust value
                                y_adjust_prev = y_adjust
                                # store x position
                                x_pos_prev = current_x
                                print('Current x: {}'.format(x_pos_prev))

                                if y_adjust_prev < 0:
                                    print('Moved right')
                                elif y_adjust_prev > 0:
                                    print('Moved left')

                        #                                 print('largest contour area: {}'.format())

                        # Move forward
                        else:
                            current_x = move_forward(cf, current_x, X_ADJUST, current_y)

                            if current_x > OBSTACLE_PART_LENGTH:
                                print('Drone is in end zone!')
                                break

        ### Book detection
        run_book = False
        if run_book:
            # Center y
            current_y = adjust_position(cf, current_y, -current_y, current_x)
            # Maybe adjust height if needed

            # Detect blue contours
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask0 = cv2.inRange(hsv, np.array([90, 50, 50]), np.array([110, 255, 255]))

            contours, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            largest_area, largest_contour_index = findGreatestContour(contours)
            largest_contour = contours[largest_contour_index]
            center = np.mean(largest_contour, axis=0)  # Should give [x, y]

            # Move towards contour with adjust_position
            while center[1] > 330 or center[1] < 310:
                y_adjust = 0.05 if center[1] < 320 else -0.05
                current_y = adjust_position(cf, current_y, y_adjust, current_x)

            # Move forward until contour becomes large enough
            # Add while loop that runs while contour too small
            while largest_area < 1000:
                current_x = move_forward(cf, current_x, X_ADJUST, current_y)

                # update contour
                contours, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
                largest_area, _ = findGreatestContour(contours)

            print('Drone is close to target book')
            print(largest_area)

            # Land

        cap.release()

        # Descend and stop all motion:
        hover_and_descend(cf)

print('Done!')
