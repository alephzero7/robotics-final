from main import START_POSITION, adjust_position


print('BEFORE ASCEND TO TABLE HEIGHT')
for _ in range(10):
    cf.commander.send_hover_setpoint(0, 0, 0, 0.85)
    time.sleep(0.1)
time.sleep(0.11)

print('BEFORE MOVING LEFT')
for _ in range(10):
    current_y = adjust_position(
        cf, current_y, -current_y - START_POSITION + 0.1, current_x, 0.85)
    time.sleep(0.1)

time.sleep(0.1)
print('AFTER MOVED LEFT AND AT TABLE HEIGHT')

while True:
    ret, frame = cap.read()

    if ret:  # Detect blue contours
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(hsv, np.array(
            [90, 50, 50]), np.array([110, 255, 255]))

        contours, _ = cv2.findContours(
            mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        largest_area, largest_contour_index = findGreatestBookContour(contours)
        # print('largest area: {}'.format(largest_area))
        if largest_contour_index == -1:
            current_y = adjust_position(cf, current_y, -0.05, current_x, 0.85)
            continue
        # If it reaches here, it deteceted a valid contour index
        largest_contour = contours[largest_contour_index]
        center = np.mean(largest_contour, axis=0)[0]  # Should give [x, y]
        if center[0] > 380:
            break
        plt.figure()
        plt.scatter([center[0]], [center[1]], marker="x", color="red", s=200)
        plt.imshow(mask_frame(frame, 'blue'))
        plt.figure()
        plt.imshow(frame)

    break
######
largest_contour_index = -1

# read a frame
while largest_contour_index == -1:
    ret, frame = cap.read()

    if ret:

        # Detect blue contours
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask0 = cv2.inRange(hsv, np.array(
            [90, 50, 50]), np.array([110, 255, 255]))

        contours, _ = cv2.findContours(
            mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        largest_area, largest_contour_index = findGreatestBookContour(contours)
        print('largest area: {}'.format(largest_area))

largest_contour = contours[largest_contour_index]
center = np.mean(largest_contour, axis=0)[0]  # Should give [x, y]

plt.figure()
plt.scatter([center[0]], [center[1]], marker="x", color="red", s=200)
plt.imshow(frame)
plt.figure()
plt.imshow(mask_frame(frame, 'blue'))

# Move towards contour with adjust_position
while center[0] > 330 or center[0] < 310:
    largest_contour_index = -1

    print('Center: {}'.format(center))

    y_adjust = 0.05 if center[0] < 320 else -0.05
    if y_adjust > 0:
        print('moving left')
    else:
        print('moving right')
    current_y = adjust_position(cf, current_y, y_adjust, current_x, 0.85)

    ret, frame = cap.read()

    while not ret:
        ret, frame = cap.read()

    # Detect blue contours
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask0 = cv2.inRange(hsv, np.array([90, 50, 50]), np.array([110, 255, 255]))

    contours, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

    #                 largest_area, largest_contour_index = findGreatestBookContour(contours)
    #                 if largest_contour_index == -1:
    #                     continue
    while largest_contour_index == -1:
        ret, frame = cap.read()

        if ret:

            # Detect blue contours
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask0 = cv2.inRange(hsv, np.array(
                [90, 50, 50]), np.array([110, 255, 255]))

            contours, _ = cv2.findContours(
                mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

            largest_area, largest_contour_index = findGreatestBookContour(
                contours)
    print('largest area: {}'.format(largest_area))

    largest_contour = contours[largest_contour_index]
    center = np.mean(largest_contour, axis=0)[0]  # Should give [x, y]

    plt.figure()
    plt.scatter([center[0]], [center[1]], marker="x", color="red", s=200)
    plt.imshow(mask_frame(frame, 'blue'))
    plt.figure()
    plt.imshow(frame)

print('Centered on the book')
# Move forward until contour becomes large enough
# Add while loop that runs while contour too small

while largest_area < 5000:
    break
    print('largest area: {}'.format(largest_area))

    _, _, z_est = position_estimate(scf)
    print('z estimate: {}'.format(z_est))
    # if z_est < 0.5:
    #     current_x = move_forward(cf, current_x, X_ADJUST, current_y, 0.15)
    # else:
    #     current_x = move_forward(cf, current_x, X_ADJUST, current_y, 0.85)
    # current_x = move_forward(cf, current_x, X_ADJUST, current_y, z_est)
    if current_x < 3.0:
        current_x = move_forward(cf, current_x, X_ADJUST, current_y, 0.85)
    else:
        current_x = move_forward(cf, current_x, X_ADJUST, current_y, 0.15)

    # update contour
    ret, frame = cap.read()

    while not ret:
        ret, frame = cap.read()

    plt.figure()
    plt.imshow(mask_frame(frame, 'blue'))

    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    mask0 = cv2.inRange(hsv, np.array([90, 50, 50]), np.array([110, 255, 255]))

    contours, _ = cv2.findContours(mask0, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    largest_area, _ = findGreatestBookContour(contours)

print('Drone is close to target book')
print(largest_area)
