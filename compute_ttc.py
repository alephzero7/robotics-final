import cv2
import numpy as np


# import cv2.optflow.calcOpticalFlowSparseToDense as dense_optical_flow


# def compute_flow():
def compute_ttc(flow):
    """
    flow: a tensor with dimensions (x_dim, y_dim, 2), where the 0 index in the third dimension is the x value
    and the 1 index is the y value
    """
    y_flow = flow[:, :, 1]
    print(y_flow.shape)

    # create matrix where center line has value o f0
    # these are the pixel values
    y_matrix = np.mgrid[-240:240, -320:320][0]
    #     print(y_matrix)

    ttc_matrix = -1 * np.divide(y_matrix, y_flow)

    return ttc_matrix


def main():
    # Possibly try 0, 1, 2 ...
    camera = 0

    image_size = (640, 480)

    # lk_params = dict(winSize=(15, 15),
    #                  maxLevel=2,
    #                  criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

    # lk_params = dict(pyr_scale=0.5,
    #                  levels=3,
    #                  winsize=15,
    #                  iterations=3,
    #                  poly_n=5,
    #                  poly_sigma=1.2,
    #                  flags=0)
    params = [0.5, 3, 15, 3, 5, 1.2, 0]

    cap = cv2.VideoCapture(camera)

    ret, old_frame = cap.read()

    # resize frame and convert to gray
    old_frame = cv2.cvtColor(cv2.resize(old_frame, image_size), cv2.COLOR_BGR2GRAY)

    # Display the resulting frame
    cv2.imshow('frame', old_frame)

    while (True):

        # Hit q to quit.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            # Release the capture
            break

        ret, new_frame = cap.read()

        # Display the resulting frame
        cv2.imshow('frame', new_frame)

        if ret == False:
            break
        else:
            # resize frame and convert to gray
            new_frame = cv2.cvtColor(cv2.resize(new_frame, image_size), cv2.COLOR_BGR2GRAY)

            # flow = dense_optical_flow(old_frame, new_frame, None)
            flow = cv2.calcOpticalFlowFarneback(old_frame, new_frame, None, *params)
            print(np.mean(flow))

            #             print(flow)

            ttc = compute_ttc(flow)
            print(np.mean(np.abs(ttc)))

            # compute time to collision from optical flow values

            old_frame = new_frame

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
