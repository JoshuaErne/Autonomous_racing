import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import glob


def find_distance(y_cam, H_mount, calibMat):
    """
    Finds the estimated distance of a specific pixel coordinate.

    :param: y_cam: float, the row of the pixel in the camera (v)
    :param: H_mount: float, Pre-computed camera mounting height
    :param: calibMat: 3x3 ndarray, pre-computed camera intrinsics matrix

    :returns: X_car, the frontal distance of the pixel from the camera
    """
    fy = calibMat[1,1]
    y0 = calibMat[1,-1] 
    X_car = (fy * H_mount)/(y_cam - y0)
    return X_car


def estimateHmount(calibMat, distance, y_cam):
    """
    Computes an estimate of the camera's mounting height, given a known pixel's ground-truth distance

    :param: calibMat: 3x3 ndarray, pre-computed camera intrinsics matrix
    :param: distance: float, the ground-truth frontal distance of the pixel to the camera
    :param: y_cam: float, the row of the pixel in the camera (v)

    :returns: h_mount, float, the camera's mounting height
    """
    fy = calibMat[1,1]
    y0 = calibMat[1,-1] 
    h_mount = (y_cam - y0) * (distance/fy)
    return h_mount

if __name__ == "__main__":
    image_path = 'cone_x40cm.png'
    calibMat   = np.loadtxt("../calibration/calibration_matrix.txt")

    distance   = 0.4

    x_cam_cone = 666
    y_cam_cone = 495

    correspondonce_image = cv.imread(image_path)
    correspondonce_image = cv.cvtColor(correspondonce_image, cv.COLOR_BGR2RGB)
    correspondonce_image = cv.circle(correspondonce_image, (x_cam_cone, y_cam_cone), 10, (255,0,0), 2)
    plt.imshow(correspondonce_image)
    plt.show()

    H_mount = estimateHmount(calibMat, distance, y_cam_cone)
    np.savetxt('h_mount.txt', np.array([H_mount]))
    print("Estimated Height (H_mount):", H_mount)

    cone_u = cv.imread('cone_unknown.png')
    cone_u = cv.cvtColor(cone_u, cv.COLOR_BGR2RGB)

    x_cam_unkown = 597
    y_cam_unkown = 415
    cone_u = cv.circle(cone_u, (x_cam_unkown,y_cam_unkown), 10, (255,0,0), 2)
    plt.imshow(cone_u)
    plt.show()

    distance = find_distance(y_cam_unkown, H_mount, calibMat)
    print("Estimated Distance:", distance)
