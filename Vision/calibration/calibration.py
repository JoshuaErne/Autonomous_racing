import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import glob


def calibrateCamera(imageDir, saveData=True, plotting=False, logging=True):
    """
    Returns the intrinisics matrix  and distortion coefficients derived from a glob of images.

    :param: imageDir: String, the path of the folder containing all checkerboard png images.
    :param: saveData: boolean, set to True if you wish to save calibration matrix and distortion coefficients.
    :param: plotting: boolean, set to True if you wish to visualize the detected checkerboard corners.
    :param: logging : boolean, set to True if you wish to print the output of calibration.

    :returns: mtx: 3x3 ndarray, intrinsics of the camer; dist, 1x5 ndarray of distortion coefficients of the camera
    """
    # termination criteria
    criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
    objp = np.zeros((6*8,3), np.float32)
    objp[:,:2] = np.mgrid[0:8,0:6].T.reshape(-1,2)

    # Arrays to store object points and image points from all the images.
    objpoints = [] # 3d point in real world space
    imgpoints = [] # 2d points in image plane.
    images = glob.glob(imageDir + '/*.png')
    for fname in images:
        img = cv.imread(fname)
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (8,6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)
            corners2 = cv.cornerSubPix(gray,corners, (11,11), (-1,-1), criteria)
            imgpoints.append(corners)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (8,6), corners2, ret)
            if(plotting):
                plt.imshow(img)
                plt.show()

    ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    if(logging):
        print('RMS Re-Projection Error:',ret)
        print('Calibration Matrix: \r\n', str(mtx))
        print('Distortion Coefficients:', dist)
        print('Rotation Vectors:')
        for rvec in rvecs:
            print(rvec)
            print('\r\n')
        print('Translation Vectors:')
        for tvec in tvecs:
            print(tvec)
            print('\r\n')
    
    if(saveData):
        np.savetxt('calibration_matrix.txt', mtx)
        np.savetxt('distortion_coeffs.txt',dist)

    return mtx, dist


if __name__ == '__main__':
    calibrateCamera('.', True, False, True)
