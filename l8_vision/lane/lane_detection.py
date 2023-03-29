import cv2


def detect_lane(image):
    # gaussian blurring
    image_blur = cv2.GaussianBlur(image, (5, 5), 0)
    
    # convert image to hsv space
    image_hsv = cv2.cvtColor(image_blur, cv2.COLOR_BGR2HSV)

    # color filtering
    mask = cv2.inRange(image_hsv, (20, 60, 100), (60, 200, 255))

    # find Canny edges
    edged = cv2.Canny(mask, 170, 200, L2gradient = True)

    # find contours
    contours, hierarchy = cv2.findContours(
        edged, 
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    return contours


def display_contours(image, contours):
    # draw detection
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

    # save result image
    cv2.imwrite('images/lane_contours.png', image)

    # display result
    cv2.imshow('Contours', image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


if __name__ == "__main__":
    image = cv2.imread('images/lane.png')
    contours = detect_lane(image)
    display_contours(image, contours)
