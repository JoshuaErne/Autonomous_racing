import cv2

from object_detection.yolo import *
from distance.distanceMeasurement import *
from lane.lane_detection import *


def run_on_image(image_path, output_path, detect_lane=False):
    # load image
    image = cv2.imread(image_path)

    # detect lane
    contours = []
    if detect_lane:
        contours = detect_lane(image)

    # preprocess image
    image = image / 255.0

    # load TensorRT engine from file
    engine, context = load_trt_engine("./object_detection/models/model.trt")

    # load calibration data to find distance
    h_mount = np.loadtxt("distance/h_mount.txt")
    calibration_matrix = np.loadtxt("calibration/calibration_matrix.txt")

    # run inference on image
    output = inference(image, engine, context)

    # draw detection on image
    for box in output:
        x1, y1 = 3 * int(box[0] - box[2]/2), 3 * int(box[1] - box[3]/2)
        x2, y2 = 3 * int(box[0] + box[2]/2), 3 * int(box[1] + box[3]/2)
        image = cv2.rectangle(image, (x1, y1), (x2, y2), (0,255,0), 2)

        distance = find_distance(y2, h_mount, calibration_matrix)
        cv2.putText(
            image,
            str(round(distance, 2)) + "m",
            (int(box[0]) * 3, int(box[1] + (box[3] / 2)) * 3 + 30),
            cv2.FONT_HERSHEY_COMPLEX,
            1.0,
            (0, 255, 0),
            2
        )

    # draw lane detection
    cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

    # save output image
    cv2.imwrite(output_path, image * 255.0)

    # display output image
    while True:
        cv2.imshow("Detection Output", image)
        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break
    cv2.destroyAllWindows()


def run_on_camera(detect_lane=False):
    # initialize realsense camera
    cam = cv2.VideoCapture(
        "v4l2src device=/dev/video2 extra-controls=\"c," +
        "exposure_auto=3\" ! video/x-raw, width=960, height=540 !" +
        "videoconvert ! video/x-raw, format=BGR ! appsink"
    )

    # load TensorRT engine from file
    engine, context = load_trt_engine("./object_detection/models/model.trt")

    # load calibration data to find distance
    h_mount = np.loadtxt("distance/h_mount.txt")
    calibration_matrix = np.loadtxt("calibration/calibration_matrix.txt")

    # inference on camera stream
    while True:
        ret, image = cam.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # detect lane
        if detect_lane:
            contours = detect_lane(image)
            cv2.drawContours(image, contours, -1, (0, 255, 0), 3)

        # run inferene on image frame
        image = image / 255.0
        output = inference(image, engine, context)

        # draw detection on image
        for box in output:
            x1, y1 = 3 * int(box[0] - box[2]/2), 3 * int(box[1] - box[3]/2)
            x2, y2 = 3 * int(box[0] + box[2]/2), 3 * int(box[1] + box[3]/2)
            image = cv2.rectangle(image, (x1, y1), (x2, y2), (0,255,0), 2)

            distance = find_distance(y2, h_mount, calibration_matrix)
            cv2.putText(
                image,
                str(round(distance, 2)) + "m",
                (int(box[0]) * 3, int(box[1] + (box[3] / 2)) * 3),
                cv2.FONT_HERSHEY_COMPLEX,
                0.5,
                (0, 255, 0),
                2
            )

        cv2.imshow("Detection Output", image)

        k = cv2.waitKey(1)
        if k%256 == 27:
            # ESC pressed
            print("Escape hit, closing...")
            break

    cam.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    run_on_camera()
    # run_on_image("object_detection/images/test_car_x60cm.png", "imgs/test_car_x60cm_output.png")
