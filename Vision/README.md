# Lab 8: Vision Lab

## The x, y distance of the unknown cones?
(0.6, 0.125)

## Lane Detection Result Image
![lane_detection](lane/images/lane_contours.png)

## Integrated Object Detection + Distance Calculation Result Image
![objec_detection](imgs/test_car_x60cm_output.png)

## Nerual Network Training & Testing Loss Plot
![yolo_loss](imgs/yolo_loss.png)

## Is FP16 faster? Why?
Yes, FP16 is faster because it is half-precision floating point which means it reqeuires less memory footprint and less computational resources for numeric calculations.

## Pipeline Instructions
Use command below to run pipeline on camera stream on vehicle

```python3 run_detection.py```
