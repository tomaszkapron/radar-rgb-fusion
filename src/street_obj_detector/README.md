# street_obj_detector
<!-- Required -->
<!-- Package description -->
Package provides wrapper for ROS2 around ultralitics YOLOv8 object detector. It is used to detect objects on the road, such as pedestrians, cars, traffic signs, etc. It publishes ROS2 messages with detected objects and their bounding boxes.

## Installation
<!-- Required -->
<!-- Things to consider:
    - How to build package? 
    - Are there any other 3rd party dependencies required? -->

```bash
colcon build --symlink-install --packages-up-to street_obj_detector
```

## Usage
<!-- Required -->
<!-- Things to consider:
    - Launching package. 
    - Exposed API (example service/action call. -->

```bash
ros2 launch street_obj_detector street_obj_detector.launch.py
```

## API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->

### Input

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/camera/image_raw` | sensor::msg::Image | Image being input for detection |

### Output

| Name         | Type                  | Description  |
| ------------ | --------------------- | ------------ |
| `/inference_image` | sensor::msg::Image | Image with detection bounding boxes |
| `/yolov8_inference` | yolov8::msg::Yolov8Inference | Servo position message for VESC in (0 - 1) range where 0.5 is straight |


### Parameters

| Name         | Type | Description  |
| ------------ | ---- | ------------ |
| `input_image_topic` | string  | image topic to be processed #TODO  |

## References / External links
<!-- Optional -->
- about ultralitics YOLOv8 [link](https://docs.ultralytics.com/usage/python/)
