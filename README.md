# Nose Tracking System
This ROS-system based project tracks nose tip using MediaPipe and then plots it's pixel-space coordinates onto Matlibplot plot 

## Requirements
- ROS Noetic
- OpenCV (`opencv-python`)
- Matplotlib (`matplotlib`)
- Python 3
- MediaPipe (`mediapipe`)

## System Overview
- `nose_detection_node.py` - Captures frames from a webcam (default is `/dev/video0`), tracks the nose tip using MediaPipe and publishes it's pixel-space coordinates to a ROS topic
- `nose_plotter_node.py` - Subscribes to said topic and plots it onto Matlibplot plot

## How to Run

1. **Install Required Dependencies**
- Make sure ROS Noetic is installed and your catkin workspace is properly configured
- Install the required Python dependencies:
    ```bash
    pip install opencv-python mediapipe matplotlib
    ```

2. **Build the Workspace**  
    Navigate to your catkin workspace and build the packages:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```
3. **Run nodes**
- Separately
    ```bash
    rosrun nose_tracking face_detection_node.py
    rosrun nose_tracking nose_plotter_node.py
    ```
- Or run everything with roslaunch
    ```bash
    roslaunch nose_tracking nose_tracking.launch
    ```
    You can also specify parameters using the launch file:
    - specify a different camera device (e.g. `/dev/video1`)
         ```bash
        roslaunch nose_tracking nose_tracking.launch video_device:=/dev/video1
         ```
    - specify how many points are shown on a plot (`0` for showing all received points)
        ```bash
        roslaunch nose_tracking nose_tracking.launch how_many_plot:=0
        ```
## Additional Notes
- Designed for tracking only a single nose at a time 
- Matlibplot plot may lag behind at higher rates



