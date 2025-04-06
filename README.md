# Nose Tracking System
This ROS-system based project tracks nose tip using MediaPipe and then plots it's pixel-space coordinates onto Matlibplot plot 

## Requirements
- ROS Noetic
- OpenCV (`opencv-python`)
- Matplotlib (`matplotlib`)
- Python 3
- MediaPipe (`mediapipe`)

## How to Run the System

###  Install Required Dependencies
- Make sure ROS Noetic is installed and your catkin workspace is properly configured
- Install the required Python dependencies:
    ```bash
    pip install opencv-python mediapipe matplotlib
    ```

### Steps to Run

1. **Build the Workspace**  
    Navigate to your catkin workspace and build the packages:
    ```bash
    cd ~/catkin_ws
    catkin_make
    source devel/setup.bash
    ```

2. **Run the Face Detection Node**  
    Open a terminal and execute:
    ```bash
    rosrun nose_tracking face_detection_node.py
    ```

3. **Run the Plotting Node**  
    Open another terminal and execute:
    ```bash
    rosrun nose_tracking nose_plotter_node.py
    ```

4. **System Workflow**  
    - The `face_detection_node.py` will start publishing nose tip coordinates to a `\face_coordinates` ROS topic .
    - The `nose_plotter_node.py` will subscribe to this topic and show the nose trajectory in a live updating matplotlib plot.

