# golain_roscon23_demo

## How to use
- Source Foxy Installation
    ```
    source /opt/ros/foxy/setup.bash
    ```
- Build (inside ros2_ws directory ONLY)
    ```
    colcon build
    ```
- Source Setup Files
    ```
    source install/setup.bash
    ```
- In Terminal 1
    ```
    source install/setup.bash
    ros2 run golain_demo golain_pub
    ```
- In Terminal 2
    ```
    source install/setup.bash
    ros2 run golain_demo golain_sub
    ```