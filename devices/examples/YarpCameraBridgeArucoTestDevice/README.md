# YARPCameraBridgeArucoTestDevice

The **YARPCameraBridgeArucoTestDevice** is an example YARP device based on `YarpCameraBridge` and `ArucoDetector` that demonstrates the use of `YarpCameraBridge` through aruco marker detection in a simulation environment.
The use-case is demonstrated on `iCubGazeboV3` which is equipped with a RGBD camera on its chest.
In this example, we access this camera through the `YarpCameraBridge` and perform aruco marker detection.

## Prerequisites
- Ensure `iCubGazeboV3` model in `icub-models` folder is an updated model containing the `realsense_chest_depth` and `realsense_chest_rgb` tags.
- Follow the instructions in `./gazebo/models` folder to add aruco markers in Gazebo environment.
- After loading the `iCubGazeboV3 (no hands)` model in the Gazebo environment, ensure, that the ports `/icubSim/depthCamera/rgbImage:o` and `/icubSim/depthCamera/depthImage:o` exist, by running `yarp name list`. One could also additionally check the image outputs by using `yarpview` connections to the ports.

## :running: How to use the device


- Launch `yarprobotinterface` on the robot.

  A RGBD client device is automatically opened to access the simulated cameras and is passed onto the YarpCameraBridge. The necessary configuration is already available in `app` folder. The user simply needs to launch the device using the following command
  - **iCubGazeboV3**

    ```
    YARP_ROBOT_NAME=iCubGazeboV3 yarprobotinterface --config launch-yarp-camera-bridge-aruco-test-device.xml
    ```
If the device was properly installed, then the launching must be succesful, resulting in the popping up of three OpenCV windows displaying the RGB image, depth image and image with aruco marker detected. Please note that sometimes, the image with aruco marker detected may not appear or may appear slowly, which will be the case when there are no markers detected in the current image view.

- Press Ctrl+c to close the device, and the dataset is stored as the device is closed.



