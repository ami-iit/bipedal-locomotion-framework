# JointTorqueControlDevice

The **JointTorqueControlDevice** is a YARP device to run joint torque control and generate motor current set-points.

## :running: How to use the device

### Run the device on a fake robot

The `example` folder contains a simple example on how to run the `JointTorqueControlDevice` to hijack an existing control board device.
The `jtcvcExample.xml` launches four devices:
* `original_controlboard` of type `fakeMotionControlboard`, the original controlboard
* `original_nws_yarp` of type `controlBoard_nws_yarp`, the network wrapper server to expose `original_controlboard` on YARP ports (for example to access it via yarpmotorgui)
* `hijacked_controlboard` of type `JointTorqueControlDevice`, the controlboard wrapped by `JointTorqueControlDevice` to implement the `ITorqueControl` methods on the top of `ICurrentControl`
* `hijacked_nws_yarp` of type `controlBoard_nws_yarp`, the network wrapper server to expose `hijacked_controlboard` on YARP ports (for example to access it via yarpmotorgui)

This permit to show how the hijacking  works. First of all, open three terminals in the `example` directory.

In one, launch the `jtcvcExample.xml` :
~~~
yarprobotinterface --config ./jtcvcExample.xml
~~~

In another, launch the `yarpmotorgui` to monitor the state of `original_controlboard` :
~~~
yarpmotorgui --from ./yarpmotorgui.ini
~~~

In the third, launch the `yarpmotorgui` to monitor the state of `hijacked_controlboard` :
~~~
yarpmotorgui --from ./yarpmotorgui_hijacked.ini
~~~

If everything started correctly, if on the  `hijacked_controlboard` `yarpmotorgui` you change a joint controlmode to torque, you should see that on `original_controlboard`'s `yarpmotorgui` the same joint will switch to current control mode. If you change the desired torque, you should see also the measured current change.

### Run the device on the real robot

To run the device on a robot make sure to define the configuration files for the robot in the `app/robots` folder.

- **ergoCubSN001**

  - Launch `yarprobotinterface` on the robot.
  - launch `JointTorqueControlDevice`
    ```
    YARP_ROBOT_NAME=ergoCubSN001 yarprobotinterface --config launch-joint-torque-control.xml
    ```