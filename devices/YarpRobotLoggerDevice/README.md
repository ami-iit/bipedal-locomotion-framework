# YARPRobotLoggerDevice

The **YARPRobotLoggerDevice** is a YARP device based on `YarpSensorBridge` and [`robometry`](https://github.com/robotology/robometry) that allows logging data from robot sensors and actuators in a mat file.

## Configuration Parameters
The logger is currently supported only for the robots listed in the [application folder](./app/robots). Each robot folder contains:

- `launch-yarp-robot-logger.xml`: Configuration parameters for the `yarprobotinterface` to launch the logger device and associated devices.
- `yarp-robot-logger.xml`: Configuration parameters for the logger device.
- `blf-yarp-robot-logger-interfaces`: Folder containing all interfaces used by the logger device to log data.

## How to Use the Logger
To use the logger, launch the `yarprobotinterface` with the `launch-yarp-robot-logger.xml` configuration file:

```console
yarprobotinterface --config launch-yarp-robot-logger.xml
```
When you close the yarprobotinterface, the logger will save the logged data in a mat file. Additionally, a md file will contain information about the software version in the robot setup. If video recording is enabled, a mp4 file with the video recording will also be generated.

## How to log exogenous data
The `YarpRobotLoggerDevice` can also log exogenous data, i.e., data not directly provided by the robot sensors and actuators. To do this:
1. modify the `yarp-robot-logger.xml` file to specify the exogenous data to log
2. modify the application that streams the exogenous data

### Modification `yarp-robot-logger.xml` configuration file
Modify the [`ExogenousSignalGroup` in the `yarp-robot-logger.xml` file](https://github.com/ami-iit/bipedal-locomotion-framework/blob/a3a8e9cb8a0c3532db81d814d4851009f8134195/devices/YarpRobotLoggerDevice/app/robots/ergoCubSN000/yarp-robot-logger.xml#L27-L37) to log the data streamed by an application that allows the robot to balance:
   ```xml
   <group name="ExogenousSignals">
     <!-- List containing the names of exogenous signals. Each name should be associated to a sub-group -->
     <param name="vectors_collection_exogenous_inputs">("balancing")</param>
     <param name="vectors_exogenous_inputs">()</param>

      <!-- Sub-group containing the information about the exogenous signal "balancing" -->
     <group name="balancing">
        <!-- Name of the port opened by the logger used to retrieve the exogenous signal data -->
        <param name="local">"/yarp-robot-logger/exogenous_signals/balancing"</param>

        <!-- Name of the port opened by the application used to stream the exogenous signal data -->
        <param name="remote">"/balancing-controller/logger"</param>

        <!-- Name of the exogenous signal (this will be the name of the matlab struct containing all the data associated to the exogenous signal) -->
        <param name="signal_name">"balancing"</param>

        <!-- Carrier used in the port connection -->
        <param name="carrier">"udp"</param>
    </group>
   </group>
   ```

### Stream exogenous data
You need to modify the application that streams the exogenous data to open a port with the name specified in the `remote` parameter of the `balancing` sub-group. For example, if you want to stream the data from the your `balancing` application you need to use `BipedalLocomotion::YarpUtilities::VectorsCollectionServer` class as follows
#### C++
If your application is written in C++ you can use the `BipedalLocomotion::YarpUtilities::VectorsCollectionServer` class as follows

```c++
#include <BipedalLocomotion/YarpUtilities/VectorsCollectionServer.h>

class Module
{
    BipedalLocomotion::YarpUtilities::VectorsCollectionServer m_vectorsCollectionServer; /**< Logger server. */
public:
    // all the other functions you need
}
```
The `m_vectorsCollectionServer` helps you to handle the data you want to send and to populate the metadata. To use this functionality, call `BipedalLocomotion::YarpUtilities::VectorsCollectionServer::populateMetadata` during the configuration phase. Once you have finished populating the metadata you should call `BipedalLocomotion::YarpUtilities::VectorsCollectionServer::finalizeMetadata`
```c++
//This code should go into the configuration phase
auto loggerOption = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>(rf);
if (!m_vectorsCollectionServer.initialize(loggerOption->getGroup("LOGGER")))
{
    log()->error("[BalancingController::configure] Unable to configure the server.");
    return false;
}

m_vectorsCollectionServer.populateMetadata("dcm::position::measured", {"x", "y"});
m_vectorsCollectionServer.populateMetadata("dcm::position::desired", {"x", "y"});

m_vectorsCollectionServer.finalizeMetadata(); // this should be called only once
```
In the main loop, add the following code to prepare and populate the data:

```c++
m_vectorsCollectionServer.prepareData(); // required to prepare the data to be sent
m_vectorsCollectionServer.clearData(); // optional see the documentation

// DCM
m_vectorsCollectionServer.populateData("dcm::position::measured", <signal>);
m_vectorsCollectionServer.populateData("dcm::position::desired", <signal>);

m_vectorsCollectionServer.sendData();
```

**Note:** Replace `<signal>` with the actual data you want to log.


#### Python
If your application is written in Python you can use the `BipedalLocomotion.yarp_utilities.VectorsCollectionServer` class as follows
```python
import bipedal_locomotion_framework as blf

class Module:
    def __init__(self):
        self.vectors_collection_server = blf.yarp_utilities.VectorsCollectionServer() # Logger server.
        # all the other functions you need
```
The `vectors_collection_server` helps you to handle the data you want to send and to populate the metadata. To use this functionality, call `BipedalLocomotion.yarp_utilities.VectorsCollectionServer.populate_metadata` during the configuration phase. Once you have finished populating the metadata you should call `BipedalLocomotion.yarp_utilities.VectorsCollectionServer.finalize_metadata`
```python
#This code should go into the configuration phase
logger_option = blf.parameters_handler.StdParametersHandler()
logger_option.set_parameter_string("remote", "/test/log")
if not self.vectors_collection_server.initialize(logger_option):
    blf.log().error("[BalancingController::configure] Unable to configure the server.")
    raise RuntimeError("Unable to configure the server.")

# populate the metadata
self.vectors_collection_server.populate_metadata("dcm::position::measured", ["x", "y"])
self.vectors_collection_server.populate_metadata("dcm::position::desired", ["x", "y"])

self.vectors_collection_server.finalize_metadata() # this should be called only once when the metadata are ready
```
In the main loop, add the following code to prepare and populate the data:
```python
self.vectors_collection_server.prepare_data() # required to prepare the data to be sent
self.vectors_collection_server.clear_data() # optional see the documentation
self.vectors_collection_server.populate_data("dcm::position::measured", <signal>)
self.vectors_collection_server.populate_data("dcm::position::desired", <signal>)
self.vectors_collection_server.send_data()
```
**Note:** Replace `<signal>` with the actual data you want to log.

## How to visualize the logged data
To visualize the logged data you can use [robot-log-visualizer](https://github.com/ami-iit/robot-log-visualizer). To use the `robot-log-visualizer` you can follow the instructions in the [README](https://github.com/ami-iit/robot-log-visualizer/blob/main/README.md) file.

Once you have installed the `robot-log-visualizer` you can open it from the command line with the following command:
```console
robot-log-visualizer
```
Then, you can open the mat file generated by the logger and explore the logged data as in the following video:

[robot-log-visualizer.webm](https://github.com/ami-iit/robot-log-visualizer/assets/16744101/3fd5c516-da17-4efa-b83b-392b5ce1383b)
