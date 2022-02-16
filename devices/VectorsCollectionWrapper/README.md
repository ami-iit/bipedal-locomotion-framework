# VectorsCollectionWrapper

 A Yarp Device called `VectorsCollectionWrapper` that connects and reads from arbitrary **vector** ports on the network and populates the VectorsCollection structure and streams the relevant data in a single port.
For example, this can be useful in recording two streams of data coming from different sources, say Gazebo base state and estimator base state, collect them in a VectorsCollection port and stream which in turn can be sent as input to the `YarpRobotLoggerDevice` to log the base state from two streams.



An example configuration flle is as follows,

The configuration of the device wrapping two ports existing on the YARP network with the names `/icubSim/floating_base/state:o` and  `/icubSim/head/state:o` will be as simple as,
``` xml
<?xml version="1.0" encoding="UTF-8" ?>
<device  xmlns:xi="http://www.w3.org/2001/XInclude" name="vectors-collection-wrapper" type="VectorsCollectionWrapper">
  <param name="sampling_period_in_s">0.01</param>
  <param name="port_prefix">/vcWrapper</param>
  <param name="remote_port_names">("/icubSim/torso/state:o", "/icubSim/head/state:o")</param>
  <param name="remote_var_names">("sim_torso_state", "sim_head_state")</param>
  <param name="output_port_name">"/vcWrapper/upper_body_state"</param>
</device>
```



We can find an example  launch configuration for `iCubGazeboV3` in the `app`folder.

The device can be launched using the command,

``` sh
yarprobotinterface --config launch-vectors-collection-wrapper.xml
```



If launched successfully, an output port  `/vcWrapper/upper_body_state` should stream the collection.
