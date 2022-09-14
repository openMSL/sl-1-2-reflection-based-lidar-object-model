# Detected Objects to ROS Output Strategy

This strategy outputs osi3::DetectedMovingObjects as ROS Marker messages.
Using the ROS tool rviz is a convenient way to visualize OSI sensor view input and sensor data output.

## Usage

You need to add the name of the strategy in a new line in the *ros_output_sequence.conf* file in the *src/model/strategies/* folder.

In order for the strategy to be called during simulation, the FMI parameter *switch_for_ros_output* needs to be set to *1* and be passed to the framework or model fmu.

## ROS Distributions

This strategy needs a full install of either ROS noetic or ROS melodic. The ROS distribution needs to be located in /opt/ros/.
