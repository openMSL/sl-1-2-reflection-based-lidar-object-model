# OSI Ground Truth Objects to ROS Output Strategy

This strategy outputs OSI global_ground_truth.moving_objects and global_ground_truth.stationary_objects as ROS Marker messages.
It can be used within the [Modular OSMP Framework](https://gitlab.com/tuda-fzd/perception-sensor-modeling/modular-osmp-framework) or any framework compliant model. Using the ROS tool rviz is a convenient way to visualize OSI sensor view input and sensor data output. There are several other [output strategies](https://gitlab.com/tuda-fzd/perception-sensor-modeling/output-strategies) available for different kinds of OSI data.

## Usage
The easiest way to use the strategy within the [Modular OSMP Framework](https://gitlab.com/tuda-fzd/perception-sensor-modeling/modular-osmp-framework) is by including it as a git submodule in *src/model/strategies/*. Navigate to the git repository of your framework or model and use `git submodule add git@gitlab.com:tuda-fzd/perception-sensor-modeling/output-strategies/ros-output-gtobjects-strategy.git src/model/strategies/ros-output-gtobjects-strategy`.

Now you need to add the new strategy to your sequence.conf. In this case of a ROS output strategy, add the name of the strategy in a new line in the *ros_output_sequence.conf* file in the *src/model/strategies/* folder.

In order for the strategy to be called during simulation, the FMI parameter *switch_for_ros_output* needs to be set to *1* and be passed to the framework or model fmu.

## ROS Distributions
This strategy needs a full install of either ROS noetic or ROS melodic. The ROS distribution needs to be located in /opt/ros/.
