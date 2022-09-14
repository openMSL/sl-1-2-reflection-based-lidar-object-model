# Detections to ROS Output Strategy

This strategy outputs osi3::LidarDetectionData and osi3::RadarDetectionData as ROS PointCloud2 messages.
It can be used within the [Modular OSMP Framework](https://gitlab.com/tuda-fzd/perception-sensor-modeling/modular-osmp-framework).
The easiest way to use it within the Framework for your own models is by including it as a git submodule in *src/model/strategies/*

