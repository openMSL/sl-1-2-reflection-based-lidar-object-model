# Lidar Environmental Effects Strategy

This strategy adds environmental effects due to fog, rain, sun light and road spray to the detections, previously calculated by the Detection Sensing Strategy.

**The code will be published shortly...**

## Fog, Rain, Sun Light

This section is based on

C. Linnhoff, K. Hofrichter, L. Elster, P. Rosenberger, H. Winner, *"Measuring the Influence of Environmental Conditions on Automotive Lidar Sensors,”* submitted to MDPI Sensors Journal, 2022

If you find our work useful in your research, please consider citing:

```
@ARTICLE{linnhoff2022,
author={Linnhoff, Clemens and Hofrichter, Kristof and Elster, Lukas and Rosenberger, Philipp and Winner, Hermann},
journal={MDPI Sensors Journal},
title={Measuring the Influence of Environmental Conditions on Automotive Lidar Sensors},
year={2022}
}
```

## Road Spray

The road spray simulation is based on a methodical data set recorded in real spray conditions on a test track covering multiple vehicle classes, different pavement watering levels and object speeds ranging from 50 km/h to 130 km/h. Modeling parameters are extracted from the recorded data and utilized to calibrate a novel stochastic road spray model. The model is focused on simulating clustering effects appearing in the lidar point cloud due to turbulences in the spray plume. More detail can be found shortly in an upcoming publication, currently under review. The following video shows a sneek preview of the spray simulation with a van overtaking a stationary vehicle at 100 km/h.

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/163b40eee2d73c63dd8acb9b1c5d5a33/Spray.gif" width="800" />

C. Linnhoff, D. Scheuble, M. Bijelic, L. Elster, P. Rosenberger, W. Ritter, D. Dai and H. Winner, *"Simulating Road Spray in Lidar Sensor Models,”* submitted to IEEE Sensors Journal, 2022

If you find our work useful in your research, please consider citing:

```
@ARTICLE{linnhoff2022,
author={Linnhoff, Clemens and Schuble, Dominik and Bijelic, Mario and Elster, Lukas and Rosenberger, Philipp and Ritter, Werner and Dai, Dengxin and Winner, Hermann},
journal={IEEE Sensors Journal},
title={Simulating Road Spray in Lidar Sensor Models},
year={2022}
}
```
