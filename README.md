[![pipeline status](https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/badges/master/pipeline.svg)](https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/-/commits/master) 

:warning: **Current version not compliant with official ASAM OSI**: The current version of the model is build on the enhancements to the Open Simulation Interface from the publicly funded SETLevel project. It is therefore dependent on the non-standard [SL OSI](https://gitlab.setlevel.de/open/osi) and not [ASAM OSI](https://github.com/OpenSimulationInterface/open-simulation-interface).

# Reflection Based Lidar Object Model

<img align="right" src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/object-based-generic-perception-object-model/uploads/17c84e9ec0acf0fac2e35855f038ad0b/fzdlogo.jpg" width="100" />

This is the FZD Reflection Based Lidar Model based on the FZD OSI Sensor Model Packaging Framework.
It is a highly parameterizable sensor system model including detection calculation and object tracking simulation.
The model gets lidar reflection calculated in a simulation tool beforehand e.g. with ray tracing.
The model outputs are lidar detections and detected moving objects.<br><br>

<img src="https://user-images.githubusercontent.com/27010086/148824838-12edae52-fd20-4f4b-938e-fdd3ee8c3544.gif" width="800" />

## Modeling Approach

### Modeling Framework

The outer layer of the model is the [Modular OSMP Framework](https://gitlab.com/tuda-fzd/perception-sensor-modeling/modular-osmp-framework) by FZD.
It specifies ways in which models using the [Open Simulation Interface (OSI)](https://github.com/OpenSimulationInterface/open-simulation-interface) are to be packaged for their use in simulation environments using [FMI 2.0](https://fmi-standard.org).

The actual logic of the model is packed in so called strategies.
This is where the magic happens.
The `apply()` function of the strategy is called by the `do_calc()` function of the Framework.
There are four subsequent strategies as shown in the image below.

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/ab8425c05dd6a6fe3c3d3e82232e5602/image.png" width="800" />

### Modeling of Beam Divergence by Super-Sampling with Rays

The super-sampled reflections that describe the beam-divergence of the simulated lidar sensor are calculated in the simulation tool bevorehand.
As a first step in the here described model, they are sorted per beam.
Next, the rays per beam are sorted by distance to start a peak-detection process, finding as many peaks or echoes as parameterized in the currently active profile.
During that process, a threshold is applied to the reflections, as visualized by the red dashed line in the figure below.
In this figure, the noise floor is depicted in blue.
It is not explicitly modeled, but indirectly covered by the threshold.
Finally, the echo pulse width in m, depicted in the same figure as the widths [A, B] and [C, D], and the intensity for each echo are calculated based on the correctly summarized signal strengths from the corresponding reflections.
Consequently, compared to single-ray-per-beam ray tracing, in partly occlusion scenarios, where the diverging lidar beam hits an object in front and another one behind, two detections can be derived, as it would be the case with a real lidar sensor.<br><br>

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/739aeea64d29a93212b882457dbbcb95/_reflection-based_model.png" width="800" />

The image depicts the super-sampling of a lidar sensor beam and the detection calculation within the reflection-based lidar sensor simulation from [[1](#Rosenberger2020)</sup>, p. 192]. The intervals A-B and C-D mark the resulting echoes at the edge-shaped object (left) and in the signal (right). The ⊗ visualizes the center of a beam that one would get by single-ray-per-beam ray tracing without multi-echo capability.<br><br>

After transformation of the detections from the sensor's spherical coordinates to the vehicle's Cartesian coordinates, they are available as osi3::LogicalDetections.
Subsequently, the tracking-strategy is applied on the calculated logical detections.
It uses the extreme points of the detections for every object for dimension and pose estimation.
At first, position, orientation, and dimensions are estimated as the detected object state.
The detected objects are compared to a list of previously detected and tracked objects and thereby added, updated or possibly deleted.
The track logic can be specified in the model profile.
It specifies e.g. the number of consecutive cycles an object has to be detected to create a new track or how many cycles an object is tracked without any detection.
Therefore, objects that are hidden now, but where previously detected, are still tracked estimating their movement by either using ground truth information or by predictions based on detected objects of the previous time steps.
The estimation of the object dimensions is also considered and filtered over several cycles.
If an object is no longer detected over a defined number of cycles, it is deleted from the list.
Consideration of the class uncertainties is provided by the model architecture, as well.
The output of the tracking module is a list of tracked objects.

<a name="Rosenberger2020">1</a>: P. Rosenberger, M. F. Holder, N. Cianciaruso, P. Aust, J. F. Tamm-Morschel, C. Linnhoff, and H. Winner, “Sequential lidar sensor system simulation: A modular approach for simulation-based safety validation of automated driving”, In Automotive and Engine Technology, vol. 5, no. 3-4, pp. 187–197, Dec. 2020.

### Modeling of Weather Influence on Lidar Point Cloud

In the "Lidar-Environmental-Effects-Strategy", several weather influences are added to the simulated point cloud.
The influences are parameterizable via the sensor profiles and calibrated for certain sensors based on measurements published by Linnhoff et al.[[2](#Linnhoff2022)</sup>]
The stochastically modeled weather conditions include
- Direct Sun Light
- Fog
- Rain
- Snow

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/dc9a1de25d433eb4ec0b68c725ec15b8/lidarModelRain.gif" width="800" />

<a name="Linnhoff2022">2</a>: C. Linnhoff, K. Hofrichter, L. Elster, P. Rosenberger, H. Winner, "Measuring the Influence of Environmental Conditions on Automotive Lidar Sensors,” in MDPI Sensors Journal, vol. 22, no. 14, July 2022.


### Modeling of Road Spray in Lidar Point Cloud

The "Lidar-Environmental-Effects-Strategy" does not only contain weather effects, but also other environmental conditions, specifically the influence of road spray on lidar.
The road spray simulation is based on a methodical data set recorded in real spray conditions on a test track covering multiple vehicle classes, different pavement watering levels and object speeds ranging from 50 km/h to 130 km/h.
Modeling parameters are extracted from the recorded data and utilized to calibrate a novel stochastic road spray model.
The model is focused on simulating clustering effects appearing in the lidar point cloud due to turbulences in the spray plume.
More detail can be found shortly in an upcoming publication, currently under review.[[3](#Linnhoff2022-2)</sup>]

<img src="https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model/uploads/163b40eee2d73c63dd8acb9b1c5d5a33/Spray.gif" width="800" />

<a name="Linnhoff2022-2">3</a>: Under review: C. Linnhoff, D. Scheuble, M. Bijelic, L. Elster, P. Rosenberger, W. Ritter, D. Dai and H. Winner, "Simulating Road Spray Effects in Automotive Lidar Sensor Models,” submitted to IEEE Sensors Journal, 2022.


## Parameterization

The profiles are parameterized in the files `profile_*.hpp.in`.
The parameters are defined in the files `profile.hpp.in`.
The profiles can be extended by the strategies with additional parameters and values in their respective folders as in e.g. `src/model/strategies/lidar-tracking-strategy/` with `profile_struct.hpp.in` with the parameters and `profile_*.hpp.in` with the values.

The profile to be loaded for simulation is set via a model parameter defined in the `modelDescription.xml` of the FMU.
The first name in `src/model/profiles/profile_list.conf` is taken as default.
If you would like to have a different one or if your simulation master does not support the configuration of model parameters, you have to adapt the *start* value of the parameter `profile` in `src/osmp/modelDescription.in.xml`.


### Sensor Parameters

| Parameter                         | Description                                                                                                                       |
| --------------------------------- | --------------------------------------------------------------------------------------------------------------------------------- |
| sensor_view_configuration*        | Update cycle, range, field of view, physical mounting position w.r.t. center of rear axle                                         |
| lidar_sensor_view_configuration** | Field of view horizontal / vertical, number of rays horizontal / vertical, emitter's mounting position w.r.t. center of rear axle |
| min_range                         | Minimum range of every simulated lidar sensor                                                                                     |
<dl>
  <dd>*sensor_view_configuration is defined for every physical sensor system mounted on the ego car</dd>
  <dd>**lidar_sensor_view_configuration is defined for every emitter located within every sensor system</dd>
</dl>

### Detection Sensing Parameters

| Parameter                      | Description                                                                                               |
| ------------------------------ | --------------------------------------------------------------------------------------------------------- |
| no_of_beams_vertical           | Number of real layers of the lidar                                                                        |
| no_of_beams_horizontal         | Real beams per layer of the lidar                                                                         |
| horizontal_angle_min_rad       | Leftmost (negative) horizontal angle incl. beam divergence                                                |
| beam_layer_angles_rad          | Vertical angles of the individual layers of the sensor starting with the highest positive angle           |
| beam_resolution_horizontal_rad | Horizontal beam spacing in rad                                                                            |
| beam_height_rad                | Vertical beam divergence in rad                                                                           |
| beam_width_rad                 | Horizontal beam divergence in rad                                                                         |
| max_echos_per_beam             | Maximum number of echos that are computed per beam                                                        |
| rays_per_beam_vertical         | Vertical super-sampling parameter of rays per beam                                                        |
| rays_per_beam_horizontal       | Horizontal super-sampling parameter of rays per beam                                                      |
| ray_resolution_vertical_rad    | Vertical angular distance between rays in rad                                                             |
| ray_resolution_horizontal_rad  | Horizontal angular distance between rays in rad                                                           |
| distance_resolution_adc        | Specified distance resolution of the adc in m, 0.3 means time resolution of 1 ns                          |
| distance_resolution            | Specified distance resolution of the sensor output in m                                                   |
| distance_stddev                | Specified distance standard deviation in m for gaussian noise model                                       |
| intensity_resolution           | Specified intensity resolution of the sensor output in %                                                  |
| intensity_threshold_in_dB      | Specified intensity threshold of the sensor at output in dB                                               |
| echo_separation_distance       | Distance for separation of two echos in m, when no reflections registered in between                      |
| echo_determination_mode        | Determines what part of the echo pulse is taken for distance. Options: {"peak", "start", "center", "end"} |

### Segmentation Parameters

| Parameter                  | Description                                                                    |
| -------------------------- | ------------------------------------------------------------------------------ |
| tolerance_for_segmentation | Tolerance in m that is added to object dimensions for point cloud segmentation |

### Object Tracking Parameters

| Parameter                                    | Description                                                  |
| -------------------------------------------- | ------------------------------------------------------------ |
| classification_flag                          | 0 = from ground truth; 1 = all "Unknown Big"                 |
| orientation_flag                             | 0 = from ground truth; 1 = from current point cloud segment  |
| dimension_and_position_flag                  | 0 = from ground truth;<br/>1 = from current point cloud segment;<br/>2 = dimension from current point cloud segments with lower bounds, position as center of manipulated pcl segment;<br/>3 = maximum dimension of current and mean of historical point cloud segments, position as center of manipulated pcl segment;<br/>4 = maximum dimension of current and mean of historical point cloud segments with lower bounds, position as center of manipulated pcl segment; |
| minimum_object_dimension                     | Minimum dimension in m for detected objects                  |
| historical_limit_dimension                   | Limits the historical data used for historical mean dimension calculation |
| velocity_flag                                | 0 = from ground truth; 1 = derivation of position            |
| tracking_flag                                | 0 = ideal (track all ground truth objects); 1 = realistic lidar tracking behavior |
| existence_probability_threshold_for_tracking | Threshold for existence probability, tracking is enabled above threshold |
| min_detections_in_segment_for_tracking       | Minimum no. of detections per segment to track it            |
| existence_probability_increment              | Increment for existence probability                          |
| existence_probability_decrement              | Decrement for existence probability                          |

## Configuration

### Model name

The model's name (in this case "ReflectionBasedLidarModel") used for CMake-projects and the FMU at the end is defined in file `model_name.conf` located at `src/model`.

### Install path

When building and installing, the framework will build an FMU package into `FMU_INSTALL_DIR`, which can be used with a simulation tool that supports OSI and fills the required fields listed below.

### VariableNamingConvention

The parameter variableNamingConvention for the FMU specified within the modelDescription.xml is taken from file `variableNamingConvention.conf` located at `src/osmp`.
Possible values are "flat" or "structured".

## Inferface

### Required SensorViewConfiguration (parameterized in profile_*.hpp.in) to be Set in the Simulation Tool

- For every simulated physical sensor system:
  - sensor_view_configuration.mounting_position.position
  - sensor_view_configuration.mounting_position.orientation
  - sensor_view_configuration.update_cycle_time
  - sensor_view_configuration.range
  - sensor_view_configuration.field_of_view_horizontal
  - sensor_view_configuration.field_of_view_vertical
- For every simulated lidar signal emitter per sensor system:
  - sensor_view_configuration.lidar_sensor_view_configuration.mounting_position.position
  - sensor_view_configuration.lidar_sensor_view_configuration.mounting_position.orientation
  - sensor_view_configuration.lidar_sensor_view_configuration.field_of_view_horizontal
  - sensor_view_configuration.lidar_sensor_view_configuration.field_of_view_vertical
  - sensor_view_configuration.lidar_sensor_view_configuration.number_of_rays_horizontal
  - sensor_view_configuration.lidar_sensor_view_configuration.number_of_rays_vertical

### Required Fields in OSI3 Sensor_View Filled at the Input by the Simulation Tool
- Ground Truth object list
  - sensor_view.mounting_position
  - sensor_view.global_ground_truth.timestamp
  - sensor_view.global_ground_truth.host_vehicle_id
  - sensor_view.global_ground_truth.stationary_object.id
  - sensor_view.global_ground_truth.stationary_object.base.position
  - sensor_view.global_ground_truth.stationary_object.base.orientation
  - sensor_view.global_ground_truth.stationary_object.base.dimension
  - sensor_view.global_ground_truth.stationary_object.classification.type
  - sensor_view.global_ground_truth.moving_object.id
  - sensor_view.global_ground_truth.moving_object.base.position
  - sensor_view.global_ground_truth.moving_object.base.orientation
  - sensor_view.global_ground_truth.moving_object.base.orientation_rate
  - sensor_view.global_ground_truth.moving_object.base.velocity
  - sensor_view.global_ground_truth.moving_object.base.acceleration
  - sensor_view.global_ground_truth.moving_object.base.dimension
  - sensor_view.global_ground_truth.moving_object.type
  - sensor_view.global_ground_truth.moving_object.vehicle_classification.type
  - sensor_view.global_ground_truth.moving_object.vehicle_attributes.bbcenter_to_rear
- Lidar reflections
  - sensor_view.lidar_sensor_view.rendering_result.received_signal
  - sensor_view.lidar_sensor_view.rendering_result.path_length
  - sensor_view.lidar_sensor_view.rendering_result.emitted_signal_idx

### Additionally Filled Fields in OSI3 Sensor_Data by the Sensor Model

---

**NOTE**

Currently, all information on model input is passed to the output.

---

- sensor_data.timestamp
- sensor_data.moving_object_header.measurement_time
- sensor_data.moving_object_header.cycle_counter
- sensor_data.moving_object_header.data_qualifier
- sensor_data.moving_object.header.ground_truth_id
- sensor_data.moving_object.header.tracking_id
- sensor_data.moving_object.header.age
- sensor_data.moving_object.base.position
- sensor_data.moving_object.base.orientation
- sensor_data.moving_object.base.orientation_rate
- sensor_data.moving_object.base.velocity
- sensor_data.moving_object.base.acceleration
- sensor_data.moving_object.base.dimension
- sensor_data.moving_object.reference_point
- sensor_data.moving_object.movement_state
- sensor_data.moving_object.candidate.probability
- sensor_data.moving_object.candidate.type
- sensor_data.feature_data.lidar_sensor.detection.position
- sensor_data.feature_data.lidar_sensor.detection.intensity
- sensor_data.feature_data.lidar_sensor.detection.echo_pulse_width

## Build Instructions in Windows 10

### Install Dependencies in Windows 10

1. Install cmake from https://github.com/Kitware/CMake/releases/download/v3.20.3/cmake-3.20.3-windows-x86_64.msi
2. Install protobuf for [MSYS-2020](install_protobuf_Win64_MSYS-2020.md) or [Visual Studio 2017](install_protobuf_Win64_VS2017.md)

### Clone with Submodules, Build, and Install in Windows 10

1. Clone this repository <ins>with submodules</ins>:
   ```bash
   $ git clone https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model.git --recurse-submodules
   ```
2. Build the model in [MSYS-2020](install_model_Win64_MSYS-2020.md) or [Visual Studio 2017](install_model_Win64_VS2017.md)
3. Take FMU from `FMU_INSTALL_DIR`

    (Please note that sources are not packed into the FMU at the moment.)

## Build Instructions in Ubuntu 18.04 / 20.04

### Install Dependencies in Ubuntu 18.04 / 20.04

1. Install cmake 3.12
   * as told in [these install instructions](install_cmake_ubuntu_3-12.md)
2. Install protobuf 3.0.0:
   * Check your version via `protoc --version`. It should output: `libprotoc 3.0.0`
   * If needed, you can install it via `sudo apt-get install libprotobuf-dev protobuf-compiler`
   * or from source:
     * Download it from https://github.com/protocolbuffers/protobuf/releases/tag/v3.0.0 and extract the archive.
     * Try to run `./autogen.sh`, if it failes, download the gmock-1.7.0.zip from https://pkgs.fedoraproject.org/repo/pkgs/gmock/gmock-1.7.0.zip/073b984d8798ea1594f5e44d85b20d66/gmock-1.7.0.zip, extract it into the protobuf folder and rename the gmock-1.7.0 folter to gmock.
     * Proceed with the install with
     ```bash
     $ make
     $ sudo make install
     $ sudo ldconfig # refresh shared library cache.
     ```

### Clone with Submodules, Build, and Install in Ubuntu 18.04 / 20.04

1. Clone this repository <ins>with submodules</ins>:
    ```bash
    $ git clone https://gitlab.com/tuda-fzd/perception-sensor-modeling/reflection-based-lidar-object-model.git --recurse-submodules
    ```
2. Build the model by executing in the extracted project root directory:
    ```bash
    $ mkdir cmake-build
    $ cd cmake-build
    # If FMU_INSTALL_DIR is not set, CMAKE_BINARY_DIR is used
    $ cmake -DCMAKE_BUILD_TYPE=Release -DFMU_INSTALL_DIR:PATH=/tmp ..
    $ make -j N_JOBS
    ```
3. Take FMU from `FMU_INSTALL_DIR`

    (Please note that sources are not packed into the FMU at the moment.)

## Licensing

**Please read file [COPYING](COPYING), which is located in the project root, carefully.**

## Credits
This work received funding from the research project 
"[SET Level](https://setlevel.de/)" of the [PEGASUS ](https://pegasus-family.de) project family, promoted by the German Federal Ministry for Economic Affairs and Energy based on a decision of the German Bundestag.
| SET Level                                                                                                | PEGASUS Family                                                                                                       | BMWi                                                                                                                                                                                 |
|----------------------------------------------------------------------------------------------------------|----------------------------------------------------------------------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| <a href="https://setlevel.de"><img src="https://setlevel.de/assets/logo-setlevel.svg" width="100" /></a> | <a href="https://pegasus-family.de"><img src="https://setlevel.de/assets/logo-pegasus-family.svg" width="100" /></a> | <a href="https://www.bmwi.de/Redaktion/DE/Textsammlungen/Technologie/fahrzeug-und-systemtechnologien.html"><img src="https://setlevel.de/assets/logo-bmwi-en.svg" width="100" /></a> |


Thanks to all contributors of the following libraries:

- [Open Simulation Interface](https://github.com/OpenSimulationInterface/open-simulation-interface), a generic interface based on protocol buffers for the environmental perception of automated driving functions in virtual scenarios
- [FMI Version 2.0: FMI for Model Exchange and Co-Simulation](https://fmi-standard.org/downloads/)
- [Eigen](http://eigen.tuxfamily.org/), a C++ template library for linear algebra: matrices, vectors, numerical solvers, and related algorithms.
