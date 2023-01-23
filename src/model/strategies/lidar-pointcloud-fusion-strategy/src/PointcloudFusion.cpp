//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "pointcloudfusion/PointcloudFusion.hpp"

#include <string>

#include "model/strategies/transformation-functions/TransformationFunctions.hpp"

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

using namespace model;
using namespace osi3;

void PointcloudFusion::apply(SensorData& in)
{
    log("Starting point cloud fusion");

    auto no_of_lidar_sensors = in.feature_data().lidar_sensor_size();

    if (in.has_feature_data() & (no_of_lidar_sensors > 0))
    {
        calculate_fused_pointcloud_for_given_sensors(in, profile, log);
        log("Size of logical detections after point cloud fusion: " + std::to_string(in.logical_detection_data().logical_detection_size()));
    }
    else
    {
        auto timestamp = (double)in.sensor_view(0).global_ground_truth().timestamp().seconds() + (double)in.sensor_view(0).global_ground_truth().timestamp().nanos() / 1000000000;
        log("No feature data available for timestamp " + std::to_string(timestamp));
    }
}

//// Functions
void PointcloudFusion::calculate_fused_pointcloud_for_given_sensors(SensorData& in, const Profile& profile, const Log& log)
{
    in.mutable_logical_detection_data()->clear_logical_detection();

    for (int sensor_idx = 0; sensor_idx < in.feature_data().lidar_sensor_size(); sensor_idx++)
    {
        for (int detection_no = 0; detection_no < in.feature_data().lidar_sensor(sensor_idx).detection_size(); detection_no++)
        {

            double elevation = in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).position().elevation();
            double azimuth = in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).position().azimuth();
            double distance = in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).position().distance();

            Vector3d point_cartesian_sensor;
            point_cartesian_sensor.set_x(distance * cos(elevation) * cos(azimuth));
            point_cartesian_sensor.set_y(distance * cos(elevation) * sin(azimuth));
            point_cartesian_sensor.set_z(distance * sin(elevation));
            /*Vector3d point_cartesian_in_ego_coordinates =
               TransformationFunctions::transform_from_local_coordinates(point_cartesian_sensor,
                                                          profile.sensor_view_configuration.lidar_sensor_view_configuration(sensor_idx).mounting_position().orientation(),
                                                          profile.sensor_view_configuration.lidar_sensor_view_configuration(sensor_idx).mounting_position().position());*/
            Vector3d point_cartesian_in_ego_coordinates =
                TF::transform_from_local_coordinates(point_cartesian_sensor,
                                                     in.feature_data().lidar_sensor(sensor_idx).header().mounting_position().orientation(),
                                                     in.feature_data().lidar_sensor(sensor_idx).header().mounting_position().position());

            auto current_logical_detection = in.mutable_logical_detection_data()->add_logical_detection();
            current_logical_detection->mutable_position()->CopyFrom(point_cartesian_in_ego_coordinates);
            if (in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).has_intensity())
            {
                current_logical_detection->set_intensity(in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).intensity());
            }
            else if (in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).has_echo_pulse_width())
            {
                current_logical_detection->set_echo_pulse_width(
                    in.feature_data().lidar_sensor(sensor_idx).detection(detection_no).echo_pulse_width());  // TODO: Field for echo pulse width within logical detections not
                                                                                                             // existing, yet
            }
        }
    }
}
