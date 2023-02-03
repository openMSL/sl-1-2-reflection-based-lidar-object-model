//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#include "pcdoutputlogicaldetections/PcdOutputLogicalDetections.hpp"

#include <ctime>
#include <fstream>
#include <iostream>

#ifdef _WIN32
#include <math.h>

#include <direct.h>
#else
#include <sys/stat.h>
#endif

using namespace model;
using namespace osi3;

void model::PcdOutputLogicalDetections::apply(SensorData& sensor_data)
{
    log("Starting .pcd output for logical detections");

    if ((sensor_data.sensor_view_size() == 0) || (!sensor_data.has_feature_data()))
    {
        log("No sensor view or feature data received");
        return;
    }

    if (!sensor_data.sensor_view(0).has_global_ground_truth())
    {
        log("No global ground truth received");
        return;
    }

    auto time_nanos = sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos();
    auto time_seconds = sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds();
    double timestamp = (double)time_seconds + (double)time_nanos / 1000000000;

    size_t epw_intensity_rcs_flag;

    if ((sensor_data.sensor_view(0).lidar_sensor_view().size() > 0) && (sensor_data.feature_data().lidar_sensor(0).detection_size() > 0))
    {
        if (sensor_data.feature_data().lidar_sensor(0).detection(0).has_echo_pulse_width())
        {
            epw_intensity_rcs_flag = 0;
        }
        else if (sensor_data.feature_data().lidar_sensor(0).detection(0).has_intensity())
        {
            epw_intensity_rcs_flag = 1;
        }
    }
    else if (sensor_data.sensor_view(0).radar_sensor_view().size() > 0)
    {
        epw_intensity_rcs_flag = 2;
    }
    else
    {
        log("No EPW or Intensity or RCS value in detections available");
    }

    if (sensor_data.logical_detection_data().logical_detection_size() == 0)
    {
        log("No logical detections for .csv output at timestamp " + std::to_string(timestamp));
        return;
    }

    if (first_call)
    {
#include <pcdoutputlogicaldetections/set_pcd_file_path_logicaldetections.cpp>
        first_call = false;
    }

    /// Header needed in every file
    std::string filename = "LogicalDetections_";
    filename.append(std::to_string(timestamp));
    filename.append(".pcd");
#if defined(_WIN32)
    std::string path = path_string + "\\" + filename;
#else
    std::string path = path_string + "/" + filename;
#endif
    write_pcd_header(path, sensor_data);

    for (const auto& logical_detection : sensor_data.logical_detection_data().logical_detection())
    {
        write_2_pcd(path,
                  float(logical_detection.position().x()),
                  float(logical_detection.position().y()),
                  float(logical_detection.position().z()),
                  (epw_intensity_rcs_flag != 0) ? float(logical_detection.intensity()) : float(logical_detection.echo_pulse_width()));
    }
}

void PcdOutputLogicalDetections::write_pcd_header(const std::string& path, const SensorData& sensor_data)
{
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
    my_file << "VERSION "
            << "0.7" << std::endl;
    my_file << "FIELDS "
            << "x y z intensity" << std::endl;
    my_file << "SIZE "
            << "4 4 4 4" << std::endl;
    my_file << "TYPE "
            << "F F F F" << std::endl;
    my_file << "COUNT "
            << "1 1 1 1" << std::endl;
    my_file << "WIDTH " << sensor_data.logical_detection_data().logical_detection_size() << std::endl;
    my_file << "HEIGHT "
            << "1" << std::endl;
    my_file << "VIEWPOINT "
            << "0 0 0 1 0 0 0" << std::endl;
    my_file << "POINTS " << sensor_data.logical_detection_data().logical_detection_size() << std::endl;
    my_file << "DATA "
            << "ascii" << std::endl;
    my_file.close();
}

void PcdOutputLogicalDetections::write_2_pcd(const std::string& path, float x, float y, float z, float intensity)
{
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << x << " " << y << " " << z << " " << intensity << std::endl;
    my_file.close();
}
