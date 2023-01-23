//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "csvoutputdetections/CsvOutputDetections.hpp"

#include <ctime>
#include <fstream>
#include <iostream>

#ifdef _WIN32
#include <math.h>

#include <direct.h>
#else
#include <cmath>

#include <sys/stat.h>
#endif

using namespace model;
using namespace osi3;

static bool first_call = true;

void model::CsvOutputDetections::apply(SensorData& sensor_data)
{
    log("Starting .csv output for detections");

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
    double timestamp = (double)time_seconds + time_nanos / 1000000000.0;

    if (sensor_data.sensor_view(0).lidar_sensor_view().size() > 0)
    {
        if (first_call && (sensor_data.feature_data().lidar_sensor(0).detection().size() > 0))
        {
#include <csvoutputdetections/set_csv_file_path_detections.cpp>
            write_first_line_to_CSV(file_path_detections, sensor_data.feature_data().lidar_sensor(0).detection(0).has_intensity());
            first_call = false;
        }
        for (int sensor_idx = 0; sensor_idx < sensor_data.sensor_view(0).lidar_sensor_view_size(); sensor_idx++)
        {
            if (sensor_data.feature_data().lidar_sensor(sensor_idx).detection().size() > 0)
            {
                /// Run through all detections
                size_t detection_idx = 0;
                for (const auto& detection : sensor_data.feature_data().lidar_sensor(sensor_idx).detection())
                {

                    if (detection.has_intensity())
                        write_data_to_CSV(file_path_detections,
                                          timestamp,
                                          detection_idx,
                                          detection.position().azimuth() * 180 / M_PI,
                                          detection.position().elevation() * 180 / M_PI,
                                          detection.position().distance(),
                                          detection.intensity());
                    else if (detection.has_echo_pulse_width())
                        write_data_to_CSV(file_path_detections + "\\Detections.csv",
                                          timestamp,
                                          detection_idx,
                                          detection.position().azimuth() * 180 / M_PI,
                                          detection.position().elevation() * 180 / M_PI,
                                          detection.position().distance(),
                                          detection.echo_pulse_width());
                    detection_idx++;
                }
            }
        }
    }
    else if (sensor_data.sensor_view(0).radar_sensor_view().size() > 0)
    {
        if (first_call && (sensor_data.feature_data().radar_sensor(0).detection().size() > 0))
        {
#include <csvoutputdetections/set_csv_file_path_detections.cpp>
            write_first_line_to_CSV(file_path_detections, 2);
            first_call = false;
        }
        for (int sensor_idx = 0; sensor_idx < sensor_data.sensor_view(0).radar_sensor_view_size(); sensor_idx++)
        {
            if (sensor_data.feature_data().radar_sensor(sensor_idx).detection().size() > 0)
            {
                /// Run through all detections
                size_t detection_idx = 0;
                for (const auto& detection : sensor_data.feature_data().radar_sensor(sensor_idx).detection())
                {
                    write_data_to_CSV(file_path_detections,
                                      timestamp,
                                      detection_idx,
                                      detection.position().azimuth() * 180 / M_PI,
                                      detection.position().elevation() * 180 / M_PI,
                                      detection.position().distance(),
                                      detection.rcs());
                    detection_idx++;
                }
            }
            else
            {
                log("No radar detections for .csv output at timestamp " + std::to_string(timestamp));
            }
        }
    }
    else
    {
        log("No lidar or radar sensor view");
        return;
    }
}

void CsvOutputDetections::write_first_line_to_CSV(const std::string& path, const size_t& epw_intensity_rcs_flag)
{
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    if (epw_intensity_rcs_flag == 0)
    {
        my_file << "timestamp_in_s, detection_id, azimuth_in_deg, elevation_in_deg, distance_in_m, epw_in_m" << std::endl;
    }
    else if (epw_intensity_rcs_flag == 1)
    {
        my_file << "timestamp_in_s, detection_id, azimuth_in_deg, elevation_in_deg, distance_in_m, intensity_in_%" << std::endl;
    }
    else if (epw_intensity_rcs_flag == 2)
    {
        my_file << "timestamp_in_s, detection_id, azimuth_in_deg, elevation_in_deg, distance_in_m, rcs_in_dbm²" << std::endl;
    }
    else
        alert("Flag for intensity, RCS, or echo pulse width output not set correctly!");
    my_file.close();
}

void CsvOutputDetections::write_data_to_CSV(const std::string& path,
                                            double timestamp,
                                            size_t detection_idx,
                                            double azimuth_in_deg,
                                            double elevation_in_deg,
                                            double distance,
                                            double intensity)
{
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << timestamp << ", " << detection_idx << ", " << azimuth_in_deg << ", " << elevation_in_deg << ", " << distance << ", " << intensity << std::endl;
    my_file.close();
}