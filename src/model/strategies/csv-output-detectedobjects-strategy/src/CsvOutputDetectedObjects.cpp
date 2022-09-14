//
// Copyright Institute of Automotive Engineering
// of Technical University of Darmstadt 2020.
// Licensed under the EUPL-1.2-or-later
//
// This work covered by the EUPL can be used/merged and distributed
// in other works covered by GPL-2.0, GPL-3.0, LGPL, AGPL, CeCILL,
// OSL, EPL, MPL and other licences listed as compatible in the EUPL
// Appendix. This applies to the other (combined) work, while the
// original project stays covered by the EUPL without re-licensing.
//
// Alternatively, the contents of this file may be used under the
// terms of the Mozilla Public License, v. 2.0. If a copy of the MPL
// was not distributed with this file, you can obtain one at
// http://mozilla.org/MPL/2.0/.
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "csvoutputdetectedobjects/CsvOutputDetectedObjects.hpp"
#include <fstream>
#include <iostream>
#include <ctime>

#ifdef _WIN32
    #include <math.h>
    #include <direct.h>
#else
    #include <cmath>
    #include <sys/stat.h>
#endif

using namespace model;
using namespace osi3;

void model::CsvOutputDetectedObjects::apply(SensorData &sensor_data) {
    log("Starting .csv output for detected objects");

    if ((sensor_data.sensor_view_size()==0) || (!sensor_data.sensor_view(0).has_global_ground_truth())) {
        log("No sensor view or global ground truth");
        return;
    }

    const auto &ground_truth = sensor_data.sensor_view(0).global_ground_truth();

    auto time_nanos = ground_truth.timestamp().nanos();
    auto time_seconds = ground_truth.timestamp().seconds();
    double timestamp = (double) time_seconds + (double) time_nanos / 1000000000;

    auto no_of_moving_objects = sensor_data.moving_object_size();
    auto no_of_stationary_objects = sensor_data.stationary_object_size();
    auto no_of_objects = no_of_moving_objects + no_of_stationary_objects;

    /// Add tracking_id, x, y, z, roll, pitch, yaw, width, length, height, is_moving, existence_probability to csv
    if (no_of_objects > 0) {

        /// Write header line of .csv on first call
        if (first_call){
            #include <csvoutputdetectedobjects/set_csv_file_path_detectedobjects.cpp>
            write_first_line_to_CSV(file_path_detectedobjects);
            first_call = false;
        }
        
        for (const auto &moving_object : sensor_data.moving_object()) {
            write_data_to_CSV(file_path_detectedobjects,
                              timestamp,
                              moving_object.header().tracking_id().value(),
                              std::round(moving_object.base().position().x() * 1000) / 1000,
                              std::round(moving_object.base().position().y() * 1000) / 1000,
                              std::round(moving_object.base().position().z() * 1000) / 1000,
                              std::round(moving_object.base().orientation().roll() * 1000) / 1000,
                              std::round(moving_object.base().orientation().pitch() * 1000) / 1000,
                              std::round(moving_object.base().orientation().yaw() * 1000) / 1000,
                              std::round(moving_object.base().dimension().width() * 1000) / 1000,
                              std::round(moving_object.base().dimension().length() * 1000) / 1000,
                              std::round(moving_object.base().dimension().height() * 1000) / 1000,
                              true,
                              std::round(moving_object.header().existence_probability() * 1000) / 1000);
        }
        for (const auto &stationary_object : sensor_data.stationary_object()) {
            write_data_to_CSV(file_path_detectedobjects,
                              timestamp,
                              stationary_object.header().tracking_id().value(),
                              std::round(stationary_object.base().position().x() * 1000) / 1000,
                              std::round(stationary_object.base().position().y() * 1000) / 1000,
                              std::round(stationary_object.base().position().z() * 1000) / 1000,
                              std::round(stationary_object.base().orientation().roll() * 1000) / 1000,
                              std::round(stationary_object.base().orientation().pitch() * 1000) / 1000,
                              std::round(stationary_object.base().orientation().yaw() * 1000) / 1000,
                              std::round(stationary_object.base().dimension().width() * 1000) / 1000,
                              std::round(stationary_object.base().dimension().length() * 1000) / 1000,
                              std::round(stationary_object.base().dimension().height() * 1000) / 1000,
                              false,
                              std::round(stationary_object.header().existence_probability() * 1000) / 1000);
        }
    } else {
        log("No detected objects for .csv output at timestamp " + std::to_string(timestamp));
        return;
    }
}

void CsvOutputDetectedObjects::write_first_line_to_CSV(const std::string& path) {
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << "timestamp_in_s, tracking_id, x_in_m, y_in_m, z_in_m, roll_in_deg, pitch_in_deg, yaw_in_deg, width_in_m, length_in_m, height_in_m, is_moving, existence_probability" << std::endl;
    my_file.close();
}

void
CsvOutputDetectedObjects::write_data_to_CSV(const std::string &path, double timestamp, size_t tracking_id, double x, double y, double z, double roll, double pitch, double yaw, double width, double length,
                                            double height, bool is_moving, double existence_probability) {
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << timestamp << ", " << tracking_id << ", " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch << ", " << yaw << ", " << width << ", " << length << ", " << height << ", " << (is_moving ? "true": "false") << ", " << existence_probability << std::endl;
    my_file.close();
}
