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

#include "csvoutputgtobjects/CsvOutputGTObjects.hpp"
#include "../../transformation-functions/TransformationFunctions.hpp"
#include <fstream>
#include <iostream>
#include <vector>
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

static bool first_call = true;

void model::CsvOutputGTObjects::apply(SensorData &sensor_data) {
    log("Starting .csv output for GT objects");

    if (sensor_data.sensor_view_size() == 0) {
        log("No sensor view received for .csv output");
        return;
    }

    TF::EgoData ego_data;
    if (!TF::get_ego_info(ego_data, sensor_data.sensor_view(0))) {
        log("Ego vehicle has no base, no id, or is not contained in GT moving objects.");
        return;
    }

    auto time_nanos = sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos();
    auto time_seconds = sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds();
    double timestamp = (double)time_seconds + (double) time_nanos / 1000000000;

    if (!sensor_data.sensor_view(0).has_global_ground_truth()) {
        log("No GT for .csv output at timestamp " + std::to_string(timestamp));
        return;
    }

    if ((sensor_data.sensor_view(0).global_ground_truth().moving_object_size() == 0) &&
        (sensor_data.sensor_view(0).global_ground_truth().stationary_object_size() == 0)) {
        log("No objects in GT for .csv output at timestamp " + std::to_string(timestamp));
        return;
    }
    
    /// Write header line of .csv on first call
    if (first_call) {
        #include <csvoutputgtobjects/set_csv_file_path_gtobjects.cpp>
        write_first_line_to_CSV(file_path_gtobjects);
        first_call = false;
    }

    auto no_of_moving_objects = sensor_data.sensor_view(0).global_ground_truth().moving_object_size();
    auto no_of_stationary_objects = sensor_data.sensor_view(0).global_ground_truth().stationary_object_size();

    /// Start a vector for gt_objects with (gt_id, x, y, z, roll, pitch, yaw, width, length, height, is_moving)
    std::vector<GT_object> gt_objects;
    gt_objects.reserve(no_of_moving_objects + no_of_stationary_objects);

    /// Collect moving objects
    for (const auto &gt_moving_object : sensor_data.sensor_view(0).global_ground_truth().moving_object()) {
        if (gt_moving_object.id().value() == sensor_data.sensor_view(0).global_ground_truth().host_vehicle_id().value())
            continue;

        Vector3d relative_position_ego_coordinate_system = TF::transform_position_from_world_to_ego_coordinates(gt_moving_object.base().position(), ego_data);
        Orientation3d relative_orientation = TF::calc_relative_orientation_to_local(gt_moving_object.base().orientation(), ego_data.ego_base.orientation());
        GT_object actual_gt_object;
        actual_gt_object.id = gt_moving_object.id().value();
        actual_gt_object.x = std::round(float(relative_position_ego_coordinate_system.x()) * 1000) / 1000;
        actual_gt_object.y = std::round(float(relative_position_ego_coordinate_system.y()) * 1000) / 1000;
        actual_gt_object.z = std::round(float(relative_position_ego_coordinate_system.z()) * 1000) / 1000;
        actual_gt_object.roll  = (float)std::round(float(relative_orientation.roll())  * 180 / M_PI * 1000) / 1000;
        actual_gt_object.pitch = (float)std::round(float(relative_orientation.pitch()) * 180 / M_PI * 1000) / 1000;
        actual_gt_object.yaw   = (float)std::round(float(relative_orientation.yaw())   * 180 / M_PI * 1000) / 1000;
        actual_gt_object.width  = float(gt_moving_object.base().dimension().width());
        actual_gt_object.length = float(gt_moving_object.base().dimension().length());
        actual_gt_object.height = float(gt_moving_object.base().dimension().height());
        actual_gt_object.is_moving = true;
        gt_objects.emplace_back(actual_gt_object);
    }

    /// Collect stationary objects
    for (const auto &gt_stationary_object : sensor_data.sensor_view(0).global_ground_truth().stationary_object()) {
        Vector3d relative_position_ego_coordinate_system = TF::transform_position_from_world_to_ego_coordinates(
                gt_stationary_object.base().position(), ego_data);
        Orientation3d relative_orientation = TF::calc_relative_orientation_to_local(
                gt_stationary_object.base().orientation(),
                ego_data.ego_base.orientation());
        GT_object actual_gt_object;
        actual_gt_object.id = gt_stationary_object.id().value();
        actual_gt_object.x = std::round(float(relative_position_ego_coordinate_system.x()) * 1000) / 1000;
        actual_gt_object.y = std::round(float(relative_position_ego_coordinate_system.y()) * 1000) / 1000;
        actual_gt_object.z = std::round(float(relative_position_ego_coordinate_system.z()) * 1000) / 1000;
        actual_gt_object.roll  = (float)std::round(float(relative_orientation.roll())  * 180 / M_PI * 1000) / 1000;
        actual_gt_object.pitch = (float)std::round(float(relative_orientation.pitch()) * 180 / M_PI * 1000) / 1000;
        actual_gt_object.yaw   = (float)std::round(float(relative_orientation.yaw())   * 180 / M_PI * 1000) / 1000;
        actual_gt_object.width  = float(gt_stationary_object.base().dimension().width());
        actual_gt_object.length = float(gt_stationary_object.base().dimension().length());
        actual_gt_object.height = float(gt_stationary_object.base().dimension().height());
        actual_gt_object.is_moving = false;
        gt_objects.emplace_back(actual_gt_object);
    }

    for (const auto &gt_object : gt_objects) {
        auto gt_id = gt_object.id;

        auto roll = std::round(gt_object.roll* 180 / M_PI * 1000) / 1000;
        auto pitch = std::round(gt_object.pitch * 180 / M_PI * 1000) / 1000;
        auto yaw = std::round(gt_object.yaw * 180 / M_PI * 1000) / 1000;

        write_data_to_CSV(file_path_gtobjects, timestamp, gt_id, gt_object.x, gt_object.y, gt_object.z, gt_object.roll, gt_object.pitch, gt_object.yaw,
                            gt_object.width, gt_object.length, gt_object.height, gt_object.is_moving);
    }
}

void CsvOutputGTObjects::write_first_line_to_CSV(const std::string &path) {
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file
            << "timestamp_in_s, gt_object_id, x_in_m, y_in_m, z_in_m, roll_in_deg, pitch_in_deg, yaw_in_deg, width_in_m, length_in_m, height_in_m, is_moving"
            << std::endl;
    my_file.close();
}

void CsvOutputGTObjects::write_data_to_CSV(const std::string& path, double timestamp, size_t object_idx, float x, float y, float z, float roll, float pitch, float yaw, float width, float length, float height, bool is_moving) {
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << timestamp << ", " << object_idx << ", " << x << ", " << y << ", " << z << ", " << roll << ", " << pitch
            << ", " << yaw << ", " << width << ", " << length << ", " << height << ", "
            << (is_moving ? "true" : "false") << std::endl;
    my_file.close();
}