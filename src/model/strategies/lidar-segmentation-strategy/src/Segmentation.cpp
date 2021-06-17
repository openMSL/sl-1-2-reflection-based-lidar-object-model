//
// Copyright Philipp Rosenberger, M. Sc.
// Institute of Automotive Engineering
// of Technical University of Darmstadt, 2021.
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

#include "segmentation/Segmentation.hpp"
#include <string>

using namespace model;
using namespace osi3;

void model::Segmentation::apply(SensorData &in) {
    log("Starting Segmentation");

    uint64_t no_of_moving_objects = in.sensor_view(0).global_ground_truth().moving_object_size();
    log("No. of moving objects from GT is " + std::to_string(no_of_moving_objects) + " (incl. ego vehicle)");

    if (in.has_logical_detection_data()) {
        TransformationFunctions::EgoData ego_data;
        if(!TransformationFunctions::get_ego_info(ego_data, in.sensor_view(0)))
            alert("Ego vehicle has no base, no id, or is not contained in GT moving objects.");

        /// Run through all objects in current frame except the host vehicle
        for (uint64_t object_no_in = 0; object_no_in < no_of_moving_objects; object_no_in++) {
            uint64_t object_id = in.sensor_view(0).global_ground_truth().moving_object(object_no_in).id().value();
            if (object_id != ego_data.ego_vehicle_id.value()) { // Continue in case of ego car
                /// Get corners of current object and set the object id for all points
                std::vector<Vector3d> bounding_box_corners = get_bounding_box_corners(in.sensor_view(0).global_ground_truth().moving_object(object_no_in));
                calculate_segment_of_point_cloud(in, object_no_in, bounding_box_corners, ego_data, profile, log);
            }
        }
    }
    else{
        auto timestamp = (double) in.sensor_view(0).global_ground_truth().timestamp().seconds() + (double) in.sensor_view(0).global_ground_truth().timestamp().nanos() / 1000000000;
        log("No logical detection data available for timestamp " + std::to_string(timestamp));
    }
}

std::vector<Vector3d> Segmentation::get_bounding_box_corners(const MovingObject &current_moving_object) {

    std::vector< Vector3d > bounding_box_corners;
    std::vector< std::vector <double> > corner_factor_vectors = {{ 0.5,  0.5, -0.5},
                                                       { 0.5, -0.5, -0.5},
                                                       {-0.5,  0.5, -0.5},
                                                       {-0.5, -0.5, -0.5},
                                                       { 0.5,  0.5,  0.5},
                                                       { 0.5, -0.5,  0.5},
                                                       {-0.5,  0.5,  0.5},
                                                       {-0.5, -0.5,  0.5}
    };

    for (std::vector <double> &corner_factor_vector : corner_factor_vectors) {
        Vector3d current_corner;
        current_corner.set_x(corner_factor_vector.at(0) * current_moving_object.base().dimension().length());
        current_corner.set_y(corner_factor_vector.at(1) * current_moving_object.base().dimension().width());
        current_corner.set_z(corner_factor_vector.at(2) * current_moving_object.base().dimension().height());
        bounding_box_corners.emplace_back(current_corner);
    }

    return bounding_box_corners;
}

uint64_t Segmentation::calculate_segment_of_point_cloud(SensorData &in, uint64_t object_no_in, const std::vector<Vector3d> &bounding_box_corners, const TransformationFunctions::EgoData &ego_data, const Profile &profile, const Log &log) {

    /// Bounding box corners with tolerance for segmentation
    double x_min = bounding_box_corners.at(2).x() - profile.segmentation_parameters.tolerance_for_segmentation;
    double x_max = bounding_box_corners.at(0).x() + profile.segmentation_parameters.tolerance_for_segmentation;
    double y_min = bounding_box_corners.at(1).y() - profile.segmentation_parameters.tolerance_for_segmentation;
    double y_max = bounding_box_corners.at(0).y() + profile.segmentation_parameters.tolerance_for_segmentation;
    double z_min = bounding_box_corners.at(0).z() - profile.segmentation_parameters.tolerance_for_segmentation;
    double z_max = bounding_box_corners.at(4).z() + profile.segmentation_parameters.tolerance_for_segmentation;

    /// Run through all logical_detections
    uint64_t segment_size_of_current_object = 0;
    for (uint64_t logical_detection_idx = 0; logical_detection_idx < in.logical_detection_data().logical_detection_size(); logical_detection_idx++) {
        Vector3d logical_detection_in_world_coordinates = TransformationFunctions::transform_position_from_ego_to_world_coordinates(in.logical_detection_data().logical_detection(logical_detection_idx).position(), ego_data);
        Vector3d logical_detection_in_object_coordinates = TransformationFunctions::transform_to_local_coordinates(logical_detection_in_world_coordinates,
                                                                                                                   in.sensor_view(0).global_ground_truth().moving_object(object_no_in).base().orientation(),
                                                                                                                   in.sensor_view(0).global_ground_truth().moving_object(object_no_in).base().position());

        /// Check if Point is in GT Bounding Box
        if((logical_detection_in_object_coordinates.x() >= x_min) && (logical_detection_in_object_coordinates.x() <= x_max)
        && (logical_detection_in_object_coordinates.y() >= y_min) && (logical_detection_in_object_coordinates.y() <= y_max)
        && (logical_detection_in_object_coordinates.z() >= z_min) && (logical_detection_in_object_coordinates.z() <= z_max)) {
            /// Set gt object id for each point
            in.mutable_logical_detection_data()->mutable_logical_detection(logical_detection_idx)->mutable_object_id()->CopyFrom(in.sensor_view(0).global_ground_truth().moving_object(object_no_in).id());
            segment_size_of_current_object++;
        }
    }
    log("Segment size of detected object " + std::to_string(object_no_in) + " with object id " + std::to_string(in.sensor_view(0).global_ground_truth().moving_object(object_no_in).id().value()) + ": " + std::to_string(segment_size_of_current_object));
    return segment_size_of_current_object;
}