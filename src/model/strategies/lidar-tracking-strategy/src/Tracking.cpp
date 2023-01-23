//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "tracking/Tracking.hpp"

#include <string>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>

#endif

using namespace model;
using namespace osi3;

void Tracking::apply(SensorData& sensor_data)
{
    log("Starting tracking");

    Tracking::Data data_of_current_time_step;

    if ((sensor_data.sensor_view_size() == 0) || !sensor_data.sensor_view(0).has_global_ground_truth() || !sensor_data.sensor_view(0).global_ground_truth().has_timestamp())
    {
        alert("No valid sensor view available!");
        return;
    }

    data_of_current_time_step.sensor_data.mutable_timestamp()->set_nanos(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
    data_of_current_time_step.sensor_data.mutable_timestamp()->set_seconds(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());
    sensor_data.mutable_timestamp()->set_nanos(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
    sensor_data.mutable_timestamp()->set_seconds(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());

    calculate_delta_t_of_current_time_step(data_of_current_time_step);

    TF::EgoData ego_data;
    if (!TF::get_ego_info(ego_data, sensor_data.sensor_view(0)))
    {
        alert("Ego vehicle has no base, no id, or is not contained in GT moving objects.");
        return;
    }

    log("Ego car " + std::to_string(ego_data.ego_vehicle_id.value()) + ": absolute velocity: " +
        std::to_string(std::sqrt(std::pow(ego_data.ego_base.velocity().x(), 2) + std::pow(ego_data.ego_base.velocity().y(), 2) + std::pow(ego_data.ego_base.velocity().z(), 2)) *
                       3.6) +
        " km/h");

    /// Loop over all GT objects in current frame
    int historical_object_no = 0;
    for (const MovingObject& current_GT_object : sensor_data.sensor_view(0).global_ground_truth().moving_object())
    {

        if (!current_GT_object.has_base())
        {
            alert("GT Object has no base information!");
            return;
        }

        /// Continue in case of ego car
        auto object_id = current_GT_object.id().value();
        if (object_id == ego_data.ego_vehicle_id.value())
        {
            continue;
        }

        /// Start detected object for current time stamp
        DetectedMovingObject* current_moving_object = data_of_current_time_step.sensor_data.mutable_moving_object()->Add();
        DetectedMovingObject::CandidateMovingObject* current_moving_object_candidate = current_moving_object->add_candidate();

        bool object_contained_in_history = false;
        bool object_tracked_in_history = false;

        /// Check and get object ID
        if (current_GT_object.has_id())
        {
            current_moving_object->mutable_header()->add_ground_truth_id()->CopyFrom(current_GT_object.id());
            current_moving_object->mutable_header()->mutable_tracking_id()->CopyFrom(current_GT_object.id());
        }
        else
        {
            alert("Object " + std::to_string(current_GT_object.id().value()) + " has no id!");
            continue;
        }
        transform_gt_object_to_ego_coordinate_system(current_GT_object, current_moving_object, ego_data);

        find_object_in_history(object_contained_in_history, historical_object_no, current_moving_object, object_tracked_in_history);

        get_pcl_segment_of_current_object(sensor_data.logical_detection_data(), data_of_current_time_step, current_GT_object.id().value(), ego_data);
        if (!data_of_current_time_step.pcl_segment_points_in_ego_coordinates.empty())
        {
            if (profile.tracking_parameters.dimension_and_position_flag > 0)
            {  // 0: Idealized position and dimension from GT
                calculate_dimension_and_position_from_pcl(
                    current_GT_object, data_of_current_time_step, current_moving_object, object_contained_in_history, historical_object_no, ego_data);
            }
            if (profile.tracking_parameters.dimension_and_position_flag > 0)
            {  // 0: Idealized orientation from GT
                calculate_orientation_from_pcl(data_of_current_time_step, current_moving_object);
            }
            if (profile.tracking_parameters.velocity_flag > 0)
            {  // 0: Idealized velocity from GT
                calculate_velocity_from_pcl(sensor_data, data_of_current_time_step, current_moving_object, object_contained_in_history, historical_object_no);
            }
        }
        else
        {
            log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) + ": NO logical detections in bounding box!");
            if (object_contained_in_history && history.sensor_data.back().moving_object(historical_object_no).has_base())
            {
                if (profile.tracking_parameters.dimension_and_position_flag > 0)
                {  // 0: Idealized position and dimension from GT
                    calculate_dimension_and_position_from_history(data_of_current_time_step, current_moving_object, historical_object_no);
                }
                if (profile.tracking_parameters.dimension_and_position_flag > 0)
                {  // 0: Idealized orientation from GT
                    calculate_orientation_from_history(data_of_current_time_step, current_moving_object, historical_object_no);
                }
                if (profile.tracking_parameters.velocity_flag > 0)
                {  // 0: Idealized velocity from GT
                    current_moving_object->mutable_base()->mutable_velocity()->CopyFrom(history.sensor_data.back().moving_object(historical_object_no).base().velocity());
                }
            }
        }
        log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) + ": position: x: " + std::to_string(current_moving_object->base().position().x()) +
            " m, y: " + std::to_string(current_moving_object->base().position().y()) + " m, z: " + std::to_string(current_moving_object->base().position().z()) + " m");
        log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) +
            ": orientation: roll: " + std::to_string(current_moving_object->base().orientation().roll() * 180 / M_PI) +
            "°, pitch: " + std::to_string(current_moving_object->base().orientation().pitch() * 180 / M_PI) +
            "°, yaw: " + std::to_string(current_moving_object->base().orientation().yaw() * 180 / M_PI) + "°");
        log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) + ": relative velocity: x:" +
            std::to_string(current_moving_object->base().velocity().x() * 3.6) + " km/h, y:" + std::to_string(current_moving_object->base().velocity().y() * 3.6) +
            " km/h, z:" + std::to_string(current_moving_object->base().velocity().z() * 3.6) + " km/h");

        if (object_contained_in_history && history.sensor_data.back().moving_object(historical_object_no).has_base())
        {
            if (profile.tracking_parameters.tracking_flag == 0)
            {  // 0: Idealized tracking
                current_moving_object->mutable_header()->CopyFrom(history.sensor_data.back().moving_object(historical_object_no).header());
                current_moving_object->mutable_header()->set_existence_probability(1);
                current_moving_object->mutable_header()->set_age(history.sensor_data.back().moving_object(historical_object_no).header().age() + 1);
            }
            else
            {
                continue_tracking_for_current_pcl_segment(
                    data_of_current_time_step, current_moving_object, current_GT_object, object_tracked_in_history, historical_object_no, ego_data);
            }
        }
        else
        {
            start_tracking_for_current_pcl_segment(data_of_current_time_step, current_moving_object, current_GT_object);
        }
        limit_existence_probability_to_0_and_1(current_moving_object);
        log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) + ": existence_probability: " +
            std::to_string(current_moving_object->header().existence_probability()) + ", age: " + std::to_string(current_moving_object->header().age()));

        determine_object_type(current_moving_object_candidate, current_GT_object);
        log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) + ": type: " + get_type_string(current_moving_object_candidate) +
            ", vehicle type: " + get_vehicle_type_string(current_moving_object_candidate));
        if (!profile.sensor_view_configuration.radar_sensor_view_configuration().empty())
        {  // radar
            set_rcs(current_moving_object);
            log("Object " + std::to_string(current_moving_object->header().tracking_id().value()) + ": rcs: " + std::to_string(current_moving_object->radar_specifics().rcs()));
        }
    }  // End of loop over all GT objects

    if (!data_of_current_time_step.sensor_data.moving_object().empty())
    {
        update_history(data_of_current_time_step, sensor_data);
        sensor_data.mutable_moving_object()->Clear();
        write_data_back_to_osi(sensor_data, data_of_current_time_step);
    }
    else
    {
        log("No moving objects detected in this time stamp!");
        sensor_data.mutable_moving_object()->Clear();
    }
}

void Tracking::determine_object_type(DetectedMovingObject_CandidateMovingObject* current_moving_object_candidate, const MovingObject& current_GT_object)
{
    if (current_GT_object.has_vehicle_classification() && current_GT_object.vehicle_classification().has_type() && profile.tracking_parameters.classification_flag == 0)
    {
        current_moving_object_candidate->mutable_vehicle_classification()->set_type(current_GT_object.vehicle_classification().type());
    }
    else
    {
        log("Object " + std::to_string(current_GT_object.id().value()) + " has no vehicle type!");
        current_moving_object_candidate->mutable_vehicle_classification()->set_type(MovingObject_VehicleClassification_Type_TYPE_UNKNOWN);
    }
    if (current_GT_object.has_type() && profile.tracking_parameters.classification_flag == 0)
    {
        current_moving_object_candidate->set_type(current_GT_object.type());
    }
    else
    {
        log("Object " + std::to_string(current_GT_object.id().value()) + " has no type!");
        current_moving_object_candidate->set_type(MovingObject_Type_TYPE_UNKNOWN);
    }

    current_moving_object_candidate->set_probability(1);
}

void Tracking::set_rcs(DetectedMovingObject* current_moving_object)
{
    double rcs_dbsm;
    if (current_moving_object->candidate(0).type() == osi3::MovingObject_Type_TYPE_VEHICLE)
    {
        if (current_moving_object->candidate(0).vehicle_classification().type() <= osi3::MovingObject_VehicleClassification::TYPE_LUXURY_CAR)
        {
            rcs_dbsm = 10;
        }
        else if (current_moving_object->candidate(0).vehicle_classification().type() <= osi3::MovingObject_VehicleClassification::TYPE_TRAILER)
        {
            rcs_dbsm = 45;
        }
        else if (current_moving_object->candidate(0).vehicle_classification().type() == osi3::MovingObject_VehicleClassification::TYPE_MOTORBIKE)
        {
            rcs_dbsm = 0;
        }
        else if (current_moving_object->candidate(0).vehicle_classification().type() == osi3::MovingObject_VehicleClassification::TYPE_BICYCLE)
        {
            rcs_dbsm = -5;
        }
        else
        {
            rcs_dbsm = 40;
        }
    }
    else if (current_moving_object->candidate(0).type() == osi3::MovingObject_Type_TYPE_PEDESTRIAN)
    {
        rcs_dbsm = -10;
    }
    current_moving_object->mutable_radar_specifics()->set_rcs(rcs_dbsm);
}

void Tracking::transform_gt_object_to_ego_coordinate_system(const MovingObject& current_GT_object, DetectedMovingObject* current_moving_object, const TF::EgoData& ego_data)
{

    /// Relative position of the object in the ego coordinate system (x_rel)
    current_moving_object->mutable_base()->mutable_position()->CopyFrom(TF::transform_position_from_world_to_ego_coordinates(current_GT_object.base().position(), ego_data));

    /// Relative orientation of object (delta)
    current_moving_object->mutable_base()->mutable_orientation()->CopyFrom(
        TF::calc_relative_orientation_to_local(current_GT_object.base().orientation(), ego_data.ego_base.orientation()));

    /// Relative velocity of object in ego coordinate system
    current_moving_object->mutable_base()->mutable_velocity()->CopyFrom(
        TF::transform_to_local_coordinates(current_GT_object.base().velocity(), ego_data.ego_base.orientation(), ego_data.ego_base.velocity()));

    /// Relative orientation rate of object (delta_rate)
    current_moving_object->mutable_base()->mutable_orientation_rate()->CopyFrom(
        TF::calc_relative_orientation_to_local(current_GT_object.base().orientation_rate(), ego_data.ego_base.orientation_rate()));

    /// Relative acceleration of object in ego coordinate system
    current_moving_object->mutable_base()->mutable_acceleration()->CopyFrom(
        TF::transform_to_local_coordinates(current_GT_object.base().acceleration(), ego_data.ego_base.orientation(), ego_data.ego_base.acceleration()));

    /// Relative orientation_acceleration of object (delta_delta_rate)
    current_moving_object->mutable_base()->mutable_orientation_acceleration()->CopyFrom(
        TF::calc_relative_orientation_to_local(current_GT_object.base().orientation_acceleration(), ego_data.ego_base.orientation_acceleration()));
}

void Tracking::get_pcl_segment_of_current_object(const LogicalDetectionData& logical_detection_data,
                                                 Tracking::Data& data_of_current_time_step,
                                                 size_t gt_object_id,
                                                 const TF::EgoData& ego_data)
{
    data_of_current_time_step.pcl_segment_points_in_ego_coordinates.clear();
    data_of_current_time_step.pcl_segment_points_in_ego_coordinates.reserve(logical_detection_data.logical_detection_size());
    for (const auto& logical_detection : logical_detection_data.logical_detection())
    {
        if (logical_detection.object_id().value() == gt_object_id)
        {
            data_of_current_time_step.pcl_segment_points_in_ego_coordinates.emplace_back(logical_detection.position());
        }
    }
}

void Tracking::calculate_dimension_and_position_from_pcl(const MovingObject& current_GT_object,
                                                         Data& data_of_current_time_step,
                                                         DetectedMovingObject* current_moving_object,
                                                         bool object_contained_in_history,
                                                         int historical_object_no,
                                                         const TF::EgoData& ego_data)
{

    calculate_object_dimension_and_position_in_object_from_pcl_segment(data_of_current_time_step, current_GT_object, ego_data);
    switch (profile.tracking_parameters.dimension_and_position_flag)
    {
        case 1:  // 1: From current point cloud segment
            current_moving_object->mutable_base()->mutable_dimension()->CopyFrom(data_of_current_time_step.pcl_segment_dimension);
            current_moving_object->mutable_base()->mutable_position()->CopyFrom(
                TF::transform_position_from_object_to_ego_coordinates(data_of_current_time_step.pcl_segment_position_in_object, ego_data, current_GT_object));
            break;
        case 2:  // 2: Dimension from current point cloud segments with lower bounds, position as center of manipulated
                 // pcl segment
            eliminate_ground_clearance(data_of_current_time_step.pcl_segment_dimension,
                                       current_GT_object.base().dimension(),
                                       &data_of_current_time_step.pcl_segment_dimension,
                                       data_of_current_time_step.pcl_segment_position_in_object,
                                       &data_of_current_time_step.pcl_segment_position_in_object);
            calculate_dimension_with_lower_bounds(profile.tracking_parameters.minimum_object_dimension,
                                                  data_of_current_time_step.pcl_segment_dimension,
                                                  &data_of_current_time_step.pcl_segment_dimension,
                                                  data_of_current_time_step.pcl_segment_position_in_object,
                                                  &data_of_current_time_step.pcl_segment_position_in_object);
            current_moving_object->mutable_base()->mutable_dimension()->CopyFrom(data_of_current_time_step.pcl_segment_dimension);

            current_moving_object->mutable_base()->mutable_position()->CopyFrom(
                TF::transform_position_from_object_to_ego_coordinates(data_of_current_time_step.pcl_segment_position_in_object, ego_data, current_GT_object));
            break;
        case 3:  // 3: Maximum dimension of current and mean of historical point cloud segments, position as center of
                 // manipulated pcl segment
            eliminate_ground_clearance(data_of_current_time_step.pcl_segment_dimension,
                                       current_GT_object.base().dimension(),
                                       &data_of_current_time_step.pcl_segment_dimension,
                                       data_of_current_time_step.pcl_segment_position_in_object,
                                       &data_of_current_time_step.pcl_segment_position_in_object);
            set_object_dimension_with_tracking(data_of_current_time_step.pcl_segment_dimension,
                                               &data_of_current_time_step.pcl_segment_dimension,
                                               object_contained_in_history,
                                               historical_object_no,
                                               data_of_current_time_step.pcl_segment_position_in_object,
                                               &data_of_current_time_step.pcl_segment_position_in_object);
            current_moving_object->mutable_base()->mutable_dimension()->CopyFrom(data_of_current_time_step.pcl_segment_dimension);

            current_moving_object->mutable_base()->mutable_position()->CopyFrom(
                TF::transform_position_from_object_to_ego_coordinates(data_of_current_time_step.pcl_segment_position_in_object, ego_data, current_GT_object));
            break;
        case 4:  // 4: Maximum dimension of current and mean of historical point cloud segments with lower bounds,
                 // position as center of manipulated pcl segment
            eliminate_ground_clearance(data_of_current_time_step.pcl_segment_dimension,
                                       current_GT_object.base().dimension(),
                                       &data_of_current_time_step.pcl_segment_dimension,
                                       data_of_current_time_step.pcl_segment_position_in_object,
                                       &data_of_current_time_step.pcl_segment_position_in_object);
            calculate_dimension_with_lower_bounds(profile.tracking_parameters.minimum_object_dimension,
                                                  data_of_current_time_step.pcl_segment_dimension,
                                                  &data_of_current_time_step.pcl_segment_dimension,
                                                  data_of_current_time_step.pcl_segment_position_in_object,
                                                  &data_of_current_time_step.pcl_segment_position_in_object);
            set_object_dimension_with_tracking(data_of_current_time_step.pcl_segment_dimension,
                                               &data_of_current_time_step.pcl_segment_dimension,
                                               object_contained_in_history,
                                               historical_object_no,
                                               data_of_current_time_step.pcl_segment_position_in_object,
                                               &data_of_current_time_step.pcl_segment_position_in_object);
            current_moving_object->mutable_base()->mutable_dimension()->CopyFrom(data_of_current_time_step.pcl_segment_dimension);
            current_moving_object->mutable_base()->mutable_position()->CopyFrom(
                TF::transform_position_from_object_to_ego_coordinates(data_of_current_time_step.pcl_segment_position_in_object, ego_data, current_GT_object));
            break;
    }
}

void Tracking::calculate_dimension_and_position_from_history(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, int historical_object_no)
{
    current_moving_object->mutable_base()->mutable_dimension()->CopyFrom(history.sensor_data.back().moving_object(historical_object_no).base().dimension());
    Vector3d delta_position;
    delta_position.set_x(history.sensor_data.back().moving_object(historical_object_no).base().velocity().x() * data_of_current_time_step.delta_t);
    delta_position.set_y(history.sensor_data.back().moving_object(historical_object_no).base().velocity().y() * data_of_current_time_step.delta_t);
    delta_position.set_z(history.sensor_data.back().moving_object(historical_object_no).base().velocity().z() * data_of_current_time_step.delta_t);
    current_moving_object->mutable_base()->mutable_position()->CopyFrom(
        TF::vector_translation(history.sensor_data.back().moving_object(historical_object_no).base().position(), delta_position, 1));
}

void Tracking::calculate_object_dimension_and_position_in_object_from_pcl_segment(Tracking::Data& data_of_current_time_step,
                                                                                  const MovingObject& current_GT_object,
                                                                                  const TF::EgoData& ego_data)
{

    /// Get maximal and minimal values of the points from the current pcl segment
    double x_max = INT_MIN;
    double x_min = INT_MAX;
    double y_max = INT_MIN;
    double y_min = INT_MAX;
    double z_max = INT_MIN;
    double z_min = INT_MAX;
    for (auto& point_in_ego_coordinates : data_of_current_time_step.pcl_segment_points_in_ego_coordinates)
    {
        Vector3d point_in_object = TF::transform_position_from_ego_to_object_coordinates(point_in_ego_coordinates, ego_data, current_GT_object);
        if (point_in_object.x() < x_min)
        {
            x_min = point_in_object.x();
        }
        if (point_in_object.x() > x_max)
        {
            x_max = point_in_object.x();
        }
        if (point_in_object.y() < y_min)
        {
            y_min = point_in_object.y();
        }
        if (point_in_object.y() > y_max)
        {
            y_max = point_in_object.y();
        }
        if (point_in_object.z() < z_min)
        {
            z_min = point_in_object.z();
        }
        if (point_in_object.z() > z_max)
        {
            z_max = point_in_object.z();
        }
    }
    data_of_current_time_step.pcl_segment_dimension.set_length(x_max - x_min);
    data_of_current_time_step.pcl_segment_dimension.set_width(y_max - y_min);
    data_of_current_time_step.pcl_segment_dimension.set_height(z_max - z_min);
    data_of_current_time_step.pcl_segment_position_in_object.set_x((x_max + x_min) / 2);
    data_of_current_time_step.pcl_segment_position_in_object.set_y((y_max + y_min) / 2);
    data_of_current_time_step.pcl_segment_position_in_object.set_z((z_max + z_min) / 2);
}

void Tracking::eliminate_ground_clearance(const Dimension3d& current_dimension,
                                          const Dimension3d& current_gt_object_dimension,
                                          Dimension3d* new_dimension,
                                          Vector3d& current_position_in_object,
                                          Vector3d* new_position_in_object)
{
    new_dimension->CopyFrom(current_dimension);
    double height_diff_to_gt = current_gt_object_dimension.height() - current_dimension.height();
    double ground_clearance = current_position_in_object.z() + height_diff_to_gt / 2;
    new_dimension->set_height(current_dimension.height() + ground_clearance);
    new_position_in_object->set_z(current_position_in_object.z() - ground_clearance / 2);
}

[[maybe_unused]] void Tracking::limit_to_gt_height(Vector3d& pcl_segment_position_in_object,
                                                   const Dimension3d& current_dimension,
                                                   const Dimension3d& current_gt_object_dimension,
                                                   Dimension3d* new_dimension)
{
    new_dimension->CopyFrom(current_dimension);
    double height_diff_to_gt = current_dimension.height() - current_gt_object_dimension.height();
    if (height_diff_to_gt > 0)
    {
        new_dimension->set_height(current_gt_object_dimension.height());
        pcl_segment_position_in_object.set_z(0);
    }
}

void Tracking::calculate_dimension_with_lower_bounds(const Dimension3d& lower_bounds_dimension,
                                                     const Dimension3d& current_pcl_segment_dimension,
                                                     Dimension3d* new_pcl_segment_dimension,
                                                     const Vector3d& current_pcl_segment_position_in_object,
                                                     Vector3d* new_pcl_segment_position_in_object)
{
    if (lower_bounds_dimension.length() > current_pcl_segment_dimension.length())
    {
        new_pcl_segment_position_in_object->set_x(current_pcl_segment_position_in_object.x() - copysign(1.0, current_pcl_segment_position_in_object.x()) *
                                                                                                   (lower_bounds_dimension.length() - current_pcl_segment_dimension.length()) / 2);
        new_pcl_segment_dimension->set_length(lower_bounds_dimension.length());
    }
    if (lower_bounds_dimension.width() > current_pcl_segment_dimension.width())
    {
        new_pcl_segment_position_in_object->set_y(current_pcl_segment_position_in_object.y() - copysign(1.0, current_pcl_segment_position_in_object.y()) *
                                                                                                   (lower_bounds_dimension.width() - current_pcl_segment_dimension.width()) / 2);
        new_pcl_segment_dimension->set_width(lower_bounds_dimension.width());
    }
    if (lower_bounds_dimension.height() > current_pcl_segment_dimension.height())
    {
        new_pcl_segment_position_in_object->set_z(current_pcl_segment_position_in_object.z() + (lower_bounds_dimension.height() - current_pcl_segment_dimension.height()) / 2);
        new_pcl_segment_dimension->set_height(lower_bounds_dimension.height());
    }
}

void Tracking::set_object_dimension_with_tracking(const Dimension3d& current_dimension,
                                                  Dimension3d* new_dimension,
                                                  bool object_contained_in_history,
                                                  int historical_object_no,
                                                  const Vector3d& current_pcl_segment_position_in_object,
                                                  Vector3d* new_pcl_segment_position_in_object) const
{
    if (!history.sensor_data.empty() && object_contained_in_history)
    {
        Dimension3d historical_dimensions_sum = current_dimension;
        size_t no_of_historical_data_used = 1;
        for (auto historical_data = history.sensor_data.crbegin(); historical_data != history.sensor_data.crend(); ++historical_data)
        {
            if (historical_data->moving_object_size() > historical_object_no && historical_data->moving_object(historical_object_no).has_base())
            {
                if ((historical_data->moving_object(historical_object_no).base().dimension().length() >= 0) &&
                    (historical_data->moving_object(historical_object_no).base().dimension().width() >= 0) &&
                    (historical_data->moving_object(historical_object_no).base().dimension().height() >= 0))
                {
                    historical_dimensions_sum.set_length(historical_data->moving_object(historical_object_no).base().dimension().length() + historical_dimensions_sum.length());
                    historical_dimensions_sum.set_width(historical_data->moving_object(historical_object_no).base().dimension().width() + historical_dimensions_sum.width());
                    historical_dimensions_sum.set_height(historical_data->moving_object(historical_object_no).base().dimension().height() + historical_dimensions_sum.height());
                    no_of_historical_data_used++;
                    if (no_of_historical_data_used == profile.tracking_parameters.historical_limit_dimension)
                    {
                        break;
                    }
                }
            }
        }
        new_dimension->set_length(historical_dimensions_sum.length() / double(no_of_historical_data_used));
        new_dimension->set_width(historical_dimensions_sum.width() / double(no_of_historical_data_used));
        new_dimension->set_height(historical_dimensions_sum.height() / double(no_of_historical_data_used));
    }
}

void Tracking::calculate_velocity_from_pcl(SensorData& in,
                                           Data& data_of_current_time_step,
                                           DetectedMovingObject* current_moving_object,
                                           bool object_contained_in_history,
                                           int historical_object_no)
{
    if (profile.tracking_parameters.velocity_flag == 1 && object_contained_in_history && history.sensor_data.back().moving_object(historical_object_no).has_base())
    {
        calculate_velocity_as_derivation_of_position(data_of_current_time_step, current_moving_object, object_contained_in_history);
    }
}

void Tracking::calculate_velocity_as_derivation_of_position(Tracking::Data& data_of_current_time_step,
                                                            DetectedMovingObject* current_moving_object,
                                                            bool object_contained_in_history) const
{
    Vector3d velocity;
    velocity.set_x(0);
    velocity.set_y(0);
    velocity.set_z(0);

    if (object_contained_in_history)
    {
        int counter = 0;
        Vector3d historical_position;
        for (auto historical_data = history.sensor_data.crbegin(); historical_data != history.sensor_data.crend(); ++historical_data)
        {
            for (int old_object_no = 0; old_object_no < historical_data->moving_object_size(); old_object_no++)
            {
                if (current_moving_object->header().tracking_id().value() == historical_data->moving_object(old_object_no).header().tracking_id().value())
                {
                    historical_position = historical_data->moving_object(old_object_no).base().position();
                    counter++;
                    break;
                }
            }
            if (counter == profile.tracking_parameters.historical_limit_velocity)
            {
                break;
            }
        }
        Vector3d difference_position = TF::vector_translation(current_moving_object->base().position(), historical_position, -1);
        velocity.set_x(difference_position.x() / (counter * data_of_current_time_step.delta_t));
        velocity.set_y(difference_position.y() / (counter * data_of_current_time_step.delta_t));
        velocity.set_z(difference_position.z() / (counter * data_of_current_time_step.delta_t));
    }
    current_moving_object->mutable_base()->mutable_velocity()->CopyFrom(velocity);
}

void Tracking::calculate_orientation_from_pcl(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object)
{
    // TODO fill this function
}

void Tracking::calculate_orientation_from_history(Tracking::Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, int historical_object_no)
{
    current_moving_object->mutable_base()->mutable_orientation()->set_roll(history.sensor_data.back().moving_object(historical_object_no).base().orientation().roll() +
                                                                           history.sensor_data.back().moving_object(historical_object_no).base().orientation_rate().roll() *
                                                                               data_of_current_time_step.delta_t);
    current_moving_object->mutable_base()->mutable_orientation()->set_pitch(history.sensor_data.back().moving_object(historical_object_no).base().orientation().pitch() +
                                                                            history.sensor_data.back().moving_object(historical_object_no).base().orientation_rate().pitch() *
                                                                                data_of_current_time_step.delta_t);
    current_moving_object->mutable_base()->mutable_orientation()->set_yaw(history.sensor_data.back().moving_object(historical_object_no).base().orientation().yaw() +
                                                                          history.sensor_data.back().moving_object(historical_object_no).base().orientation_rate().yaw() *
                                                                              data_of_current_time_step.delta_t);
}

void Tracking::continue_tracking_for_current_pcl_segment(Data& data_of_current_time_step,
                                                         DetectedMovingObject* current_moving_object,
                                                         const MovingObject& current_GT_object,
                                                         bool object_tracked_in_history,
                                                         int historical_object_no,
                                                         const TF::EgoData& ego_data)
{
    double current_absolute_gt_velocity = TF::get_vector_abs(current_GT_object.base().velocity());
    current_moving_object->mutable_header()->CopyFrom(history.sensor_data.back().moving_object(historical_object_no).header());
    if (object_tracked_in_history)
    {
        current_moving_object->mutable_header()->set_age(history.sensor_data.back().moving_object(historical_object_no).header().age() + 1);
        if (!data_of_current_time_step.pcl_segment_points_in_ego_coordinates.empty())
        {
            /// Increase existence probability
            current_moving_object->mutable_header()->set_existence_probability(history.sensor_data.back().moving_object(historical_object_no).header().existence_probability() +
                                                                               profile.tracking_parameters.existence_probability_increment);

            if (history.object_was_moving.back()[historical_object_no])
            {
                data_of_current_time_step.object_is_moving.push_back(true);
            }
            else
            {
                data_of_current_time_step.object_is_moving.push_back(current_absolute_gt_velocity > 0);
            }
        }
        else
        {  // Object currently not visible
            /// Reduce existence probability
            current_moving_object->mutable_header()->set_existence_probability(history.sensor_data.back().moving_object(historical_object_no).header().existence_probability() -
                                                                               profile.tracking_parameters.existence_probability_decrement);

            /// If object is still tracked
            if (current_moving_object->header().existence_probability() >= profile.tracking_parameters.existence_probability_threshold_for_tracking)
            {

                /// Copy information from last step
                current_moving_object->mutable_candidate()->CopyFrom(history.sensor_data.back().moving_object(historical_object_no).candidate());
                data_of_current_time_step.object_is_moving.push_back(history.object_was_moving.back()[historical_object_no]);
            }
            else
            {
                /// Reset age to 0 if object tracking ends (existence probability < threshold)
                current_moving_object->mutable_header()->set_age(0);
                data_of_current_time_step.object_is_moving.push_back(false);
            }
        }
    }
    else
    {  // Object wasn't tracked in the last time step
        if ((data_of_current_time_step.pcl_segment_points_in_ego_coordinates.size() > profile.tracking_parameters.min_detections_in_segment_for_tracking) ||
            (!data_of_current_time_step.pcl_segment_points_in_ego_coordinates.empty() && current_absolute_gt_velocity > 0))
        {
            if (!history.sensor_data.back().moving_object(historical_object_no).header().has_existence_probability())
            {
                current_moving_object->mutable_header()->set_existence_probability(profile.tracking_parameters.existence_probability_increment);
            }
            else
            {
                current_moving_object->mutable_header()->set_existence_probability(history.sensor_data.back().moving_object(historical_object_no).header().existence_probability() +
                                                                                   profile.tracking_parameters.existence_probability_increment);
            }  /// Start counting the age at the time the object tracking is being started (existence probability >
               /// threshold)
            if (current_moving_object->header().existence_probability() >= profile.tracking_parameters.existence_probability_threshold_for_tracking)
            {
                current_moving_object->mutable_header()->set_age(1);
                data_of_current_time_step.object_is_moving.push_back(current_absolute_gt_velocity > 0);
            }
            else
            {
                data_of_current_time_step.object_is_moving.push_back(false);
            }
        }
        else
        {
            current_moving_object->mutable_header()->set_existence_probability(history.sensor_data.back().moving_object(historical_object_no).header().existence_probability() -
                                                                               profile.tracking_parameters.existence_probability_decrement);
            data_of_current_time_step.object_is_moving.push_back(false);
        }
    }
}

void Tracking::start_tracking_for_current_pcl_segment(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, const MovingObject& current_GT_object)
{
    double current_absolute_gt_velocity =
        sqrt(pow(current_GT_object.base().velocity().x(), 2) + pow(current_GT_object.base().velocity().y(), 2) + pow(current_GT_object.base().velocity().z(), 2));
    if ((data_of_current_time_step.pcl_segment_points_in_ego_coordinates.size() > profile.tracking_parameters.min_detections_in_segment_for_tracking) ||
        (!data_of_current_time_step.pcl_segment_points_in_ego_coordinates.empty() && current_absolute_gt_velocity > 0))
    {
        current_moving_object->mutable_header()->set_age(0);
        current_moving_object->mutable_header()->set_existence_probability(profile.tracking_parameters.existence_probability_increment);
    }
    else
    {
        current_moving_object->mutable_header()->set_age(0);
        current_moving_object->mutable_header()->set_existence_probability(0);
    }
    data_of_current_time_step.object_is_moving.push_back(false);
}

void Tracking::limit_existence_probability_to_0_and_1(DetectedMovingObject* current_moving_object)
{
    if (current_moving_object->mutable_header()->existence_probability() < 0)
    {
        current_moving_object->mutable_header()->set_existence_probability(0);
    }
    if (current_moving_object->mutable_header()->existence_probability() > 1)
    {
        current_moving_object->mutable_header()->set_existence_probability(1);
    }
}

void Tracking::update_history(Tracking::Data& data_of_current_time_step, const SensorData& in)
{
    history.sensor_data.emplace_back(data_of_current_time_step.sensor_data);
    history.sensor_data.back().add_sensor_view();
    history.sensor_data.back().mutable_sensor_view(0)->mutable_global_ground_truth()->mutable_moving_object()->CopyFrom(in.sensor_view(0).global_ground_truth().moving_object());
    if (history.sensor_data.size() > std::max(profile.tracking_parameters.historical_limit_velocity, profile.tracking_parameters.historical_limit_dimension))
    {
        history.sensor_data.pop_front();
    }

    history.pointcloud_segment_size.emplace_back(data_of_current_time_step.pcl_segment_points_in_ego_coordinates.size());
    if (history.pointcloud_segment_size.size() > std::max(profile.tracking_parameters.historical_limit_velocity, profile.tracking_parameters.historical_limit_dimension))
    {
        history.pointcloud_segment_size.pop_front();
    }

    history.object_was_moving.emplace_back(data_of_current_time_step.object_is_moving);
    if (history.object_was_moving.size() > std::max(profile.tracking_parameters.historical_limit_velocity, profile.tracking_parameters.historical_limit_dimension))
    {
        history.object_was_moving.pop_front();
    }
}

void Tracking::write_data_back_to_osi(SensorData& in, Tracking::Data& data_of_current_time_step)
{
    for (const osi3::DetectedMovingObject& current_moving_object : data_of_current_time_step.sensor_data.moving_object())
    {
        if (current_moving_object.header().existence_probability() >= profile.tracking_parameters.existence_probability_threshold_for_tracking)
        {
            DetectedMovingObject* added_output_moving_object = in.mutable_moving_object()->Add();
            added_output_moving_object->mutable_header()->CopyFrom(current_moving_object.header());
            added_output_moving_object->mutable_candidate()->CopyFrom(current_moving_object.candidate());
            added_output_moving_object->mutable_base()->CopyFrom(current_moving_object.base());
        }
    }
}

void Tracking::calculate_delta_t_of_current_time_step(Tracking::Data& data_of_current_time_step)
{
    if (!history.sensor_data.empty() && history.sensor_data.back().has_timestamp())
    {
        data_of_current_time_step.delta_t = (double)data_of_current_time_step.sensor_data.timestamp().seconds() +
                                            double(data_of_current_time_step.sensor_data.timestamp().nanos()) / 1000000000.0 -
                                            ((double)history.sensor_data.back().timestamp().seconds() + double(history.sensor_data.back().timestamp().nanos()) / 1000000000);
    }
    else
    {
        data_of_current_time_step.delta_t = 0;
    }
}

void Tracking::find_object_in_history(bool& object_contained_in_history, int& historical_object_no, DetectedMovingObject* current_moving_object, bool& object_tracked_in_history)
{
    if (!history.sensor_data.empty())
    {
        if (history.sensor_data.back().sensor_view(0).global_ground_truth().moving_object_size() > 0)
        {
            object_contained_in_history = false;
            object_tracked_in_history = false;
            for (int old_object_no = 0; old_object_no < history.sensor_data.back().moving_object_size(); old_object_no++)
            {
                // check if object appeared in last time step
                if (current_moving_object->header().tracking_id().value() == history.sensor_data.back().moving_object(old_object_no).header().tracking_id().value())
                {
                    historical_object_no = old_object_no;
                    object_contained_in_history = true;
                    if (history.sensor_data.back().moving_object(old_object_no).header().existence_probability() >=
                        profile.tracking_parameters.existence_probability_threshold_for_tracking)
                    {
                        object_tracked_in_history = true;
                    }
                }
            }
        }
    }
}

std::string Tracking::get_type_string(const DetectedMovingObject_CandidateMovingObject* current_moving_object_candidate)
{
    std::string type_string;
    switch (current_moving_object_candidate->type())
    {
        case 0:
            type_string = "UNKNOWN";
            break;
        case 1:
            type_string = "OTHER";
            break;
        case 2:
            type_string = "VEHICLE";
            break;
        case 3:
            type_string = "PEDESTRIAN";
            break;
        case 4:
            type_string = "ANIMAL";
            break;
    }
    return type_string;
}

std::string Tracking::get_vehicle_type_string(const DetectedMovingObject_CandidateMovingObject* current_moving_object_candidate)
{
    std::string vehicle_type_string;
    switch (current_moving_object_candidate->vehicle_classification().type())
    {
        case MovingObject_VehicleClassification_Type_TYPE_UNKNOWN:
            vehicle_type_string = "UNKNOWN";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_OTHER:
            vehicle_type_string = "OTHER";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_SMALL_CAR:
            vehicle_type_string = "SMALL_CAR";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_COMPACT_CAR:
            vehicle_type_string = "COMPACT_CAR";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_MEDIUM_CAR:
            vehicle_type_string = "MEDIUM_CAR";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_LUXURY_CAR:
            vehicle_type_string = "LUXURY_CAR";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_DELIVERY_VAN:
            vehicle_type_string = "DELIVERY_VAN";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_HEAVY_TRUCK:
            vehicle_type_string = "HEAVY_TRUCK";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_SEMITRAILER:
            vehicle_type_string = "SEMITRAILER";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_TRAILER:
            vehicle_type_string = "TRAILER";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_MOTORBIKE:
            vehicle_type_string = "MOTORBIKE";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_BICYCLE:
            vehicle_type_string = "BICYCLE";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_BUS:
            vehicle_type_string = "BUS";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_TRAM:
            vehicle_type_string = "TRAM";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_TRAIN:
            vehicle_type_string = "TRAIN";
            break;
        case MovingObject_VehicleClassification_Type_TYPE_WHEELCHAIR:
            vehicle_type_string = "WHEELCHAIR";
            break;
    }
    return vehicle_type_string;
}