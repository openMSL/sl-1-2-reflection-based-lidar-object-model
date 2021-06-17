//
// Copyright Clemens Linnhoff, M. Sc.
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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#define PREV_INDEX(itr, total_num)   ((itr - 1 + total_num) % total_num)
#define NEXT_INDEX(itr, total_num)   ((itr + 1    ) % total_num)

#include "model/strategies/transformation-functions/TransformationFunctions.hpp"
#include <string>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

using namespace model;
using namespace osi3;

Vector3d TransformationFunctions::vector_translation(const Vector3d &vector_a, const Vector3d &vector_b, double factor_for_vector_b) {

    Vector3d vector_result;
    vector_result.set_x(vector_a.x() + factor_for_vector_b * vector_b.x());
    vector_result.set_y(vector_a.y() + factor_for_vector_b * vector_b.y());
    vector_result.set_z(vector_a.z() + factor_for_vector_b * vector_b.z());
    return vector_result;
}

double TransformationFunctions::dot_product(const Vector3d &vector_a, const Vector3d &vector_b) {
    return vector_a.x()*vector_b.x() + vector_a.y()*vector_b.y() + vector_a.z()*vector_b.z();
}

Vector3d TransformationFunctions::cross_product(const Vector3d &vector_a, const Vector3d &vector_b) {
    Vector3d vector_result;
    vector_result.set_x(vector_a.y()*vector_b.z() - vector_a.z()*vector_b.y());
    vector_result.set_y(vector_a.z()*vector_b.x() - vector_a.x()*vector_b.z());
    vector_result.set_z(vector_a.x()*vector_b.y() - vector_a.y()*vector_b.x());
    return vector_result;
}

Orientation3d TransformationFunctions::calc_relative_orientation_to_local(const Orientation3d& object_orientation, const Orientation3d& ego_orientation) {

    Eigen::Matrix3d ego_rotation_matrix = calc_rotation_matrix_from_euler_angles(ego_orientation);
    Eigen::Matrix3d object_rotation_matrix = calc_rotation_matrix_from_euler_angles(object_orientation);
    Eigen::Matrix3d resulting_rotation_matrix = object_rotation_matrix * ego_rotation_matrix.transpose();

    Orientation3d relative_orientation = calc_euler_angles_from_rotation_matrix(resulting_rotation_matrix);

    return relative_orientation;
}

Orientation3d TransformationFunctions::calc_relative_orientation_from_local(const Orientation3d& object_orientation, const Orientation3d& ego_orientation) {

    Eigen::Matrix3d ego_rotation_matrix = calc_rotation_matrix_from_euler_angles(ego_orientation);
    Eigen::Matrix3d object_rotation_matrix = calc_rotation_matrix_from_euler_angles(object_orientation);
    Eigen::Matrix3d resulting_rotation_matrix = object_rotation_matrix * ego_rotation_matrix;

    Orientation3d relative_orientation = calc_euler_angles_from_rotation_matrix(resulting_rotation_matrix);

    return relative_orientation;
}

Eigen::Matrix3d TransformationFunctions::calc_rotation_matrix_from_euler_angles(const Orientation3d& orientation) {
    Eigen::Matrix3d R;
    R(0,0) = cos(orientation.pitch()) * cos(orientation.yaw());
    R(0,1) = cos(orientation.pitch()) * sin(orientation.yaw());
    R(0,2) = -sin(orientation.pitch());
    R(1,0) = sin(orientation.roll()) * sin(orientation.pitch()) * cos(orientation.yaw()) - cos(orientation.roll()) * sin(orientation.yaw());
    R(1,1) = sin(orientation.roll()) * sin(orientation.pitch()) * sin(orientation.yaw())  + cos(orientation.roll()) * cos(orientation.yaw());
    R(1,2) = sin(orientation.roll()) * cos(orientation.pitch());
    R(2,0) = cos(orientation.roll()) * sin(orientation.pitch()) * cos(orientation.yaw()) + sin(orientation.yaw()) * sin(orientation.roll());
    R(2,1) = cos(orientation.roll()) * sin(orientation.pitch()) * sin(orientation.yaw()) - sin(orientation.roll()) * cos(orientation.yaw());
    R(2,2) = cos(orientation.roll()) * cos(orientation.pitch());
    return R;
}

Orientation3d TransformationFunctions::calc_euler_angles_from_rotation_matrix(Eigen::Matrix3d R) {
    Orientation3d orientation;
    orientation.set_pitch(-asin(R(0,2)));
    if(cos(orientation.pitch()) != 0) {
        orientation.set_yaw(atan2(R(0,1)/cos(orientation.pitch()),R(0,0)/cos(orientation.pitch())));
        orientation.set_roll(atan2(R(1,2)/cos(orientation.pitch()),R(2,2)/cos(orientation.pitch())));
    } else {
        orientation.set_yaw(0);
        orientation.set_roll(atan2(R(1,0),R(1,1)));
    }
    return orientation;
}

Vector3d TransformationFunctions::transform_to_local_coordinates(const Vector3d& input_coordinates, const Orientation3d& new_origin_rotation, const Vector3d& new_origin_translation) {
    double delta_x = input_coordinates.x() - new_origin_translation.x();
    double delta_y = input_coordinates.y() - new_origin_translation.y();
    double delta_z = input_coordinates.z() - new_origin_translation.z();

    Eigen::Matrix3d R = calc_rotation_matrix_from_euler_angles(new_origin_rotation);
    double x_rel = delta_x * R(0,0) +
                   delta_y * R(0,1) +
                   delta_z * R(0,2);
    double y_rel = delta_x * R(1,0) +
                   delta_y * R(1,1) +
                   delta_z * R(1,2);
    double z_rel = delta_x * R(2,0) +
                   delta_y * R(2,1) +
                   delta_z * R(2,2);

    Vector3d coordinates_new_origin;
    coordinates_new_origin.set_x(x_rel);
    coordinates_new_origin.set_y(y_rel);
    coordinates_new_origin.set_z(z_rel);

    return coordinates_new_origin;
}


Vector3d TransformationFunctions::transform_from_local_coordinates(const Vector3d& input_coordinates, const Orientation3d& new_origin_rotation, const Vector3d& new_origin_translation) {

    Eigen::Matrix3d R = calc_rotation_matrix_from_euler_angles(new_origin_rotation);
    double x_rel = input_coordinates.x() * R.transpose()(0,0) +
                   input_coordinates.y() * R.transpose()(0,1) +
                   input_coordinates.z() * R.transpose()(0,2);
    double y_rel = input_coordinates.x() * R.transpose()(1,0) +
                   input_coordinates.y() * R.transpose()(1,1) +
                   input_coordinates.z() * R.transpose()(1,2);
    double z_rel = input_coordinates.x() * R.transpose()(2,0) +
                   input_coordinates.y() * R.transpose()(2,1) +
                   input_coordinates.z() * R.transpose()(2,2);

    double delta_x = x_rel + new_origin_translation.x();
    double delta_y = y_rel + new_origin_translation.y();
    double delta_z = z_rel + new_origin_translation.z();

    Vector3d coordinates_new_origin;
    coordinates_new_origin.set_x(delta_x);
    coordinates_new_origin.set_y(delta_y);
    coordinates_new_origin.set_z(delta_z);

    return coordinates_new_origin;
}

Vector3d TransformationFunctions::transform_position_from_ego_to_world_coordinates(const Vector3d &position_in_ego_coordinates, const TransformationFunctions::EgoData &ego_data) {
    Vector3d position_to_ego_bb_center = vector_translation(position_in_ego_coordinates, ego_data.ego_vehicle_attributes.bbcenter_to_rear(), 1);
    Vector3d position_in_world_coordinates = transform_from_local_coordinates(position_to_ego_bb_center, ego_data.ego_base.orientation(), ego_data.ego_base.position());
    return position_in_world_coordinates;
}

Vector3d TransformationFunctions::transform_position_from_world_to_ego_coordinates(const Vector3d &input_in_world_coordinates, const TransformationFunctions::EgoData &ego_data) {
    Vector3d position_ego_bb_center = transform_to_local_coordinates(input_in_world_coordinates, ego_data.ego_base.orientation(), ego_data.ego_base.position());
    Vector3d position_ego_coordinates = vector_translation(position_ego_bb_center, ego_data.ego_vehicle_attributes.bbcenter_to_rear(), -1);
    return position_ego_coordinates;
}

Vector3d TransformationFunctions::transform_position_from_world_to_sensor_coordinates(const Vector3d &input_in_world_coordinates, const TransformationFunctions::EgoData &ego_data, const MountingPosition &mounting_pose) {
    Vector3d position_ego_bb_center = transform_to_local_coordinates(input_in_world_coordinates, ego_data.ego_base.orientation(), ego_data.ego_base.position());
    Vector3d position_ego_coordinates = vector_translation(position_ego_bb_center, ego_data.ego_vehicle_attributes.bbcenter_to_rear(), -1);
    Vector3d position_cartesian_sensor_coordinates = transform_to_local_coordinates(position_ego_coordinates, mounting_pose.orientation(), mounting_pose.position());
    return position_cartesian_sensor_coordinates;
}

Orientation3d TransformationFunctions::transform_orientation_from_world_to_sensor_coordinates(const Orientation3d &orientation_in_world_coordinates, const TransformationFunctions::EgoData &ego_data, const MountingPosition &mounting_pose) {
    Orientation3d relative_orientation_ego_coordinates = calc_relative_orientation_to_local(orientation_in_world_coordinates, ego_data.ego_base.orientation());
    Orientation3d relative_orientation_sensor_coordinates = calc_relative_orientation_to_local(relative_orientation_ego_coordinates, mounting_pose.orientation());
    return relative_orientation_sensor_coordinates;
}

Orientation3d TransformationFunctions::transform_orientation_from_sensor_to_world_coordinates(const Orientation3d &orientation_in_sensor_coordinates, const TransformationFunctions::EgoData &ego_data, const MountingPosition &mounting_pose) {
    Orientation3d relative_orientation_ego_coordinates = calc_relative_orientation_from_local(orientation_in_sensor_coordinates, mounting_pose.orientation());
    Orientation3d relative_orientation_world_coordinates = calc_relative_orientation_from_local(relative_orientation_ego_coordinates, ego_data.ego_base.orientation());
    return relative_orientation_world_coordinates;
}

Vector3d TransformationFunctions::transform_position_from_object_to_ego_coordinates(const Vector3d &pcl_segment_position_in_object, const TransformationFunctions::EgoData &ego_data, const MovingObject &gt_object) {

    Vector3d position_in_world_coordinates = TransformationFunctions::transform_from_local_coordinates(pcl_segment_position_in_object, gt_object.base().orientation(), gt_object.base().position());
    Vector3d position_in_ego_coordinates = TransformationFunctions::transform_position_from_world_to_ego_coordinates(position_in_world_coordinates, ego_data);
    return position_in_ego_coordinates;
}

Vector3d TransformationFunctions::transform_position_from_ego_to_object_coordinates(const Vector3d &pcl_segment_position_in_ego, const TransformationFunctions::EgoData &ego_data, const MovingObject &gt_object) {

    Vector3d position_in_world_coordinates = TransformationFunctions::transform_position_from_ego_to_world_coordinates(pcl_segment_position_in_ego, ego_data);
    Vector3d position_in_object_coordinates = TransformationFunctions::transform_to_local_coordinates(position_in_world_coordinates, gt_object.base().orientation(), gt_object.base().position());
    return position_in_object_coordinates;
}

Spherical3d TransformationFunctions::transform_cartesian_to_spherical(const Vector3d &input_relative_position) {
    double distance;
    double azimuth;
    double elevation;
    distance = sqrt(input_relative_position.x() * input_relative_position.x() +
                    input_relative_position.y() * input_relative_position.y() +
                    input_relative_position.z() * input_relative_position.z());
    azimuth = atan2(input_relative_position.y() , input_relative_position.x());
    elevation = asin(input_relative_position.z() / distance);
    Spherical3d spherical_coordinate;
    spherical_coordinate.set_distance(distance);
    spherical_coordinate.set_azimuth(azimuth);
    spherical_coordinate.set_elevation(elevation);
    return spherical_coordinate;
}

double TransformationFunctions::projection_to_spherical_surface(std::vector<Spherical3d> &spherical_coordinates, double distance) {
    // the input is in spherical coordinate system
    if (spherical_coordinates.size() < 3) return 0;
    double processed_prev, processed_next, ready_to_add;
    double sum_of_spherical_angle = 0.0;
    double elevation_a, azimuth_a, elevation_b, azimuth_b, elevation_c, azimuth_c;
    auto points_number = spherical_coordinates.size();
    for (uint64_t itr = 0; itr != points_number; ++itr) {
        azimuth_c = spherical_coordinates.at(NEXT_INDEX(itr, points_number)).azimuth();
        elevation_c = spherical_coordinates.at(NEXT_INDEX(itr, points_number)).elevation();
        azimuth_b = spherical_coordinates.at(itr).azimuth();
        elevation_b = spherical_coordinates.at(itr).elevation();
        azimuth_a = spherical_coordinates.at(PREV_INDEX(itr, points_number)).azimuth();
        elevation_a = spherical_coordinates.at(PREV_INDEX(itr, points_number)).elevation();
        processed_prev = atan2((sin(elevation_a - elevation_b) * cos(azimuth_a)),
                               (sin(azimuth_a) * cos(azimuth_b) - cos(azimuth_a) * sin(azimuth_b) * cos(elevation_a - elevation_b)));
        processed_next = atan2((sin(elevation_c - elevation_b) * cos(azimuth_c)),
                               (sin(azimuth_c) * cos(azimuth_b) - cos(azimuth_c) * sin(azimuth_b) * cos(elevation_c - elevation_b)));
        processed_prev < processed_next ? ready_to_add = (processed_prev - processed_next) + 2 * M_PI : ready_to_add = (processed_prev - processed_next);
        sum_of_spherical_angle += ready_to_add;
    }
    return (sum_of_spherical_angle - double(points_number - 2) * M_PI) * distance * distance;
}

void TransformationFunctions::transform_spherical_to_cartesian(const Spherical3d &input_spherical_coordinate, Vector3d &vector_xyz) {
    double x;
    double y;
    double z;
    x = input_spherical_coordinate.distance() * cos(input_spherical_coordinate.elevation()) * cos(input_spherical_coordinate.azimuth());
    y = input_spherical_coordinate.distance() * cos(input_spherical_coordinate.elevation()) * sin(input_spherical_coordinate.azimuth());
    z = input_spherical_coordinate.distance() * sin(input_spherical_coordinate.elevation());
    vector_xyz.set_x(x);
    vector_xyz.set_y(y);
    vector_xyz.set_z(z);
}

void TransformationFunctions::projection_onto_unit_distance_cylinder(const Spherical3d &input_spherical_coordinate, Vector2d &output_position_on_plane) {
    output_position_on_plane.set_x(input_spherical_coordinate.azimuth());
    output_position_on_plane.set_y(tan(input_spherical_coordinate.elevation()));
}

double TransformationFunctions::get_abs_velocity(const Vector3d& velocity_3d) {
    return sqrt(pow(velocity_3d.x(),2) + pow(velocity_3d.y(),2) + pow(velocity_3d.z(),2));
}

bool TransformationFunctions::get_ego_info(TransformationFunctions::EgoData &ego_data, const SensorView &input_sensor_view) {
    if (input_sensor_view.global_ground_truth().has_host_vehicle_id()) {
        ego_data.ego_vehicle_id = input_sensor_view.global_ground_truth().host_vehicle_id();
        bool ego_vehicle_found = false;
        uint64_t mov_obj_no = 0;
        while (!ego_vehicle_found && mov_obj_no < input_sensor_view.global_ground_truth().moving_object_size()) {
            if (input_sensor_view.global_ground_truth().moving_object(mov_obj_no).id().value() == ego_data.ego_vehicle_id.value()) {
                ego_data.ego_vehicle_no = mov_obj_no;
                ego_vehicle_found = true;
            }
            mov_obj_no++;
        }
        if (!ego_vehicle_found) {
            return false;
        }
        if (input_sensor_view.global_ground_truth().moving_object_size() > ego_data.ego_vehicle_no &&
            input_sensor_view.global_ground_truth().moving_object(ego_data.ego_vehicle_no).has_base()) {
            ego_data.ego_base.CopyFrom(input_sensor_view.global_ground_truth().moving_object(ego_data.ego_vehicle_no).base());
            ego_data.sensor_mounting_parameters.CopyFrom(input_sensor_view.mounting_position());
            ego_data.ego_vehicle_attributes.CopyFrom(input_sensor_view.global_ground_truth().moving_object(ego_data.ego_vehicle_no).vehicle_attributes());
        } else {
            return false;
        }
    } else {
        return false;
    }
    return true;
}

MountingPosition TransformationFunctions::get_sensor_mounting_pose(const Profile &profile, const Alert &alert) {
    MountingPosition mounting_pose;
    if(!profile.sensor_view_configuration.radar_sensor_view_configuration().empty()) { // radar
        mounting_pose.CopyFrom(profile.sensor_view_configuration.radar_sensor_view_configuration(0).mounting_position());
    } else if(!profile.sensor_view_configuration.lidar_sensor_view_configuration().empty()) { // lidar
        mounting_pose.CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position());
    } else {
        alert("No lidar or radar sensor view in profile!");
    }
    return mounting_pose;
}

