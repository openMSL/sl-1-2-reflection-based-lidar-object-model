//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "model/strategies/transformation-functions/TransformationFunctions.hpp"

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

using namespace model;
using namespace osi3;

Orientation3d TF::calc_relative_orientation_to_local(const Orientation3d& object_orientation, const Orientation3d& ego_orientation)
{

    Eigen::Matrix3d ego_rotation_matrix = calc_rotation_matrix_from_euler_angles(ego_orientation);
    Eigen::Matrix3d object_rotation_matrix = calc_rotation_matrix_from_euler_angles(object_orientation);
    Eigen::Matrix3d resulting_rotation_matrix = object_rotation_matrix * ego_rotation_matrix.transpose();

    Orientation3d relative_orientation = calc_euler_angles_from_rotation_matrix(resulting_rotation_matrix);

    return relative_orientation;
}

Orientation3d TF::calc_relative_orientation_from_local(const Orientation3d& object_orientation, const Orientation3d& ego_orientation)
{

    Eigen::Matrix3d ego_rotation_matrix = calc_rotation_matrix_from_euler_angles(ego_orientation);
    Eigen::Matrix3d object_rotation_matrix = calc_rotation_matrix_from_euler_angles(object_orientation);
    Eigen::Matrix3d resulting_rotation_matrix = object_rotation_matrix * ego_rotation_matrix;

    Orientation3d relative_orientation = calc_euler_angles_from_rotation_matrix(resulting_rotation_matrix);

    return relative_orientation;
}

Eigen::Matrix3d TF::calc_rotation_matrix_from_euler_angles(const Orientation3d& orientation)
{
    Eigen::Matrix3d rot_mat;
    rot_mat(0, 0) = cos(orientation.pitch()) * cos(orientation.yaw());
    rot_mat(0, 1) = cos(orientation.pitch()) * sin(orientation.yaw());
    rot_mat(0, 2) = -sin(orientation.pitch());
    rot_mat(1, 0) = sin(orientation.roll()) * sin(orientation.pitch()) * cos(orientation.yaw()) - cos(orientation.roll()) * sin(orientation.yaw());
    rot_mat(1, 1) = sin(orientation.roll()) * sin(orientation.pitch()) * sin(orientation.yaw()) + cos(orientation.roll()) * cos(orientation.yaw());
    rot_mat(1, 2) = sin(orientation.roll()) * cos(orientation.pitch());
    rot_mat(2, 0) = cos(orientation.roll()) * sin(orientation.pitch()) * cos(orientation.yaw()) + sin(orientation.yaw()) * sin(orientation.roll());
    rot_mat(2, 1) = cos(orientation.roll()) * sin(orientation.pitch()) * sin(orientation.yaw()) - sin(orientation.roll()) * cos(orientation.yaw());
    rot_mat(2, 2) = cos(orientation.roll()) * cos(orientation.pitch());
    return rot_mat;
}

Orientation3d TF::calc_euler_angles_from_rotation_matrix(Eigen::Matrix3d rot_mat)
{
    Orientation3d orientation;
    orientation.set_pitch(-asin(rot_mat(0, 2)));
    if (cos(orientation.pitch()) != 0)
    {
        orientation.set_yaw(atan2(rot_mat(0, 1) / cos(orientation.pitch()), rot_mat(0, 0) / cos(orientation.pitch())));
        orientation.set_roll(atan2(rot_mat(1, 2) / cos(orientation.pitch()), rot_mat(2, 2) / cos(orientation.pitch())));
    }
    else
    {
        orientation.set_yaw(0);
        orientation.set_roll(atan2(rot_mat(1, 0), rot_mat(1, 1)));
    }
    return orientation;
}

Vector3d TF::transform_to_local_coordinates(const Vector3d& input_coordinates, const Orientation3d& new_origin_rotation, const Vector3d& new_origin_translation)
{
    double delta_x = input_coordinates.x() - new_origin_translation.x();
    double delta_y = input_coordinates.y() - new_origin_translation.y();
    double delta_z = input_coordinates.z() - new_origin_translation.z();

    Eigen::Matrix3d rot_mat = calc_rotation_matrix_from_euler_angles(new_origin_rotation);
    double x_rel = delta_x * rot_mat(0, 0) + delta_y * rot_mat(0, 1) + delta_z * rot_mat(0, 2);
    double y_rel = delta_x * rot_mat(1, 0) + delta_y * rot_mat(1, 1) + delta_z * rot_mat(1, 2);
    double z_rel = delta_x * rot_mat(2, 0) + delta_y * rot_mat(2, 1) + delta_z * rot_mat(2, 2);

    Vector3d coordinates_new_origin;
    coordinates_new_origin.set_x(x_rel);
    coordinates_new_origin.set_y(y_rel);
    coordinates_new_origin.set_z(z_rel);

    return coordinates_new_origin;
}

Vector3d TF::transform_from_local_coordinates(const Vector3d& input_coordinates, const Orientation3d& new_origin_rotation, const Vector3d& new_origin_translation)
{

    Eigen::Matrix3d rot_mat = calc_rotation_matrix_from_euler_angles(new_origin_rotation);
    double x_rel = input_coordinates.x() * rot_mat.transpose()(0, 0) + input_coordinates.y() * rot_mat.transpose()(0, 1) + input_coordinates.z() * rot_mat.transpose()(0, 2);
    double y_rel = input_coordinates.x() * rot_mat.transpose()(1, 0) + input_coordinates.y() * rot_mat.transpose()(1, 1) + input_coordinates.z() * rot_mat.transpose()(1, 2);
    double z_rel = input_coordinates.x() * rot_mat.transpose()(2, 0) + input_coordinates.y() * rot_mat.transpose()(2, 1) + input_coordinates.z() * rot_mat.transpose()(2, 2);

    double delta_x = x_rel + new_origin_translation.x();
    double delta_y = y_rel + new_origin_translation.y();
    double delta_z = z_rel + new_origin_translation.z();

    Vector3d coordinates_new_origin;
    coordinates_new_origin.set_x(delta_x);
    coordinates_new_origin.set_y(delta_y);
    coordinates_new_origin.set_z(delta_z);

    return coordinates_new_origin;
}

Vector3d TF::transform_position_from_ego_to_world_coordinates(const Vector3d& position_in_ego_coordinates, const TF::EgoData& ego_data)
{
    Vector3d position_to_ego_bb_center = vector_translation(position_in_ego_coordinates, ego_data.ego_vehicle_attributes.bbcenter_to_rear(), 1);
    Vector3d position_in_world_coordinates = transform_from_local_coordinates(position_to_ego_bb_center, ego_data.ego_base.orientation(), ego_data.ego_base.position());
    return position_in_world_coordinates;
}

Vector3d TF::transform_position_from_world_to_ego_coordinates(const Vector3d& input_in_world_coordinates, const TF::EgoData& ego_data)
{
    Vector3d position_ego_bb_center = transform_to_local_coordinates(input_in_world_coordinates, ego_data.ego_base.orientation(), ego_data.ego_base.position());
    Vector3d position_ego_coordinates = vector_translation(position_ego_bb_center, ego_data.ego_vehicle_attributes.bbcenter_to_rear(), -1);
    return position_ego_coordinates;
}

Vector3d TF::transform_position_from_world_to_sensor_coordinates(const Vector3d& input_in_world_coordinates, const TF::EgoData& ego_data, const MountingPosition& mounting_pose)
{
    Vector3d position_ego_bb_center = transform_to_local_coordinates(input_in_world_coordinates, ego_data.ego_base.orientation(), ego_data.ego_base.position());
    Vector3d position_ego_coordinates = vector_translation(position_ego_bb_center, ego_data.ego_vehicle_attributes.bbcenter_to_rear(), -1);
    Vector3d position_cartesian_sensor_coordinates = transform_to_local_coordinates(position_ego_coordinates, mounting_pose.orientation(), mounting_pose.position());
    return position_cartesian_sensor_coordinates;
}

Orientation3d TF::transform_orientation_from_world_to_sensor_coordinates(const Orientation3d& orientation_in_world_coordinates,
                                                                         const TF::EgoData& ego_data,
                                                                         const MountingPosition& mounting_pose)
{
    Orientation3d relative_orientation_ego_coordinates = calc_relative_orientation_to_local(orientation_in_world_coordinates, ego_data.ego_base.orientation());
    Orientation3d relative_orientation_sensor_coordinates = calc_relative_orientation_to_local(relative_orientation_ego_coordinates, mounting_pose.orientation());
    return relative_orientation_sensor_coordinates;
}

Orientation3d TF::transform_orientation_from_sensor_to_world_coordinates(const Orientation3d& orientation_in_sensor_coordinates,
                                                                         const TF::EgoData& ego_data,
                                                                         const MountingPosition& mounting_pose)
{
    Orientation3d relative_orientation_ego_coordinates = calc_relative_orientation_from_local(orientation_in_sensor_coordinates, mounting_pose.orientation());
    Orientation3d relative_orientation_world_coordinates = calc_relative_orientation_from_local(relative_orientation_ego_coordinates, ego_data.ego_base.orientation());
    return relative_orientation_world_coordinates;
}

Vector3d TF::transform_position_from_object_to_ego_coordinates(const Vector3d& pcl_segment_position_in_object, const TF::EgoData& ego_data, const MovingObject& gt_object)
{
    Vector3d position_in_world_coordinates = TF::transform_from_local_coordinates(pcl_segment_position_in_object, gt_object.base().orientation(), gt_object.base().position());
    Vector3d position_in_ego_coordinates = TF::transform_position_from_world_to_ego_coordinates(position_in_world_coordinates, ego_data);
    return position_in_ego_coordinates;
}

Vector3d TF::transform_position_from_ego_to_object_coordinates(const Vector3d& pcl_segment_position_in_ego, const TF::EgoData& ego_data, const MovingObject& gt_object)
{
    Vector3d position_in_world_coordinates = TF::transform_position_from_ego_to_world_coordinates(pcl_segment_position_in_ego, ego_data);
    Vector3d position_in_object_coordinates = TF::transform_to_local_coordinates(position_in_world_coordinates, gt_object.base().orientation(), gt_object.base().position());
    return position_in_object_coordinates;
}

Vector3d TF::transform_position_from_sensor_to_object_coordinates(const Vector3d& pcl_segment_position_in_sensor,
                                                                  const TF::EgoData& ego_data,
                                                                  const MountingPosition& mounting_pose,
                                                                  const BaseMoving& object_base)
{
    Vector3d pcl_segment_position_in_ego = TF::transform_from_local_coordinates(pcl_segment_position_in_sensor, mounting_pose.orientation(), mounting_pose.position());
    Vector3d position_in_world_coordinates = TF::transform_position_from_ego_to_world_coordinates(pcl_segment_position_in_ego, ego_data);
    Vector3d position_in_object_coordinates = TF::transform_to_local_coordinates(position_in_world_coordinates, object_base.orientation(), object_base.position());
    return position_in_object_coordinates;
}

Vector3d TF::transform_position_from_object_to_sensor_coordinates(const Vector3d& pcl_segment_position_in_object,
                                                                  const TF::EgoData& ego_data,
                                                                  const MountingPosition& mounting_pose,
                                                                  const BaseMoving& object_base)
{
    Vector3d position_in_world_coordinates = TF::transform_from_local_coordinates(pcl_segment_position_in_object, object_base.orientation(), object_base.position());
    Vector3d position_in_ego_coordinates = TF::transform_position_from_world_to_ego_coordinates(position_in_world_coordinates, ego_data);
    Vector3d position_in_sensor_coordinates = TF::transform_to_local_coordinates(position_in_ego_coordinates, mounting_pose.orientation(), mounting_pose.position());
    return position_in_sensor_coordinates;
}

Vector3d TF::transform_position_from_object_to_world_coordinates(const Vector3d& pcl_segment_position_in_object, const BaseMoving& object_base)
{
    Vector3d position_in_world_coordinates = TF::transform_from_local_coordinates(pcl_segment_position_in_object, object_base.orientation(), object_base.position());
    return position_in_world_coordinates;
}

Spherical3d TF::transform_cartesian_to_spherical(const Vector3d& input_relative_position)
{
    double distance = sqrt(input_relative_position.x() * input_relative_position.x() + input_relative_position.y() * input_relative_position.y() +
                    input_relative_position.z() * input_relative_position.z());
    double azimuth = atan2(input_relative_position.y(), input_relative_position.x());
    double elevation = asin(input_relative_position.z() / distance);
    Spherical3d spherical_coordinate;
    spherical_coordinate.set_distance(distance);
    spherical_coordinate.set_azimuth(azimuth);
    spherical_coordinate.set_elevation(elevation);
    return spherical_coordinate;
}

void TF::transform_spherical_to_cartesian(const Spherical3d& input_spherical_coordinate, Vector3d& vector_xyz)
{
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

void TF::projection_onto_unit_distance_cylinder(const Spherical3d& input_spherical_coordinate, Vector2d& output_position_on_plane)
{
    output_position_on_plane.set_x(input_spherical_coordinate.azimuth());
    output_position_on_plane.set_y(tan(input_spherical_coordinate.elevation()));
}

bool TF::get_ego_info(TF::EgoData& ego_data, const SensorView& input_sensor_view)
{
    if (input_sensor_view.global_ground_truth().has_host_vehicle_id())
    {
        ego_data.ego_vehicle_id = input_sensor_view.global_ground_truth().host_vehicle_id();
        bool ego_vehicle_found = false;
        int mov_obj_no = 0;
        while (!ego_vehicle_found && mov_obj_no < input_sensor_view.global_ground_truth().moving_object_size())
        {
            if (input_sensor_view.global_ground_truth().moving_object(mov_obj_no).id().value() == ego_data.ego_vehicle_id.value())
            {
                ego_data.ego_vehicle_no = mov_obj_no;
                ego_vehicle_found = true;
            }
            mov_obj_no++;
        }
        if (!ego_vehicle_found)
        {
            return false;
        }
        if (input_sensor_view.global_ground_truth().moving_object_size() > ego_data.ego_vehicle_no &&
            input_sensor_view.global_ground_truth().moving_object(ego_data.ego_vehicle_no).has_base())
        {
            ego_data.ego_base.CopyFrom(input_sensor_view.global_ground_truth().moving_object(ego_data.ego_vehicle_no).base());
            ego_data.sensor_mounting_parameters.CopyFrom(input_sensor_view.mounting_position());
            ego_data.ego_vehicle_attributes.CopyFrom(input_sensor_view.global_ground_truth().moving_object(ego_data.ego_vehicle_no).vehicle_attributes());
        }
        else
        {
            return false;
        }
    }
    else
    {
        return false;
    }
    return true;
}

Vector3d TF::vector_translation(const Vector3d& vector_a, const Vector3d& vector_b, double factor_for_vector_b)
{

    Vector3d vector_result;
    vector_result.set_x(vector_a.x() + factor_for_vector_b * vector_b.x());
    vector_result.set_y(vector_a.y() + factor_for_vector_b * vector_b.y());
    vector_result.set_z(vector_a.z() + factor_for_vector_b * vector_b.z());
    return vector_result;
}

double TF::get_vector_abs(const Vector3d& vector)
{
    return sqrt(pow(vector.x(), 2) + pow(vector.y(), 2) + pow(vector.z(), 2));
}

double TF::dot_product(const Vector3d& vector_a, const Vector3d& vector_b)
{
    return vector_a.x() * vector_b.x() + vector_a.y() * vector_b.y() + vector_a.z() * vector_b.z();
}

Vector3d TF::cross_product(const Vector3d& vector_a, const Vector3d& vector_b)
{
    Vector3d vector_result;
    vector_result.set_x(vector_a.y() * vector_b.z() - vector_a.z() * vector_b.y());
    vector_result.set_y(vector_a.z() * vector_b.x() - vector_a.x() * vector_b.z());
    vector_result.set_z(vector_a.x() * vector_b.y() - vector_a.y() * vector_b.x());
    return vector_result;
}
