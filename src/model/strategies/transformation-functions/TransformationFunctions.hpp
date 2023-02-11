//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef TRANSFORMATIONFUNCTIONS_H
#define TRANSFORMATIONFUNCTIONS_H

#include <model/include/strategy.hpp>

#include "osi_sensordata.pb.h"

#include <Dense>

using namespace osi3;

namespace model
{

class TF
{
  public:
    struct EgoData
    {
        BaseMoving ego_base;
        MountingPosition sensor_mounting_parameters;
        Identifier ego_vehicle_id;
        MovingObject::VehicleAttributes ego_vehicle_attributes;
        int ego_vehicle_no = 0;
    };

    static Orientation3d calc_relative_orientation_to_local(const Orientation3d& object_orientation, const Orientation3d& ego_orientation);
    static Orientation3d calc_relative_orientation_from_local(const Orientation3d& object_orientation, const Orientation3d& ego_orientation);

    static Eigen::Matrix3d calc_rotation_matrix_from_euler_angles(const Orientation3d& orientation);
    static Orientation3d calc_euler_angles_from_rotation_matrix(Eigen::Matrix3d rot_mat);

    static Vector3d transform_to_local_coordinates(const Vector3d& input_coordinates, const Orientation3d& new_origin_rotation, const Vector3d& new_origin_translation);
    static Vector3d transform_from_local_coordinates(const Vector3d& input_coordinates, const Orientation3d& new_origin_rotation, const Vector3d& new_origin_translation);

    static Vector3d transform_position_from_world_to_ego_coordinates(const Vector3d& input_in_world_coordinates, const TF::EgoData& ego_data);
    static Vector3d transform_position_from_ego_to_world_coordinates(const Vector3d& position_in_ego_coordinates, const TF::EgoData& ego_data);

    static Vector3d transform_position_from_world_to_sensor_coordinates(const Vector3d& input_in_world_coordinates,
                                                                        const TF::EgoData& ego_data,
                                                                        const MountingPosition& mounting_pose);

    static Vector3d transform_position_from_object_to_ego_coordinates(const Vector3d& pcl_segment_position_in_object, const TF::EgoData& ego_data, const MovingObject& gt_object);
    static Vector3d transform_position_from_ego_to_object_coordinates(const Vector3d& pcl_segment_position_in_ego, const EgoData& ego_data, const MovingObject& gt_object);
    static Vector3d transform_position_from_sensor_to_object_coordinates(const Vector3d& pcl_segment_position_in_sensor,
                                                                         const EgoData& ego_data,
                                                                         const MountingPosition& mounting_pose,
                                                                         const BaseMoving& object_base);
    static Vector3d transform_position_from_object_to_sensor_coordinates(const Vector3d& pcl_segment_position_in_object,
                                                                         const TF::EgoData& ego_data,
                                                                         const MountingPosition& mounting_pose,
                                                                         const BaseMoving& object_base);
    static Vector3d transform_position_from_object_to_world_coordinates(const Vector3d& pcl_segment_position_in_object, const BaseMoving& object_base);

    static Orientation3d transform_orientation_from_world_to_sensor_coordinates(const Orientation3d& orientation_in_world_coordinates,
                                                                                const TF::EgoData& ego_data,
                                                                                const MountingPosition& mounting_pose);
    static Orientation3d transform_orientation_from_sensor_to_world_coordinates(const Orientation3d& orientation_in_sensor_coordinates,
                                                                                const EgoData& ego_data,
                                                                                const MountingPosition& mounting_pose);

    static Spherical3d transform_cartesian_to_spherical(const Vector3d& input_relative_position);
    static void transform_spherical_to_cartesian(const Spherical3d& input_spherical_coordinate, Vector3d& vector_xyz);

    static double projection_to_spherical_surface(std::vector<Spherical3d>& spherical_coordinates, double distance);
    static void projection_onto_unit_distance_cylinder(const Spherical3d& input_spherical_coordinate, Vector2d& output_position_on_plane);

    static bool get_ego_info(TF::EgoData& ego_data, const SensorView& input_sensor_view);

    static Vector3d vector_translation(const Vector3d& vector_a, const Vector3d& vector_b, double factor_for_vector_b);
    static double get_vector_abs(const Vector3d& vector);
    static double dot_product(const Vector3d& vector_a, const Vector3d& vector_b);
    static Vector3d cross_product(const Vector3d& vector_a, const Vector3d& vector_b);
};
}  // namespace model

#endif  // TRANSFORMATIONFUNCTIONS_H
