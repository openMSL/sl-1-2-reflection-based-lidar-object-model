//
// Copyright Institute of Automotive Engineering
// of Technical University of Darmstadt 2021.
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

#include "ros_gt_objects/ros_gt_objects.hpp"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/String.h"
#include <string>
#include <utility>

using namespace model;
using namespace osi3;

void ros_gt_objects::apply(SensorData &sensor_data) {
    log("Starting ROS output for GT objects");

    ros::param::set("/use_sim_time", true);

    if (sensor_data.sensor_view_size() == 0) {
        log("No sensor view received");
        return;
    }

    if (!sensor_data.sensor_view(0).has_global_ground_truth()) {
        log("No global ground truth received");
        return;
    }

    auto time_nanos = sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos();
    auto time_seconds = sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds();
    double timestamp = (double)time_seconds + (double) time_nanos / 1000000000;

    if ((sensor_data.sensor_view(0).global_ground_truth().moving_object_size() == 0) &&
        (sensor_data.sensor_view(0).global_ground_truth().stationary_object_size() == 0)) {
        log("No objects in GT for ROS output at timestamp " + std::to_string(timestamp));
        return;
    }

    worker_gt->inject(sensor_data, profile, log, alert);
}

ros_gt_objects::ros_gt_objects(const Profile &profile, const Log &log, const Alert &alert) : model::Strategy(profile,
                                                                                                             log,
                                                                                                             alert) {
    auto remapping = std::map<std::string, std::string>();
    remapping.emplace("__master", "http://localhost:11311");
    ros::init(remapping, "sensor_model_fmu");
    worker_gt = std::make_unique<ground_truth::WorkerMarker>("ground_truth", "world");
}

/// Objects
ground_truth::WorkerMarker::WorkerMarker(const std::string &topic, std::string frame_id) : publisher(
        node.advertise<visualization_msgs::MarkerArray>(topic, 1)), frame_id(std::move(frame_id)) {}

void ground_truth::WorkerMarker::inject(SensorData &sensor_data, const Profile &profile, const Log &log,
                                        const Alert &alert) {
//    auto time = ros::Time::now();
    auto sim_seconds = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());
    auto sim_nanos = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
    auto sim_time = ros::Time(sim_seconds, sim_nanos);
    auto sim_time_behind = ros::Time(sim_seconds, sim_nanos + 500);

    /// Global Ground Truth
    visualization_msgs::MarkerArray marker_array;
    const MovingObject *ego_vehicle;

    /// Moving objects
    std_msgs::ColorRGBA color_gt_moving;
    for (const MovingObject &gt_moving_object: sensor_data.sensor_view(0).global_ground_truth().moving_object()) {
        if (gt_moving_object.id().value() ==
            sensor_data.sensor_view(0).global_ground_truth().host_vehicle_id().value()) {
            ego_vehicle = &gt_moving_object;
            color_gt_moving = set_color(1.0, 0.0, 0.0, 0.3);
        } else {
            color_gt_moving = set_color(1.0, 1.0, 1.0, 0.3);
        }
        visualization_msgs::Marker marker = set_marker(gt_moving_object.base().position(),
                                                       gt_moving_object.base().orientation(),
                                                       gt_moving_object.base().dimension(),
                                                       gt_moving_object.id().value(),
                                                       color_gt_moving,
                                                       frame_id, sim_time_behind);
        marker_array.markers.push_back(marker);
    }
    /// Stationary objects
    std_msgs::ColorRGBA color_gt_stationary = set_color(1.0, 1.0, 1.0, 0.1);
    for (const StationaryObject &gt_stationary_object: sensor_data.sensor_view(
            0).global_ground_truth().stationary_object()) {
        visualization_msgs::Marker marker = set_marker(gt_stationary_object.base().position(),
                                                       gt_stationary_object.base().orientation(),
                                                       gt_stationary_object.base().dimension(),
                                                       gt_stationary_object.id().value(),
                                                       color_gt_stationary,
                                                       frame_id, sim_time_behind);
        marker_array.markers.push_back(marker);
    }

    /// Transformations
    tf::Transform transform;
    tf::Quaternion q;
    /// Transform Bounding Box Center to world
    if (ego_vehicle->has_base()) {
        transform.setOrigin(tf::Vector3(ego_vehicle->base().position().x(), ego_vehicle->base().position().y(),
                                        ego_vehicle->base().position().z()));
        q.setRPY(ego_vehicle->base().orientation().roll(), ego_vehicle->base().orientation().pitch(),
                 ego_vehicle->base().orientation().yaw());
        transform.setRotation(q);
        transform_broadcaster.sendTransform(tf::StampedTransform(transform, sim_time, "world", "bb_center"));
    } else {
        log("Ego vehicle has no base!");
    }

    /// Transform Bounding Box Center to center of rear axle
    if (ego_vehicle->has_vehicle_attributes()) {
        transform.setOrigin(tf::Vector3(ego_vehicle->vehicle_attributes().bbcenter_to_rear().x(),
                                        ego_vehicle->vehicle_attributes().bbcenter_to_rear().y(),
                                        ego_vehicle->vehicle_attributes().bbcenter_to_rear().z()));
        q.setRPY(0, 0, 0);
        transform.setRotation(q);
        transform_broadcaster.sendTransform(tf::StampedTransform(transform, sim_time, "bb_center", "base_link"));
    } else {
        log("Ego vehicle has no vehicle_attributes!");
    }

    for (auto &sensor_view: sensor_data.sensor_view()) {
        /// Transform center of rear axle to sensor('s) origin
        size_t radar_sensor_no = 0;
        for (auto &radar_sensor_view: sensor_view.radar_sensor_view()) {
            transform.setOrigin(tf::Vector3(radar_sensor_view.view_configuration().mounting_position().position().x(),
                                            radar_sensor_view.view_configuration().mounting_position().position().y(),
                                            radar_sensor_view.view_configuration().mounting_position().position().z()));
            q.setRPY(radar_sensor_view.view_configuration().mounting_position().orientation().roll(),
                     radar_sensor_view.view_configuration().mounting_position().orientation().pitch(),
                     radar_sensor_view.view_configuration().mounting_position().orientation().yaw());
            transform.setRotation(q);
            transform_broadcaster.sendTransform(
                    tf::StampedTransform(transform, sim_time, "base_link", "sensor_" + std::to_string(radar_sensor_no)));
            radar_sensor_no++;
        }
        size_t lidar_sensor_no = 0;
        for (auto &lidar_sensor_view: sensor_view.lidar_sensor_view()) {
            transform.setOrigin(tf::Vector3(lidar_sensor_view.view_configuration().mounting_position().position().x(),
                                            lidar_sensor_view.view_configuration().mounting_position().position().y(),
                                            lidar_sensor_view.view_configuration().mounting_position().position().z()));
            q.setRPY(lidar_sensor_view.view_configuration().mounting_position().orientation().roll(),
                     lidar_sensor_view.view_configuration().mounting_position().orientation().pitch(),
                     lidar_sensor_view.view_configuration().mounting_position().orientation().yaw());
            transform.setRotation(q);
            transform_broadcaster.sendTransform(
                    tf::StampedTransform(transform, sim_time, "base_link", "sensor_" + std::to_string(lidar_sensor_no)));
            lidar_sensor_no++;
        }

        publisher.publish(marker_array);
    }
}

visualization_msgs::Marker
ground_truth::WorkerMarker::set_marker(const Vector3d &position, const Orientation3d &orientation,
                                       const Dimension3d &dimension, const size_t id, std_msgs::ColorRGBA color,
                                       std::string frame, ros::Time time) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = std::move(frame);
    marker.header.stamp = time;
    marker.id = (int)id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = position.x();
    marker.pose.position.y = position.y();
    marker.pose.position.z = position.z();
    tf::Quaternion orientation_q;
    orientation_q.setRPY(orientation.roll(),
                         orientation.pitch(),
                         orientation.yaw());
    marker.pose.orientation.x = orientation_q.x();
    marker.pose.orientation.y = orientation_q.y();
    marker.pose.orientation.z = orientation_q.z();
    marker.pose.orientation.w = orientation_q.w();
    marker.scale.x = dimension.length();
    marker.scale.y = dimension.width();
    marker.scale.z = dimension.height();
    marker.color = color;
    marker.lifetime = ros::Duration(1);
    marker.frame_locked = false;
    return marker;
}

std_msgs::ColorRGBA ground_truth::WorkerMarker::set_color(float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}