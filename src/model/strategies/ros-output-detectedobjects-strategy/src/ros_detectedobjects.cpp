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
#include "ros_detectedobjects/ros_detectedobjects.hpp"
#include <visualization_msgs/MarkerArray.h>
#include <string>
#include <utility>

using namespace model;
using namespace osi3;

void ros_detectedobjects::apply(SensorData &sensor_data) {
    log("Starting ROS output for detected objects");

    ros::param::set("/use_sim_time", true);

    if ((sensor_data.sensor_view_size()==0) || (!sensor_data.has_feature_data())) {
        log("No sensor view or feature data received");
        return;
    }

    if (!sensor_data.sensor_view(0).has_global_ground_truth()) {
        log("No global ground truth received");
        return;
    }

    const auto &ground_truth = sensor_data.sensor_view(0).global_ground_truth();

    auto time_nanos = ground_truth.timestamp().nanos();
    auto time_seconds = ground_truth.timestamp().seconds();
    double timestamp = (double) time_seconds + (double) time_nanos / 1000000000;

    if (sensor_data.moving_object_size() == 0 && sensor_data.stationary_object_size() == 0) {
        log("No detected objects for ROS output at timestamp " + std::to_string(timestamp));
    }

    worker->inject(sensor_data, log);
}

ros_detectedobjects::ros_detectedobjects(const Profile &profile, const Log &log, const Alert &alert): model::Strategy(profile, log, alert) {

	auto remapping = std::map<std::string, std::string>();
	remapping.emplace("__master", "http://localhost:11311");
	ros::init(remapping, "sensor_model_fmu");
    worker = std::make_unique<detectedobjects::WorkerMarker>("tracker", "base_link");
}

//Objects
detectedobjects::WorkerMarker::WorkerMarker(const std::string& topic, std::string frame_id) : publisher(node.advertise<visualization_msgs::MarkerArray>(topic, 1)), frame_id(std::move(frame_id)) {}
void detectedobjects::WorkerMarker::inject(SensorData &sensor_data, const Log &log) {
//    auto time = ros::Time::now();


    auto sim_seconds = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());
    auto sim_nanos = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
    auto sim_time = ros::Time(sim_seconds, sim_nanos);
    auto sim_time_behind = ros::Time(sim_seconds, sim_nanos + 500);

    //Detected Objects
    visualization_msgs::MarkerArray marker_array_tracker;
    std_msgs::ColorRGBA marker_color;
    if (sensor_data.moving_object_size() > 0) {
        for (const auto &moving_object: sensor_data.moving_object()) {
            marker_color = set_color(0.0, 0.0, 1.0, float(0.6 * moving_object.header().existence_probability()));
            visualization_msgs::Marker marker = set_marker(moving_object.base().position(),
                                                           moving_object.base().orientation(),
                                                           moving_object.base().dimension(),
                                                           moving_object.header().tracking_id().value(),
                                                           marker_color,
                                                           frame_id, sim_time_behind);
            marker_array_tracker.markers.push_back(marker);
        }
    }

    if (sensor_data.stationary_object_size() > 0) {
        for (const auto &stationary_object: sensor_data.stationary_object()) {
            marker_color = set_color(0.0, 0.0, 1.0, float(0.5 * stationary_object.header().existence_probability()));
            visualization_msgs::Marker marker = set_marker(stationary_object.base().position(),
                                                           stationary_object.base().orientation(),
                                                           stationary_object.base().dimension(),
                                                           stationary_object.header().tracking_id().value(),
                                                           marker_color,
                                                           frame_id, sim_time_behind);
            marker_array_tracker.markers.push_back(marker);
        }
    }

    publisher.publish(marker_array_tracker);
}


visualization_msgs::Marker detectedobjects::WorkerMarker::set_marker(const Vector3d &position, const Orientation3d &orientation, const Dimension3d &dimension, const uint64_t id, std_msgs::ColorRGBA color, std::string frame, ros::Time time) {
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
    marker.lifetime = ros::Duration(0.7);
    marker.frame_locked = false;
    return marker;
}

std_msgs::ColorRGBA detectedobjects::WorkerMarker::set_color(float r, float g, float b, float a) {
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}