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

#include "ros_detections/ros_detections.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <utility>

#ifdef _WIN32
#include <math.h>
#else

#include <cmath>

#endif

#define _USE_MATH_DEFINES

using namespace model;
using namespace osi3;

void ros_detections::apply(SensorData &sensor_data) {
    log("Starting ROS output for detections");

    ros::param::set("/use_sim_time", true);

    if ((sensor_data.sensor_view_size() == 0) || (!sensor_data.has_feature_data())) {
        log("No sensor view or feature data received");
        return;
    }

    if (!sensor_data.sensor_view(0).has_global_ground_truth()) {
        log("No global ground truth received");
        return;
    }

    if (sensor_data.feature_data().radar_sensor().size() > 0) {
        for (int sensor_no = 0; sensor_no < sensor_data.feature_data().radar_sensor_size(); sensor_no++) {
            worker_pcl = std::make_unique<detections::WorkerPCL>("detections_" + std::to_string(sensor_no), "sensor_" + std::to_string(sensor_no));
            worker_pcl->injectRadar(sensor_data, sensor_no, log);
        }
    } else if (sensor_data.feature_data().lidar_sensor().size() > 0) {
        for (int sensor_no = 0; sensor_no < sensor_data.feature_data().lidar_sensor_size(); sensor_no++) {
            worker_pcl = std::make_unique<detections::WorkerPCL>("detections_" + std::to_string(sensor_no), "sensor_" + std::to_string(sensor_no));
            worker_pcl->injectLidar(sensor_data, sensor_no, log);
        }
    } else {
        log("No radar or lidar sensor in feature data");
    }
}

ros_detections::ros_detections(const Profile &profile, const Log &log, const Alert &alert) : model::Strategy(profile, log, alert) {
    auto remapping = std::map<std::string, std::string>();
    remapping.emplace("__master", "http://localhost:11311");
    ros::init(remapping, "sensor_model_fmu");
}

detections::WorkerPCL::WorkerPCL(const std::string &topic, std::string frame_id) : publisher(node.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic, 1)), frame_id(std::move(frame_id)) {}

void detections::WorkerPCL::injectLidar(SensorData &sensor_data, int sensor_no, const Log &log) {
    const auto &lidar_sensor = sensor_data.feature_data().lidar_sensor(sensor_no);

    auto sim_seconds = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());
    auto sim_nanos = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
    auto sim_time = ros::Time(sim_seconds, sim_nanos);
    auto sim_time_behind = ros::Time(sim_seconds, sim_nanos + 500);

    auto no_of_detections = lidar_sensor.detection_size();

    if (no_of_detections == 0) {
        auto timestamp = (double) sim_seconds + (double) sim_nanos / 1000000000;
        log("No detections from sensor " + std::to_string(sensor_no) + " for ROS output at timestamp " + std::to_string(timestamp));
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    pcl_conversions::toPCL(sim_time_behind, cloud_tmp->header.stamp);
    cloud_tmp->header.frame_id = frame_id;

    /// Run through all detections

    for (const auto &detection: lidar_sensor.detection()) {
        auto x = float(detection.position().distance() * cos(detection.position().azimuth()) * cos(detection.position().elevation()));
        auto y = float(detection.position().distance() * sin(detection.position().azimuth()) * cos(detection.position().elevation()));
        auto z = float(detection.position().distance() * sin(detection.position().elevation()));
        cloud_tmp->points.emplace_back(pcl::PointXYZ(x, y, z));
    }
    pcl::copyPointCloud(*cloud_tmp, *cloud);
    /// Add intensity to all detections
    for (int detection_idx = 0; detection_idx < lidar_sensor.detection_size(); detection_idx++) {
        cloud->points[detection_idx].intensity = float(lidar_sensor.detection(detection_idx).intensity());
    }
    publisher.publish(cloud);
}


void detections::WorkerPCL::injectRadar(SensorData &sensor_data, int sensor_no, const Log &log) {
    const auto &radar_sensor = sensor_data.feature_data().radar_sensor(sensor_no);

    auto sim_seconds = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());
    auto sim_nanos = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
    auto sim_time = ros::Time(sim_seconds, sim_nanos);
    auto sim_time_behind = ros::Time(sim_seconds, sim_nanos + 500);

    auto no_of_detections = radar_sensor.detection_size();

    if (no_of_detections == 0) {
        auto timestamp = (double) sim_seconds + (double) sim_nanos / 1000000000;
        log("No detections from sensor " + std::to_string(sensor_no) + " for ROS output at timestamp " + std::to_string(timestamp));
        return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());

    pcl_conversions::toPCL(sim_time_behind, cloud_tmp->header.stamp);
    cloud_tmp->header.frame_id = frame_id;

    /// Run through all detections

    for (const auto &detection: radar_sensor.detection()) {
        auto x = float(detection.position().distance() * cos(detection.position().azimuth()) * cos(detection.position().elevation()));
        auto y = float(detection.position().distance() * sin(detection.position().azimuth()) * cos(detection.position().elevation()));
        auto z = float(detection.position().distance() * sin(detection.position().elevation()));
        cloud_tmp->points.emplace_back(pcl::PointXYZ(x, y, z));
    }
    pcl::copyPointCloud(*cloud_tmp, *cloud);
    /// Add intensity to all detections
    for (int detection_idx = 0; detection_idx < radar_sensor.detection_size(); detection_idx++) {
        cloud->points[detection_idx].intensity = float(radar_sensor.detection(detection_idx).rcs());
    }
    publisher.publish(cloud);
}