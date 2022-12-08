//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#include "ros_logicaldetections/ros_logicaldetections.hpp"
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <string>
#include <utility>

#define _USE_MATH_DEFINES

using namespace model;
using namespace osi3;

void ros_logicaldetections::apply(SensorData &sensor_data) {
    log("Starting ROS output for logical detections");

    ros::param::set("/use_sim_time", true);

    if ((sensor_data.sensor_view_size()==0) || (!sensor_data.has_logical_detection_data())) {
        log("No sensor view or feature data received");
        return;
    }

    if (!sensor_data.sensor_view(0).has_global_ground_truth()) {
        log("No global ground truth received");
        return;
    }

    if (sensor_data.logical_detection_data().logical_detection().size() == 0) {
        log("No logical detections for ros output");
        return;
    }

    worker_pcl->inject(sensor_data, log);
}

ros_logicaldetections::ros_logicaldetections(const Profile &profile, const Log &log, const Alert &alert): model::Strategy(profile, log, alert) {

	auto remapping = std::map<std::string, std::string>();
	remapping.emplace("__master", "http://localhost:11311");
	ros::init(remapping, "sensor_model_fmu");
    worker_pcl = std::make_unique<logicaldetections::WorkerPCL>("logical_detections", "base_link");
}

logicaldetections::WorkerPCL::WorkerPCL(const std::string& topic, std::string frame_id) : publisher(node.advertise<pcl::PointCloud<pcl::PointXYZ>>(topic, 1)) , frame_id(std::move(frame_id)) {}

void logicaldetections::WorkerPCL::inject(SensorData &sensor_data, const Log &log) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_tmp(new pcl::PointCloud<pcl::PointXYZ>());
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
//        auto time = ros::Time::now();
        auto sim_seconds = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds());
        auto sim_nanos = uint32_t(sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos());
        auto sim_time = ros::Time(sim_seconds, sim_nanos);
        auto sim_time_behind = ros::Time(sim_seconds, sim_nanos + 500);
        pcl_conversions::toPCL(sim_time_behind, cloud_tmp->header.stamp);
        cloud_tmp->header.frame_id = frame_id;

        /// Run through all logical detections
        for (const auto& logical_detection : sensor_data.logical_detection_data().logical_detection()) {
            auto x = float(logical_detection.position().x());
            auto y = float(logical_detection.position().y());
            auto z = float(logical_detection.position().z());
            cloud_tmp->points.emplace_back(pcl::PointXYZ(x, y, z));
        }
        pcl::copyPointCloud(*cloud_tmp, *cloud);
        // add intensity to all detections
        for (int logical_detection_idx = 0; logical_detection_idx < sensor_data.logical_detection_data().logical_detection().size(); logical_detection_idx++) {
            cloud->points[logical_detection_idx].intensity = float(sensor_data.logical_detection_data().logical_detection(logical_detection_idx).intensity());
        }
        publisher.publish(cloud);
}
