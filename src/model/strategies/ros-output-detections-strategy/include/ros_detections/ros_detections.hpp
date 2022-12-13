//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef ROS_DETECTIONS_HPP
#define ROS_DETECTIONS_HPP

#include <ros/ros.h>
#include <model/include/strategy.hpp>
#include <string>
#include <memory>
#include <visualization_msgs/Marker.h>

using namespace model;
using namespace osi3;

namespace detections {
    class WorkerPCL final {
    public:
        WorkerPCL(const std::string &topic, std::string frame_id);
        void injectLidar(SensorData &sensor_data, int sensor_no, const Log &log);
        void injectRadar(SensorData &sensor_data, int sensor_no, const Log &log);

    private:
        const std::string frame_id;
        ros::NodeHandle node;
        ros::Publisher publisher;
    };
}


namespace model {
    class ros_detections : public Strategy {
    public:
        ros_detections(const Profile &profile, const Log &log, const Alert &alert);
        using Strategy::Strategy;

        void apply(SensorData &) override;

    private:
        std::unique_ptr<detections::WorkerPCL> worker_pcl = nullptr;
    };
}

std_msgs::ColorRGBA set_color(float r, float g, float b, float a);

#endif //ROS_DETECTIONS_HPP
