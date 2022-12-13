//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef ROS_LOGICALDETECTIONS_HPP
#define ROS_LOGICALDETECTIONS_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <model/include/strategy.hpp>
#include <string>
#include <memory>
#include <visualization_msgs/Marker.h>

using namespace model;
using namespace osi3;

namespace logicaldetections {
    class WorkerPCL final {
    public:
        WorkerPCL(const std::string &topic, std::string frame_id);
        void inject(SensorData &sensor_data, const Log &log);

    private:
        const std::string frame_id;
        ros::NodeHandle node;
        ros::Publisher publisher;
        tf::TransformListener listener;
        tf::TransformBroadcaster transform_broadcaster;
    };
}


namespace model {
    class ros_logicaldetections : public Strategy {
    public:
        ros_logicaldetections(const Profile &profile, const Log &log, const Alert &alert);
        using Strategy::Strategy;

        void apply(SensorData &) override;

    private:
        std::unique_ptr<logicaldetections::WorkerPCL> worker_pcl = nullptr;
    };
}

std_msgs::ColorRGBA set_color(float r, float g, float b, float a);

#endif //ROS_LOGICALDETECTIONS_HPP
