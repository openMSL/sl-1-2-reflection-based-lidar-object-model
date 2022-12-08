//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef ROS_DETECTEDOBJECTS_HPP
#define ROS_DETECTEDOBJECTS_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <model/include/strategy.hpp>
#include <string>
#include <memory>
#include <visualization_msgs/Marker.h>

using namespace model;
using namespace osi3;

namespace detectedobjects {
    class WorkerMarker final {
    public:
        WorkerMarker(const std::string &topic, std::string frame_id);

        void inject(SensorData &in, const Log &log);

    private:
        const std::string frame_id;
        ros::NodeHandle node;
        ros::Publisher publisher;

        static visualization_msgs::Marker set_marker(const Vector3d &position, const Orientation3d &orientation, const Dimension3d &dimension, const uint64_t id, std_msgs::ColorRGBA color, std::string frame, ros::Time time);
        static std_msgs::ColorRGBA set_color(float r, float g, float b, float a);
    };
}

namespace model {
    class ros_detectedobjects : public Strategy {
    public:
        ros_detectedobjects(const Profile &profile, const Log &log, const Alert &alert);
        using Strategy::Strategy;

        void apply(SensorData &) override;

    private:
        std::unique_ptr<detectedobjects::WorkerMarker> worker = nullptr;
    };
}



#endif //ROS_DETECTEDOBJECTS_HPP
