//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef ROS_DETECTIONS_HPP
#define ROS_DETECTIONS_HPP

#include <memory>
#include <string>

#include <model/include/strategy.hpp>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

using namespace model;
using namespace osi3;

namespace detections
{
class WorkerPCL final
{
  public:
    WorkerPCL(const std::string& topic, std::string frame_id);
    void inject_lidar(SensorData& sensor_data, int sensor_no, const Log& log);
    void inject_radar(SensorData& sensor_data, int sensor_no, const Log& log);

  private:
    const std::string frame_id;
    ros::NodeHandle node;
    ros::Publisher publisher;
};
}  // namespace detections

namespace model
{
class RosDetections : public Strategy
{
  public:
    RosDetections(const Profile& profile, const Log& log, const Alert& alert);
    using Strategy::Strategy;

    void apply(SensorData& sensor_data) override;

  private:
    std::unique_ptr<detections::WorkerPCL> worker_pcl = nullptr;
};
}  // namespace model

std_msgs::ColorRGBA set_color(float r, float g, float b, float a);

#endif  // ROS_DETECTIONS_HPP
