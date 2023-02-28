//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef ROS_GT_OBJECTS_HPP
#define ROS_GT_OBJECTS_HPP

#include <memory>
#include <string>

#include <model/include/strategy.hpp>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <visualization_msgs/Marker.h>

using namespace model;
using namespace osi3;

namespace ground_truth
{
class WorkerMarker final
{
  public:
    WorkerMarker(const std::string& topic, std::string frame_id);

    void inject(SensorData& sensor_data, const Profile& profile, const Log& log, const Alert& alert);

  private:
    const std::string frame_id;
    ros::NodeHandle node;
    ros::Publisher publisher;
    tf::TransformBroadcaster transform_broadcaster;

    static visualization_msgs::Marker set_marker(const Vector3d& position,
                                                 const Orientation3d& orientation,
                                                 const Dimension3d& dimension,
                                                 const uint64_t id,
                                                 std_msgs::ColorRGBA color,
                                                 std::string frame,
                                                 ros::Time time);
    static std_msgs::ColorRGBA set_color(float r, float g, float b, float a);
};
}  // namespace ground_truth

namespace model
{
class RosGtObjects : public Strategy
{
  public:
    RosGtObjects(const Profile& profile, const Log& log, const Alert& alert);
    using Strategy::Strategy;

    void apply(SensorData& sensor_data) override;

  private:
    std::unique_ptr<ground_truth::WorkerMarker> worker_gt = nullptr;
};
}  // namespace model

#endif  // ROS_GT_OBJECTS_HPP
