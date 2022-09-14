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

#ifndef ROS_GT_OBJECTS_HPP
#define ROS_GT_OBJECTS_HPP

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <model/include/strategy.hpp>
#include <string>
#include <memory>
#include <visualization_msgs/Marker.h>

using namespace model;
using namespace osi3;

namespace ground_truth {
    class WorkerMarker final {
    public:
        WorkerMarker(const std::string &topic, std::string frame_id);

        void inject(SensorData &sensor_data, const Profile &profile, const Log &log, const Alert &alert);

    private:
        const std::string frame_id;
        ros::NodeHandle node;
        ros::Publisher publisher;
        tf::TransformBroadcaster transform_broadcaster;

        static visualization_msgs::Marker set_marker(const Vector3d &position, const Orientation3d &orientation, const Dimension3d &dimension, const uint64_t id, std_msgs::ColorRGBA color, std::string frame, ros::Time time);
        static std_msgs::ColorRGBA set_color(float r, float g, float b, float a);
    };
}

namespace model {
    class ros_gt_objects : public Strategy {
    public:
        ros_gt_objects(const Profile &profile, const Log &log, const Alert &alert);
        using Strategy::Strategy;

        void apply(SensorData &) override;

    private:
        std::unique_ptr<ground_truth::WorkerMarker> worker_gt = nullptr;
    };
}



#endif //ROS_GT_OBJECTS_HPP
