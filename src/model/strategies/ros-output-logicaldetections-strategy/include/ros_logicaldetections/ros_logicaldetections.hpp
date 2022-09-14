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
