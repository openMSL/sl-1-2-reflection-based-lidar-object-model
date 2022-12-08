//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef SEGMENTATION_STRATEGY_HPP
#define SEGMENTATION_STRATEGY_HPP

#include <model/include/strategy.hpp>
#include "../../transformation-functions/TransformationFunctions.hpp"
#include <vector>

using namespace osi3;

namespace model {

	class Segmentation : public Strategy {

        using Strategy::Strategy;
		void apply(SensorData &) override;

	public:

	private:

        static std::vector<Vector3d> get_bounding_box_corners(const MovingObject &current_moving_object);
        static size_t calculate_segment_of_point_cloud(SensorData &in, int object_no_in, const std::vector<Vector3d> &bounding_box_corners, const TF::EgoData &ego_data, const Profile &profile, const Log &log);

    };

}

#endif //SEGMENTATION_STRATEGY_HPP
