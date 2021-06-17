//
// Copyright Philipp Rosenberger, M. Sc.
// Institute of Automotive Engineering
// of Technical University of Darmstadt, 2021.
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
        static uint64_t calculate_segment_of_point_cloud(SensorData &in, uint64_t object_no_in, const std::vector<Vector3d> &bounding_box_corners, const TransformationFunctions::EgoData &ego_data, const Profile &profile, const Log &log);

    };

}

#endif //SEGMENTATION_STRATEGY_HPP
