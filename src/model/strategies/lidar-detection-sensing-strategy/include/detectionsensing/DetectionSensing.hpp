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

#ifndef DETECTION_SENSING_STRATEGY_HPP
#define DETECTION_SENSING_STRATEGY_HPP

#include <model/include/strategy.hpp>

using namespace osi3;

namespace model {

	class DetectionSensing : public Strategy {

    private:

	public:
        //// Main Functions
        using Strategy::Strategy;
        void apply(SensorData &) override;
	};
}

#endif //DETECTION_SENSING_STRATEGY_HPP
