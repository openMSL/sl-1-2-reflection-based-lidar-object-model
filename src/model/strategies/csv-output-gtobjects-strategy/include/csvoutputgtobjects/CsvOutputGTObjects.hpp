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

#ifndef CSV_OUTPUT_GTOBJECTS_STRATEGY_HPP
#define CSV_OUTPUT_GTOBJECTS_STRATEGY_HPP

#include <model/include/strategy.hpp>
#include <string>

using namespace osi3;

namespace model {

	class CsvOutputGTObjects : public Strategy {

        using Strategy::Strategy;

        void apply(SensorData &) override;

        std::string file_path_gtobjects;
        bool first_call = true;

    public:

    private:
        
		struct GT_object {
			size_t id = 0;
            float x = 0.0;
            float y = 0.0;
            float z = 0.0;
            float roll = 0.0;
            float pitch = 0.0;
            float yaw = 0.0;
            float width = 0.0;
            float length = 0.0;
            float height = 0.0;
            bool is_moving = false;
		};

        static void write_first_line_to_CSV(const std::string& path);
        static void write_data_to_CSV(const std::string& path, double timestamp, size_t object_idx, float x, float y, float z, float roll, float pitch, float yaw, float width, float length, float height, bool is_moving);
    };

}

#endif //CSV_OUTPUT_GTOBJECTS_STRATEGY_HPP
