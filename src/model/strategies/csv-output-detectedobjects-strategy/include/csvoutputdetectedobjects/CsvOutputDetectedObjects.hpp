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

#ifndef CSV_OUTPUT_DETECTEDOBJECTS_STRATEGY_HPP
#define CSV_OUTPUT_DETECTEDOBJECTS_STRATEGY_HPP

#include <model/include/strategy.hpp>
#include <string>

using namespace osi3;

namespace model {

	class CsvOutputDetectedObjects : public Strategy {

        using Strategy::Strategy;

        void apply(SensorData &) override;

        std::string file_path_detectedobjects;
        bool first_call = true;

    public:

    private:
        static void write_first_line_to_CSV(const std::string& path);
        static void write_data_to_CSV(const std::string &path, double timestamp, size_t tracking_id, double x, double y, double z, double roll, double pitch, double yaw, double width, double length,
                                      double height, bool is_moving, double existence_probability);
    };

}

#endif //CSV_OUTPUT_DETECTEDOBJECTS_STRATEGY_HPP
