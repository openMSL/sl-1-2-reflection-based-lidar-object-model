//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
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
