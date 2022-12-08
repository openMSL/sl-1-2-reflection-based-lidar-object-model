//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
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
