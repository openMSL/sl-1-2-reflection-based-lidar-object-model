//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef PCD_OUTPUT_DETECTIOONS_STRATEGY_HPP
#define PCD_OUTPUT_DETECTIOONS_STRATEGY_HPP

#include <model/include/strategy.hpp>
#include <string>

using namespace osi3;

namespace model {

	class PcdOutputDetections : public Strategy {

        using Strategy::Strategy;

        void apply(SensorData &) override;

        std::string path_string;
        bool first_call = true;

    public:

    private:
	    static void writePcdHeader(const std::string& path, const SensorData& sensor_data, const size_t& no_of_sensors);
        static void write2Pcd(const std::string& path, double x, double y, double z, double intensity);
    };

}

#endif //PCD_OUTPUT_DETECTIOONS_STRATEGY_HPP
