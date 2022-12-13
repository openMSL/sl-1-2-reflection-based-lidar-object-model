//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef BIN_OUTPUT_SENSORDATA_STRATEGY_HPP
#define BIN_OUTPUT_SENSORDATA_STRATEGY_HPP

#include <model/include/strategy.hpp>
#include <string>

using namespace osi3;

namespace model {

	class OSIBinSensorDataOutput : public Strategy {

        using Strategy::Strategy;

        void apply(SensorData &) override;

        std::string file_path_tracefile;
        bool first_call = true;
    public:

    private:
        static void write_data_to_bin(const std::string& path, const SensorData &sensor_data);
    };

}

#endif //BIN_OUTPUT_SENSORDATA_STRATEGY_HPP
