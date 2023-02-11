//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef PCD_OUTPUT_DETECTIOONS_STRATEGY_HPP
#define PCD_OUTPUT_DETECTIOONS_STRATEGY_HPP

#include <string>

#include <model/include/strategy.hpp>

using namespace osi3;

namespace model
{

class PcdOutputDetections : public Strategy
{

    using Strategy::Strategy;

    void apply(SensorData& sensor_data) override;

    std::string path_string;
    bool first_call = true;

  public:
  private:
    static void write_pcd_header(const std::string& path, const SensorData& sensor_data, const size_t& no_of_sensors);
    static void write_2_pcd(const std::string& path, double x, double y, double z, double intensity);
};

}  // namespace model

#endif  // PCD_OUTPUT_DETECTIOONS_STRATEGY_HPP
