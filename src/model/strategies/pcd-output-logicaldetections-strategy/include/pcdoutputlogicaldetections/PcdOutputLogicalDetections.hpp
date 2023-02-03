//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef PCD_OUTPUT_LOGICALDETECTIOONS_STRATEGY_HPP
#define PCD_OUTPUT_LOGICALDETECTIOONS_STRATEGY_HPP

#include <string>

#include <model/include/strategy.hpp>

using namespace osi3;

namespace model
{

class PcdOutputLogicalDetections : public Strategy
{

    using Strategy::Strategy;

    void apply(SensorData&) override;

    std::string path_string = "";
    bool first_call = true;

  public:
  private:
    static void write_pcd_header(const std::string& path, const SensorData& sensor_data);
    static void write_2_pcd(const std::string& path, float x, float y, float z, float intensity);
};

}  // namespace model

#endif  // PCD_OUTPUT_LOGICALDETECTIOONS_STRATEGY_HPP
