//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef DETECTION_SENSING_STRATEGY_HPP
#define DETECTION_SENSING_STRATEGY_HPP

#include <algorithm>

#include <model/include/strategy.hpp>

using namespace osi3;

namespace model
{

class DetectionSensing : public Strategy
{

  private:
    struct LidarCuboidCellmW
    {
        int beam_idx = 0;
        size_t dist_cell_idx = 0;
        float signal_strength_in_mW = 0.0;
    };
    struct LidarBeamCellmW
    {
        size_t dist_cell_idx = 0;
        float signal_strength_in_mW = 0.0;
    };
    struct LidarBeamCelldBm
    {
        size_t dist_cell_idx = 0;
        float signal_strength_in_dBm = 0.0;
    };
    struct LidarPeak
    {
        float distance_in_m = 0.0;
        float signal_strength_in_dBm = 0.0;
        float epw_in_m = 0.0;
        float echo_pulse_start_in_m = 0.0;
        size_t echo_idx = 0;
    };

  public:
    //// Main Functions
    using Strategy::Strategy;
    void apply(SensorData&) override;

    void process_collected_beam_cells(LidarDetectionData* current_sensor, std::vector<LidarBeamCellmW>* lidar_cuboid_cells_of_beam_ptr, size_t rays_per_beam, int lidar_frontend_idx, int beam_idx);
    void threshold_summed_beam_cell(LidarBeamCellmW* summed_dist_cell_of_beam, std::vector<LidarBeamCelldBm>* thresholded_summed_dist_cells);
};
}  // namespace model

#endif  // DETECTION_SENSING_STRATEGY_HPP
