//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef DETECTION_SENSING_STRATEGY_HPP
#define DETECTION_SENSING_STRATEGY_HPP

#include <model/include/strategy.hpp>
#include <algorithm>

using namespace osi3;

namespace model {

	class DetectionSensing : public Strategy {

    private:
		struct LidarCuboidCell_mW {
			int beam_idx = 0;
			size_t dist_cell_idx = 0;
			float signal_strength_in_mW = 0.0;
		};
		struct LidarBeamCell_mW {
			size_t dist_cell_idx = 0;
			float signal_strength_in_mW = 0.0;
		};
		struct LidarBeamCell_dBm {
			size_t dist_cell_idx = 0;
			float signal_strength_in_dBm = 0.0;
		};
		struct LidarPeak {
			float distance_in_m = 0.0;
			float signal_strength_in_dBm = 0.0;
			float epw_in_m = 0.0;
			float echo_pulse_start_in_m = 0.0;
			size_t echo_idx = 0;
		};

	public:        
        //// Main Functions
        using Strategy::Strategy;
        void apply(SensorData &) override;

		void process_collected_beam_cells(LidarDetectionData *current_sensor, std::vector<LidarBeamCell_mW> *lidar_cuboid_cells_of_beam_ptr,
                                          size_t rays_per_beam, int lidar_frontend_idx, int beam_idx);
		void threshold_summed_beam_cell(LidarBeamCell_mW *summed_dist_cell_of_beam, std::vector<LidarBeamCell_dBm> *thresholded_summed_dist_cells);
	};
}

#endif //DETECTION_SENSING_STRATEGY_HPP
