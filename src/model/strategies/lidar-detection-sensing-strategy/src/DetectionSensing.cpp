//
// Copyright Philipp Rosenberger, M. Sc.
// Institute of Automotive Engineering
// of Technical University of Darmstadt, 2021.
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

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "detectionsensing/DetectionSensing.hpp"
#include <random>
#include <string>
#include <vector>
#include <iostream>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>

#endif

using namespace model;
using namespace osi3;

void DetectionSensing::apply(SensorData &sensor_data) {
    log("Running 'Reflection Based Lidar Model by FZD'");
	log("Starting detection sensing");

    if((sensor_data.sensor_view_size() == 0)) {
        alert("No SensorView given!");
        return;
    }

    /// Get time stamp from ground truth
    auto timestamp = (float) sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds() + (float) sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos() / 1000000000.0;
    log("GT time stamp: " + std::to_string(timestamp));

	if(timestamp == 0)
		return;

    auto no_of_lidar_frontends = sensor_data.sensor_view(0).lidar_sensor_view_size();
	log("Number of simulated lidar sensor front-ends: " + std::to_string(no_of_lidar_frontends));

	if (profile.sensor_view_configuration.lidar_sensor_view_configuration_size() != no_of_lidar_frontends)
		alert("Number of LidarSensorViews different to profile/SensorViewConfiguration!");

	size_t total_no_of_rays_for_frontend = 0;
	size_t total_no_of_reflections = 0;
	int lidar_sensor_view_idx = 0;
	for (auto & lidar_sensor_view : sensor_data.sensor_view(0).lidar_sensor_view()) {
		total_no_of_rays_for_frontend += profile.sensor_view_configuration.lidar_sensor_view_configuration(lidar_sensor_view_idx).emitted_signal_size();
		total_no_of_reflections += lidar_sensor_view.rendering_result_size();
		lidar_sensor_view_idx++;
	}
	log("Rays configured for simulation tool for full lidar sensor system: " + std::to_string(total_no_of_rays_for_frontend));
	log("Reflections from simulation tool for full lidar sensor system: " + std::to_string(total_no_of_reflections));


	/// Loop over all received lidar sensor front-ends per sensor view
	int lidar_frontend_idx = 0;
	for (auto & lidar_sensor_view : sensor_data.sensor_view(0).lidar_sensor_view()) {
		/// Start new set of detections
		sensor_data.mutable_feature_data()->add_lidar_sensor();
		auto *current_sensor = sensor_data.mutable_feature_data()->mutable_lidar_sensor(lidar_frontend_idx);
		current_sensor->clear_detection();
		//current_sensor->mutable_header()->mutable_mounting_position()->CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(lidar_frontend_idx).mounting_position());
		current_sensor->mutable_header()->mutable_mounting_position()->CopyFrom(lidar_sensor_view.view_configuration().mounting_position());
		current_sensor->mutable_header()->mutable_measurement_time()->CopyFrom(sensor_data.sensor_view(0).global_ground_truth().timestamp());
		current_sensor->mutable_header()->mutable_measurement_time()->set_nanos((current_sensor->mutable_header()->mutable_measurement_time()->nanos()
																	+ profile.sensor_view_configuration.lidar_sensor_view_configuration(lidar_frontend_idx).time_offset_emission(0) / 1000) % 1000000000);
		current_sensor->mutable_header()->mutable_measurement_time()->set_seconds(current_sensor->mutable_header()->mutable_measurement_time()->seconds()
																	+ current_sensor->mutable_header()->mutable_measurement_time()->nanos()
																		+ profile.sensor_view_configuration.lidar_sensor_view_configuration(lidar_frontend_idx).time_offset_emission(0) / 1000 / 1000000000);

		auto no_of_reflections = lidar_sensor_view.rendering_result_size();
		/// Get configuration from profile
		auto no_of_rays_for_frontend = profile.sensor_view_configuration.lidar_sensor_view_configuration(lidar_frontend_idx).emitted_signal_size();
		auto rays_per_beam = profile.rays_per_beam_vertical * profile.rays_per_beam_horizontal;
		auto no_of_beams_for_frontend = profile.beam_center_config.lidar_sensor_view_configuration(lidar_frontend_idx).emitted_signal_size();
		log("Rays configured to shoot in simulation tool for lidar front-end " + std::to_string(lidar_frontend_idx) + ": " + std::to_string(no_of_rays_for_frontend));
		log("Reflections calculated in simulation tool for lidar front-end " + std::to_string(lidar_frontend_idx) + ": " + std::to_string(no_of_reflections));
		log("Beams configured for lidar front-end " + std::to_string(lidar_frontend_idx) + ": " + std::to_string(no_of_beams_for_frontend));

		if (no_of_reflections == 0) {			
			log("Detections from lidar model for lidar front-end " + std::to_string(lidar_frontend_idx) + ": 0");
			lidar_frontend_idx++;
			continue;
		}
		
		/// Lidar cuboid as vector of Structs of type LidarCuboidCell_mW {size_t beam_idx; size_t dist_cell_idx; float signal_strength_in_mW;}
		std::vector<LidarCuboidCell_mW> lidar_cuboid;
		/// Reserve memory for the lidar cuboid to prevent re-allocations at every emplace_back for better performance
		lidar_cuboid.reserve(no_of_reflections);

		/// Run through all rendering results and append them to the lidar cuboid
		size_t reflection_idx = 0;
		for (auto & rendering_result : lidar_sensor_view.rendering_result()) {
			LidarCuboidCell_mW acual_lidar_cuboid_cell;
			if (rendering_result.emitted_signal_idx() < 0) {
				alert("rendering_result(" + std::to_string(reflection_idx) + ").emitted_signal_idx() < 0: " + std::to_string(rendering_result.emitted_signal_idx()));
				continue;
			}
			
			if ((rendering_result.path_length() / 2) >= profile.min_range) {
                acual_lidar_cuboid_cell.beam_idx = std::floor(rendering_result.emitted_signal_idx() / rays_per_beam);
                acual_lidar_cuboid_cell.dist_cell_idx = size_t(std::ceil((rendering_result.path_length() / 2 - profile.min_range) / profile.detection_sensing_parameters.distance_resolution_adc));
                auto signal_strength_in_dBm = rendering_result.received_signal().signal_strength();
				acual_lidar_cuboid_cell.signal_strength_in_mW = (float)std::pow(10, signal_strength_in_dBm / 10); // For summing up signal_strength in distance_cells later, it is needed in mW, not in dBm!

				lidar_cuboid.emplace_back(acual_lidar_cuboid_cell);
			}

			reflection_idx++;
		}

		/// Sort lidar cuboid by beam index
		std::sort(lidar_cuboid.begin(), lidar_cuboid.end(),
			[](const DetectionSensing::LidarCuboidCell_mW &a, const DetectionSensing::LidarCuboidCell_mW &b) {return a.beam_idx < b.beam_idx;}
		);

		/// Run through sorted lidar cuboid cells and find the peaks per beam to calculate detections
		auto beam_idx = lidar_cuboid[0].beam_idx;
		std::vector<LidarBeamCell_mW> lidar_cuboid_cells_of_beam;
		lidar_cuboid_cells_of_beam.reserve(rays_per_beam);
		for (auto lidar_cuboid_cell : lidar_cuboid) {
			/// Collect cells from sorted lidar cuboid that have the same beam
			if (lidar_cuboid_cell.beam_idx == beam_idx) {
				LidarBeamCell_mW next_lidar_beam_cell;
				next_lidar_beam_cell.dist_cell_idx         = lidar_cuboid_cell.dist_cell_idx;
				next_lidar_beam_cell.signal_strength_in_mW = lidar_cuboid_cell.signal_strength_in_mW;
				lidar_cuboid_cells_of_beam.emplace_back(next_lidar_beam_cell);
			}
			/// Process beam after collection of its cells
			else if (!lidar_cuboid_cells_of_beam.empty()) {
				process_collected_beam_cells(current_sensor, &lidar_cuboid_cells_of_beam, rays_per_beam, lidar_frontend_idx,  beam_idx);

				/// Start next beam from empty lidar_cuboid_cells_of_beam with actual lidar_cuboid_cell as first entry
				beam_idx = lidar_cuboid_cell.beam_idx;
				lidar_cuboid_cells_of_beam.clear();
				LidarBeamCell_mW first_lidar_beam_cell;
				first_lidar_beam_cell.dist_cell_idx         = lidar_cuboid_cell.dist_cell_idx;
				first_lidar_beam_cell.signal_strength_in_mW = lidar_cuboid_cell.signal_strength_in_mW;
				lidar_cuboid_cells_of_beam.emplace_back(first_lidar_beam_cell);
			}
		}
		/// Process last beam after collection of its cells
		process_collected_beam_cells(current_sensor, &lidar_cuboid_cells_of_beam, rays_per_beam, lidar_frontend_idx, beam_idx);
		
		log("Detections from lidar model for lidar front-end " + std::to_string(lidar_frontend_idx) + ": " + std::to_string(current_sensor->detection_size()));
		lidar_frontend_idx++;
	}
}

void DetectionSensing::process_collected_beam_cells(LidarDetectionData *current_sensor, std::vector<LidarBeamCell_mW> *lidar_cuboid_cells_of_beam_ptr,
                                                    size_t rays_per_beam, int lidar_frontend_idx, int beam_idx) {
	
	/// Sort cells of actual beam per distance
	auto lidar_cuboid_cells_of_beam = *lidar_cuboid_cells_of_beam_ptr;
	std::sort(lidar_cuboid_cells_of_beam.begin(), lidar_cuboid_cells_of_beam.end(),
		[](const DetectionSensing::LidarBeamCell_mW &a, const DetectionSensing::LidarBeamCell_mW &b) {return a.dist_cell_idx < b.dist_cell_idx;}
	);

	/// sum up and threshold the signal strength within same dist_cells
	LidarBeamCell_mW summed_dist_cell_of_beam;
	std::vector<LidarBeamCell_dBm> thresholded_summed_dist_cells;
	thresholded_summed_dist_cells.reserve(rays_per_beam);
	summed_dist_cell_of_beam.dist_cell_idx = lidar_cuboid_cells_of_beam[0].dist_cell_idx;
	summed_dist_cell_of_beam.signal_strength_in_mW = 0.0;
	for (auto & lidar_cuboid_cell_of_beam : lidar_cuboid_cells_of_beam) {
		if (lidar_cuboid_cell_of_beam.dist_cell_idx == summed_dist_cell_of_beam.dist_cell_idx) {
			summed_dist_cell_of_beam.signal_strength_in_mW += lidar_cuboid_cell_of_beam.signal_strength_in_mW;
		}
		else {
			threshold_summed_beam_cell(&summed_dist_cell_of_beam, &thresholded_summed_dist_cells);
			summed_dist_cell_of_beam = lidar_cuboid_cell_of_beam;
		}
	}
	threshold_summed_beam_cell(&summed_dist_cell_of_beam, &thresholded_summed_dist_cells);

	/// End processing of peaks in this beam, if no more cells exist after thresholding.
	if (!thresholded_summed_dist_cells.empty()) {

		/// Find all peaks and save their distance in m, signal_strength in dBm, echo_pulse_width in m, echo_pulse_start, and echo_no
		std::vector<LidarPeak> peaks_in_beam;
		peaks_in_beam.reserve(profile.detection_sensing_parameters.max_echos_per_beam);
		size_t no_of_peaks_in_beam = 1;
		size_t no_of_dist_cells_in_peak = 1;
		LidarPeak peak;
		peak.distance_in_m = profile.min_range  + ((float)thresholded_summed_dist_cells[0].dist_cell_idx + (float)0.5) * profile.detection_sensing_parameters.distance_resolution_adc;
		peak.signal_strength_in_dBm = thresholded_summed_dist_cells[0].signal_strength_in_dBm;
		peak.epw_in_m = (float)no_of_dist_cells_in_peak * profile.detection_sensing_parameters.distance_resolution_adc;
		peak.echo_pulse_start_in_m = profile.min_range + (float)thresholded_summed_dist_cells[0].dist_cell_idx * profile.detection_sensing_parameters.distance_resolution_adc;
		peak.echo_idx = no_of_peaks_in_beam;
		peaks_in_beam.emplace_back(peak);
		for (size_t thresholded_summed_dist_cell_dB_idx = 1; thresholded_summed_dist_cell_dB_idx < thresholded_summed_dist_cells.size(); thresholded_summed_dist_cell_dB_idx++) {
			/// New peak, if distance btw. peaks > profile.detection_sensing_parameters.echo_separation_distance
			if ((float)thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].dist_cell_idx >
				((float)thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx - 1].dist_cell_idx
					+ profile.detection_sensing_parameters.echo_separation_distance / profile.detection_sensing_parameters.distance_resolution_adc)) {
				no_of_peaks_in_beam++;
				/// End looking for peaks, if already found enough
				if (no_of_peaks_in_beam > profile.detection_sensing_parameters.max_echos_per_beam) break;

				/// Otherwise, start a new peak
				no_of_dist_cells_in_peak = 1;
				peak.distance_in_m = profile.min_range + ((float)thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].dist_cell_idx + (float)0.5) * profile.detection_sensing_parameters.distance_resolution_adc;
				peak.signal_strength_in_dBm = thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].signal_strength_in_dBm;
				peak.epw_in_m = (float)no_of_dist_cells_in_peak * profile.detection_sensing_parameters.distance_resolution_adc;
				peak.echo_pulse_start_in_m = profile.min_range + (float)thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].dist_cell_idx * profile.detection_sensing_parameters.distance_resolution_adc;
				peak.echo_idx = no_of_peaks_in_beam;
				peaks_in_beam.emplace_back(peak);
			}
			else {
				/// Count dist_cells for echo_pulse_width
				no_of_dist_cells_in_peak++;
				peaks_in_beam[no_of_peaks_in_beam - 1].epw_in_m = (float)no_of_dist_cells_in_peak * profile.detection_sensing_parameters.distance_resolution_adc;
				/// Overwrite distance and signal_strength, if actual signal_strength > saved signal_strength
				if (thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].signal_strength_in_dBm > peaks_in_beam[no_of_peaks_in_beam - 1].signal_strength_in_dBm) {
					peaks_in_beam[no_of_peaks_in_beam - 1].distance_in_m = profile.min_range + ((float)thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].dist_cell_idx + (float)0.5) * profile.detection_sensing_parameters.distance_resolution_adc;
					peaks_in_beam[no_of_peaks_in_beam - 1].signal_strength_in_dBm = thresholded_summed_dist_cells[thresholded_summed_dist_cell_dB_idx].signal_strength_in_dBm;
				}
			}
		}

		/// Save detections to OSI for each peak
		for (auto & current_peak : peaks_in_beam) {

			/// Determine the distance depending on the profile.detection_sensing_parameters.echo_determination_mode {"start" (default), "peak"}
			float distance;
			if (profile.detection_sensing_parameters.echo_determination_mode == "peak")
				distance = current_peak.distance_in_m;
			else
				distance = current_peak.echo_pulse_start_in_m;

			/// Get angles of current beam from profile
			auto azimuth = profile.beam_center_config.lidar_sensor_view_configuration(lidar_frontend_idx).emitted_signal(beam_idx).horizontal_angle();
			auto elevation = profile.beam_center_config.lidar_sensor_view_configuration(lidar_frontend_idx).emitted_signal(beam_idx).vertical_angle();
			if ((profile.vertical_angle_clamping != "center") && (profile.beam_step_elevation > 0)) {
				if (profile.vertical_angle_clamping == "max_abs") {
					int sign_elevation = (elevation > 0) - (elevation < 0);
					elevation = sign_elevation * (abs(elevation) + profile.beam_step_elevation * M_PI / 180 / 2);
				}
				else if (profile.vertical_angle_clamping == "top")
					elevation = elevation + profile.beam_step_elevation * M_PI / 180 / 2;
				else if (profile.vertical_angle_clamping == "bottom")
					elevation = elevation - profile.beam_step_elevation * M_PI / 180 / 2;
			}

			/// Noise on distance and distance resolution
			if (profile.detection_sensing_parameters.distance_stddev > 0.0) {
				std::knuth_b generator(std::rand()); // rand used for Windows compatibility
				std::normal_distribution<float> distribution_distance(0, profile.detection_sensing_parameters.distance_stddev);
				auto distance_noise = distribution_distance(generator);
				distance += distance_noise;
			}
			distance = std::round(distance / profile.detection_sensing_parameters.distance_resolution) * profile.detection_sensing_parameters.distance_resolution;

			/// Add new detection and fill it
			auto detection = current_sensor->add_detection();
			detection->mutable_position()->set_distance(distance);
			detection->mutable_position()->set_azimuth(azimuth);
			detection->mutable_position()->set_elevation(elevation);
            detection->set_beam_id(beam_idx);

			if (profile.detection_sensing_parameters.intensity_or_epw == 0) {
                double peak_intensity;
                if (profile.detection_sensing_parameters.range_compensate_intensity) {
                    double signal_strength_in_mW = pow(10, current_peak.signal_strength_in_dBm/10);
                    double signal_strength_in_mW_range_compensated = signal_strength_in_mW * (pow(detection->position().distance(),2));
                    float receiver_aperture_area = M_PI * pow(profile.receiver_aperture_diameter_m,2) / 4.0;
                    float emitted_signal_strength_mW = pow(10.0,profile.max_emitted_signal_strength_in_dBm/10.0);
                    //simulate Velodyne intensity output, calibrated to target reflectivity
                    int output_intensity = round(signal_strength_in_mW_range_compensated / receiver_aperture_area / emitted_signal_strength_mW * M_PI * 100.0);

                    peak_intensity = output_intensity/255.0*100.0;  //velodyne output is scaled [0 255]
                }
                else {
                    /// Calculate peak intensity in % from peak signal_strength in dBm.
                    /// Using scaled log-scaling, where profile.detection_sensing_parameters.signal_strength_threshold_in_dBm: 0 %, max_emitted_signal_strength_in_dBm: 100 %.
                    /// Intensity resolution is considered, as well.
                    peak_intensity = ((current_peak.signal_strength_in_dBm - profile.detection_sensing_parameters.signal_strength_threshold_in_dBm)
                                           / (profile.max_emitted_signal_strength_in_dBm - profile.detection_sensing_parameters.signal_strength_threshold_in_dBm))
                                          * 100;
                }

				/// Noise on intensity and intensity resolution
				/*if (profile.detection_sensing_parameters.distance_stddev > 0.0) {
					std::knuth_b generator(std::rand()); // rand used for Windows compatibility
					std::normal_distribution<float> distribution_intensity(0, profile.detection_sensing_parameters.intensity_stddev);
					auto intensity_noise = distribution_intensity(generator);
					peak_intensity += intensity_noise;
				}
				peak_intensity = std::round(peak_intensity / profile.detection_sensing_parameters.intensity_resolution) * profile.detection_sensing_parameters.intensity_resolution;
                */
				detection->set_intensity(peak_intensity);
			}
			else {

				/// Echo pulse width from geometrical distribution of beam at object, from intensity of peak and from emitted pulse duration
				/// Noise on emitting pulse duration
				std::knuth_b generator(std::rand()); // rand used for Windows compatibility
				std::normal_distribution<float> distribution_pulse_duration(profile.detection_sensing_parameters.pulse_duration_mean, profile.detection_sensing_parameters.pulse_duration_stddev);
				auto emitted_pulse_duration = distribution_pulse_duration(generator);
				auto echo_pulse_width_by_pulse = 299792458.0 * emitted_pulse_duration / 1000000000.0;

				auto echo_pulse_width_by_geometry = current_peak.epw_in_m;

				auto signal_strength_to_epw = profile.detection_sensing_parameters.signal_strength_to_epw;
				size_t k = 1;
				while ((k < signal_strength_to_epw.size() - 1) && (std::round(current_peak.signal_strength_in_dBm * 1000) >= std::round(signal_strength_to_epw[k][0] * 1000))) {
					k = k + 1;
				}
				auto echo_pulse_width_by_intensity = (signal_strength_to_epw[k - 1][1]
														+ (signal_strength_to_epw[k][1] - signal_strength_to_epw[k - 1][1]) / (signal_strength_to_epw[k][0]
														- signal_strength_to_epw[k - 1][0]) * (current_peak.signal_strength_in_dBm - signal_strength_to_epw[k - 1][0]));

				auto peak_epw = echo_pulse_width_by_pulse + echo_pulse_width_by_geometry + echo_pulse_width_by_intensity;
				peak_epw = std::round(peak_epw / profile.detection_sensing_parameters.epw_resolution) * profile.detection_sensing_parameters.epw_resolution;

				detection->set_echo_pulse_width(std::max(0.0, peak_epw));
			}
		}
	}
}

void DetectionSensing::threshold_summed_beam_cell(LidarBeamCell_mW *summed_dist_cell_of_beam_ptr, std::vector<LidarBeamCell_dBm> *thresholded_summed_dist_cells_ptr) {
	auto summed_signal_strength_in_dBm = 10 * std::log10(summed_dist_cell_of_beam_ptr->signal_strength_in_mW);
    double threshold = profile.detection_sensing_parameters.signal_strength_threshold_in_dBm;
    double thres_distance_m = 30.0;     //todo: put to profile
    if (profile.detection_sensing_parameters.range_comp_threshold) {
        float range = profile.min_range  + ((float)summed_dist_cell_of_beam_ptr->dist_cell_idx + (float)0.5) * profile.detection_sensing_parameters.distance_resolution_adc;
        if (range > thres_distance_m) {
            double threshold_mW = pow(10.0,threshold/10.0) * pow(thres_distance_m,2) / pow(range,2);
            threshold = 10 * std::log10(threshold_mW);
        }
    }

    if (summed_signal_strength_in_dBm >= threshold) {
		LidarBeamCell_dBm thresholded_summed_dist_cell_dBm;
		thresholded_summed_dist_cell_dBm.dist_cell_idx = summed_dist_cell_of_beam_ptr->dist_cell_idx;
		thresholded_summed_dist_cell_dBm.signal_strength_in_dBm = summed_signal_strength_in_dBm;
		thresholded_summed_dist_cells_ptr->emplace_back(thresholded_summed_dist_cell_dBm);
	}
}