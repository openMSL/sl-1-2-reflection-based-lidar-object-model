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

#ifndef Speed_of_Light
#define Speed_of_Light 299792458
#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "detectionsensing/DetectionSensing.hpp"
#include <random>
#include <string>
#include <vector>
#include <utility>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#endif

using namespace model;
using namespace osi3;

void DetectionSensing::apply(SensorData &sensor_data) {
    log("Starting Reflection Based Lidar Model");

    uint64_t no_of_lidar_sensors = sensor_data.sensor_view(0).lidar_sensor_view_size();
    log("Number of simulated lidar sensors: " + std::to_string(no_of_lidar_sensors));
    if (profile.sensor_view_configuration.lidar_sensor_view_configuration_size() != no_of_lidar_sensors)
        alert("Number of lidar sensor view different to profile/SensorViewConfiguration!");

    /// Get time stamp from ground truth
    auto timestamp = (double) sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds() + (double) sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos() / 1000000000;
    log("GT time stamp: " + std::to_string(timestamp));

    if(!sensor_data.sensor_view().empty() && (no_of_lidar_sensors > 0) && (timestamp > 0)) {
        /// Loop over all received Sensors
        for (int sensor_idx = 0; sensor_idx < no_of_lidar_sensors; sensor_idx++) {
            /// Pointer to lidar sensor view for readability
            auto *lidar_sensor_view = &sensor_data.sensor_view(0).lidar_sensor_view(sensor_idx);

			uint64_t no_of_reflections = lidar_sensor_view->reflection_size();
			log("No. of rays shot in simulation tool for lidar " + std::to_string(sensor_idx + 1) + ": " + std::to_string(no_of_reflections));

            /// Start new set of detections
            sensor_data.mutable_feature_data()->add_lidar_sensor();
            auto *current_sensor = sensor_data.mutable_feature_data()->mutable_lidar_sensor(sensor_idx);
            current_sensor->clear_detection();
            current_sensor->mutable_header()->mutable_mounting_position()->CopyFrom(profile.sensor_view_configuration.lidar_sensor_view_configuration(sensor_idx).mounting_position());

            /// Get configuration for current sensor
            auto horizontal_angle_min_rad = profile.detection_sensing_parameters.horizontal_angle_min_rad[sensor_idx];
            auto no_of_beams_vertical = profile.detection_sensing_parameters.no_of_beams_vertical[sensor_idx];
            auto no_of_beams_horizontal = profile.detection_sensing_parameters.no_of_beams_horizontal[sensor_idx];
            auto no_of_beams = no_of_beams_vertical * no_of_beams_horizontal;
            auto beam_layer_angles_rad = profile.detection_sensing_parameters.beam_layer_angles_rad[sensor_idx];
            auto beam_height_rad = profile.detection_sensing_parameters.beam_height_rad[sensor_idx];
            auto beam_width_rad = profile.detection_sensing_parameters.beam_width_rad[sensor_idx];
            auto beam_resolution_horizontal_rad = profile.detection_sensing_parameters.beam_resolution_horizontal_rad[sensor_idx];
            auto min_range = profile.min_range[sensor_idx];
            auto rays_per_beam_vertical = profile.detection_sensing_parameters.rays_per_beam_vertical[sensor_idx];
            auto rays_per_beam_horizontal = profile.detection_sensing_parameters.rays_per_beam_horizontal[sensor_idx];
            auto rays_per_beam = rays_per_beam_vertical * rays_per_beam_horizontal;
            auto ray_resolution_vertical_rad = profile.detection_sensing_parameters.ray_resolution_vertical_rad[sensor_idx];
            auto ray_resolution_horizontal_rad = profile.detection_sensing_parameters.ray_resolution_horizontal_rad[sensor_idx];
            auto no_of_rays_horizontal = profile.sensor_view_configuration.lidar_sensor_view_configuration(0).number_of_rays_horizontal();
            auto vertical_start_angle_top_left_rad = beam_layer_angles_rad[0] + beam_height_rad / 2;
            auto distance_resolution_adc = profile.detection_sensing_parameters.distance_resolution_adc[sensor_idx];
            auto distance_resolution = profile.detection_sensing_parameters.distance_resolution[sensor_idx];
            auto intensity_resolution = profile.detection_sensing_parameters.intensity_resolution[sensor_idx];
            auto intensity_threshold_in_dB = profile.detection_sensing_parameters.intensity_threshold_in_dB[sensor_idx];
            auto echo_separation_distance = profile.detection_sensing_parameters.echo_separation_distance[sensor_idx];
            auto echo_determination_mode = profile.detection_sensing_parameters.echo_determination_mode[sensor_idx];
			log("No. of Beams for lidar " + std::to_string(sensor_idx + 1) + ": " + std::to_string(no_of_beams));

            /// Lidar cuboid as vector triplet with std::vector<uint64_t> list_of_beams((std::vector<uint64_t> list_of_dist_cells, std::vector<float> signal_strength_per_cell)
            std::vector< std::pair<uint64_t, std::pair<uint64_t, float> > > lidar_cuboid;
            /// Reserve memory for the lidar cuboid to prevent re-allocations at every emplace_back for better performance
            lidar_cuboid.reserve(no_of_reflections);

            /// Run through all reflections and append valid ones to the lidar cuboid
            for (int reflection_idx = 0; reflection_idx < no_of_reflections; reflection_idx++) {
                if (lidar_sensor_view->reflection(reflection_idx).time_of_flight() > 0) {   //check if reflection is valid
                    auto row_of_ray = uint64_t(std::floor(reflection_idx / no_of_rays_horizontal));
                    auto column_of_ray = reflection_idx - row_of_ray * no_of_rays_horizontal;
                    auto ray_vertical_angle_rad = vertical_start_angle_top_left_rad - (float)row_of_ray * ray_resolution_vertical_rad;
                    auto ray_hozizontal_angle_rad = horizontal_angle_min_rad + (float)column_of_ray * ray_resolution_horizontal_rad;
                    int layer_of_ray = -1;
                    /// sort ray into beam layer
                    for (int layer = 0; layer < beam_layer_angles_rad.size(); layer++) {
                        if (ray_vertical_angle_rad < (beam_layer_angles_rad[layer]+beam_height_rad/2) &&
                        ray_vertical_angle_rad > (beam_layer_angles_rad[layer]-beam_height_rad/2)) {
                            layer_of_ray = layer;
                            break;
                        }
                    }
                    if (layer_of_ray != -1) {
                        /// sort ray into horizontal beam
                        for (int beam_in_layer_of_ray = 0; beam_in_layer_of_ray < no_of_beams_horizontal; beam_in_layer_of_ray++) {
                            float current_beam_min_angle_horizontal_rad = (float)horizontal_angle_min_rad + (float)beam_in_layer_of_ray * beam_resolution_horizontal_rad;
                            float current_beam_max_angle_horizontal_rad = (float)horizontal_angle_min_rad + beam_width_rad + (float)beam_in_layer_of_ray * beam_resolution_horizontal_rad;
                            if (ray_hozizontal_angle_rad > current_beam_min_angle_horizontal_rad && ray_hozizontal_angle_rad < current_beam_max_angle_horizontal_rad) {
                                /// calculate beam index, distance and signal strength
                                auto beam_of_ray = layer_of_ray * no_of_beams_horizontal + beam_in_layer_of_ray;
                                auto tof = lidar_sensor_view->reflection(reflection_idx).time_of_flight();
                                auto distance = tof * Speed_of_Light * 0.5;
                                auto dist_cell = uint64_t(ceil((distance - min_range) / distance_resolution_adc));
                                auto signal_strength_in_dB = lidar_sensor_view->reflection(reflection_idx).signal_strength();
                                auto signal_strength_in_fractions = pow(10, signal_strength_in_dB / 10);    //For summing up signal_strength in distance_cells later, it is needed in fractions, not in dB!

                                lidar_cuboid.emplace_back(std::make_pair(beam_of_ray, std::make_pair(dist_cell, signal_strength_in_fractions)));
                                break;
                            }
                        }
                    }
                }
            }
			auto no_of_valid_reflections = lidar_cuboid.size();
			log("No. of processed valid reflections in lidar cuboid " + std::to_string(sensor_idx + 1) + ": " + std::to_string(no_of_valid_reflections));
            if (no_of_valid_reflections == 0) break;

            /// Sort lidar cuboid by beam index
            sort(lidar_cuboid.begin(), lidar_cuboid.end());

            /// Run through sorted lidar cuboid cells and find the peaks per beam to calculate detections
            uint64_t beam_idx = lidar_cuboid[0].first;
            std::vector< std::pair<uint64_t, float> > lidar_cuboid_cells_of_beam;
            lidar_cuboid_cells_of_beam.reserve(rays_per_beam);
            for (uint64_t lidar_cuboid_cell_idx = 0; lidar_cuboid_cell_idx < no_of_valid_reflections; lidar_cuboid_cell_idx++) {
                if (lidar_cuboid[lidar_cuboid_cell_idx].first == beam_idx) {
                    lidar_cuboid_cells_of_beam.emplace_back(std::make_pair(lidar_cuboid[lidar_cuboid_cell_idx].second.first, lidar_cuboid[lidar_cuboid_cell_idx].second.second));
                } else {
					/// Process beam, if any cells exist there
					if (!lidar_cuboid_cells_of_beam.empty()) {
						/// Sort cells of actual beam per distance
						sort(lidar_cuboid_cells_of_beam.begin(), lidar_cuboid_cells_of_beam.end());

						/// sum up the duplicated dist_cells
						uint64_t summed_dist_cell_idx = 0;
						std::vector< std::pair<uint64_t, float> > summed_dist_cells;
						summed_dist_cells.reserve(rays_per_beam);
						summed_dist_cells.emplace_back(std::make_pair(lidar_cuboid_cells_of_beam[0].first, 0));
						for (auto & lidar_cuboid_cell_of_beam : lidar_cuboid_cells_of_beam) {
							if (lidar_cuboid_cell_of_beam.first == summed_dist_cells[summed_dist_cell_idx].first) {
								summed_dist_cells[summed_dist_cell_idx].second += lidar_cuboid_cell_of_beam.second;
							}
							else {
								summed_dist_cells.emplace_back(std::make_pair(lidar_cuboid_cell_of_beam.first, lidar_cuboid_cell_of_beam.second));
								summed_dist_cell_idx++;
							}
						}

						/// Summed signal strengths are normed by rays_per_beam, changed to dB and thresholded
						std::vector< std::pair<float, float> > thresholded_summed_dist_cells_dB;
						thresholded_summed_dist_cells_dB.reserve(summed_dist_cells.size());
						for (auto & summed_dist_cell : summed_dist_cells) {
							auto signal_strength_in_dB = 10 * std::log10(summed_dist_cell.second / (float)rays_per_beam);
							if (signal_strength_in_dB >= intensity_threshold_in_dB)
								thresholded_summed_dist_cells_dB.emplace_back(std::make_pair(summed_dist_cell.first, signal_strength_in_dB));
						}

						/// End processing of peaks in this beam, if no more cells exist after thresholding.
						if (!thresholded_summed_dist_cells_dB.empty()) {

							/// Find all peaks and save their distance in m, signal_strength in dB, echo_pulse_width in m, echo_pulse_start, and echo_no
							std::vector< std::pair<float, std::pair<float, std::pair<float, std::pair<float, float> > > > > peaks;
							peaks.reserve(profile.detection_sensing_parameters.max_echos_per_beam[sensor_idx]);
							uint64_t no_of_peaks_in_beam = 1;
							int no_of_dist_cells_in_peak = 1;
							peaks.emplace_back(std::make_pair((min_range + (thresholded_summed_dist_cells_dB[0].first + 0.5) * distance_resolution_adc),
								std::make_pair(thresholded_summed_dist_cells_dB[0].second,
									std::make_pair(((float)no_of_dist_cells_in_peak * distance_resolution_adc),
										std::make_pair((min_range + thresholded_summed_dist_cells_dB[0].first * distance_resolution_adc),
											no_of_peaks_in_beam)))));
							for (uint64_t thresholded_summed_dist_cell_dB_idx = 1; thresholded_summed_dist_cell_dB_idx < thresholded_summed_dist_cells_dB.size(); thresholded_summed_dist_cell_dB_idx++) {
								/// New peak, if distance btw. peaks > echo_separation_distance
								if (thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].first > (thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx - 1].first + echo_separation_distance / distance_resolution_adc)) {
									no_of_peaks_in_beam++;
									/// End looking for peaks, if already found enough
									if (no_of_peaks_in_beam > profile.detection_sensing_parameters.max_echos_per_beam[sensor_idx]) break;
									/// Otherwise, start a new peak
									no_of_dist_cells_in_peak = 1;
									peaks.emplace_back(std::make_pair((min_range + (thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].first + 0.5) * distance_resolution_adc),
										std::make_pair(thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].second,
											std::make_pair(((float)no_of_dist_cells_in_peak * distance_resolution_adc),
												std::make_pair((min_range + thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].first * distance_resolution_adc),
													no_of_peaks_in_beam)))));
								}
								else {
									/// Count dist_cells for echo_pulse_width
									no_of_dist_cells_in_peak++;
									/// save new echo pulse width in m
									peaks[no_of_peaks_in_beam - 1].second.second.first = (float)no_of_dist_cells_in_peak * distance_resolution_adc;
									/// Overwrite distance and signal_strength, if actual signal_strength > saved signal_strength
									if (thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].second > peaks[no_of_peaks_in_beam - 1].second.first) {
										/// save distance of peak in m
										peaks[no_of_peaks_in_beam - 1].first = min_range + (thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].first + 0.5) * distance_resolution_adc;
										/// save intensity in dB
										peaks[no_of_peaks_in_beam - 1].second.first = thresholded_summed_dist_cells_dB[thresholded_summed_dist_cell_dB_idx].second;
									}
								}
							}

							/// Save detections to OSI for each peak
							for (auto & peak : peaks) {
								/// Calculate intensity in % from signal_strength in dB. Using log-scaling, where intensity_threshold_in_dB: 0 %, 0 dB: 100 %. Intensity resolution considered, as well.
								auto intensity = (1 - peak.second.first / (intensity_threshold_in_dB)) * 100;
								intensity = std::round(intensity / intensity_resolution) * intensity_resolution;

								auto peak_distance = peak.first;
								auto echo_pulse_width = peak.second.second.first;
								auto echo_pulse_start = peak.second.second.second.first;

								/// Determine the distance depending on the echo_determination_mode
								double distance = 0;
								if (echo_determination_mode == "peak") {
									distance = peak_distance;
								}
								else if (echo_determination_mode == "start") {
									distance = echo_pulse_start;
								}
								else if (echo_determination_mode == "center") {
									distance = echo_pulse_start + echo_pulse_width / 2;
								}
								else if (echo_determination_mode == "end") {
									distance = echo_pulse_start + echo_pulse_width;
								}

								/// Calculate angles of current beam
								auto layer = uint64_t(std::floor(beam_idx / no_of_beams_horizontal));
								uint64_t beam_in_layer = beam_idx - layer * no_of_beams_horizontal;
								//double azimuth = horizontal_start_angle_top_left_rad + double(rays_per_beam_horizontal - 1) / 2 * ray_resolution_horizontal_rad + (float)beam_in_layer * beam_width_rad;
								double azimuth = horizontal_angle_min_rad + (float)beam_in_layer * beam_resolution_horizontal_rad;
								double elevation = beam_layer_angles_rad[layer];

								/// Noise on distance
								std::knuth_b generator(std::rand());     // rand used for Windows compatibility
								std::normal_distribution<double> distribution_distance(0, profile.detection_sensing_parameters.distance_stddev[sensor_idx]);
								double distance_noise = distribution_distance(generator);
								distance += distance_noise;
								distance = round(distance / distance_resolution) * distance_resolution;

								/// Add new detection and fill it
								auto detection = current_sensor->add_detection();
								detection->set_intensity(intensity);
								detection->mutable_position()->set_distance(distance);
								detection->mutable_position()->set_azimuth(azimuth);
								detection->mutable_position()->set_elevation(elevation);
							}
						}
					}

                    /// Start next beam with empty lidar_cuboid_cells_of_beam vector
                    beam_idx = lidar_cuboid[lidar_cuboid_cell_idx].first;
                    lidar_cuboid_cells_of_beam.clear();
                }
            }
            log("No. of Detections from lidar model for lidar "       + std::to_string(sensor_idx + 1) + ": " + std::to_string(current_sensor->detection_size()));
        }
    }
}
