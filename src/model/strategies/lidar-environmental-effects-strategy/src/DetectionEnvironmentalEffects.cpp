//
// Copyright Clemens Linnhoff, M. Sc.
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

#include "detectionenvironmentaleffects/DetectionEnvironmentalEffects.hpp"
#include <random>
#include <vector>

#ifdef _WIN32
#include <math.h>
#else
#include <cmath>
#include <iostream>
#include <fstream>

#endif
//experimental structs
struct Sun {
    double azimuth = 0;     //Azimuth of the sun, counted counterclockwise, 0=north, PI/2 = east, PI=south, 3/2 PI=west. Unit: radian; Range: [0..2PI].
    double elevation = 0;   // Solar elevation angle, 0=x/y plane, PI/2=zenith. Unit: rad; Range: [-PI..PI].
    double intensity = 0;   // Illuminance of the sun, direct sunlight is around 100,00 lx. Unit: lux; Range: [0..inf[.
};

struct SprayProfile {
    //float mean_dissolve_time_s = 1.7;  //mean time clusters stay in the atmosphere
    float std_time_constant = 0.2;
    //float num_clusters_std = 0.0;
    float object_velocity_threshold_in_m_s = 40/3.6;
    //float start_tx_rx_overlap = 0.9;    //Distance to the start of the overlap of sender and receiver beam in m
    //float end_tx_rx_overlap = 1.0;    //Distance to the point, where sender and receiver beams fully overlap
} spray_profile;

bool is_fog = false;
bool is_rain = false;
bool is_snow = false;
bool is_sun = false;
std::vector<float> weatherSequence;

using namespace model;
using namespace osi3;

int isPavement = 0;

void DetectionEnvironmentalEffects::apply(SensorData &sensor_data) {
    log("Apply Environmental Effects");

    auto no_of_lidar_sensors = sensor_data.feature_data().lidar_sensor_size();

    bool any_evironmental_effects = profile.det_envir_effects.calibrated_fog || profile.det_envir_effects.calibrated_rain || profile.det_envir_effects.calibrated_snow || profile.det_envir_effects.calibrated_sun || profile.det_envir_effects.calibrated_spray;
    if(!sensor_data.sensor_view().empty() && (no_of_lidar_sensors > 0) && any_evironmental_effects) {
        /// Loop over all received Sensors
        for (int sensor_idx = 0; sensor_idx < no_of_lidar_sensors; sensor_idx++) {
            auto *current_sensor = sensor_data.mutable_feature_data()->mutable_lidar_sensor(sensor_idx);

            TF::EgoData ego_data;
            if(!TF::get_ego_info(ego_data, sensor_data.sensor_view(sensor_idx)))
                alert("Ego vehicle has no base, no id, or is not contained in GT moving objects.");

            ///Precipitation Detections
            add_hydrometeor_detections(sensor_data, current_sensor, sensor_idx, ego_data);

            ///Spray
            add_spray_detections(sensor_data, current_sensor, sensor_idx, ego_data);

            ///Sun blinding
            add_sun_blinding_detections(sensor_data, current_sensor, ego_data);
        }
    }
}

void DetectionEnvironmentalEffects::add_hydrometeor_detections(osi3::SensorData &sensor_data, osi3::LidarDetectionData *current_sensor, int sensor_idx, const TF::EgoData& ego_data) {
    float weather_intensity = 0;
    if(!weatherSequence.empty()){
        double update_cycle_time_s = ((double) profile.sensor_view_configuration.update_cycle_time().seconds() + profile.sensor_view_configuration.update_cycle_time().nanos() * pow(10, -9));
        double current_time = (double)sensor_data.sensor_view(0).timestamp().seconds()+sensor_data.sensor_view(0).timestamp().nanos() * pow(10, -9);
        int current_time_step = int(current_time/update_cycle_time_s);
        weather_intensity = weatherSequence.at(current_time_step);
    }
    else if(sensor_data.sensor_view(0).global_ground_truth().has_environmental_conditions() &&
       sensor_data.sensor_view(0).global_ground_truth().environmental_conditions().has_precipitation() &&
       sensor_data.sensor_view(0).global_ground_truth().environmental_conditions().precipitation() > 2) {
        std::vector<float> precipitation_intensity_class_mean = {0, 0, 0, 0.2, 0.6, 3, 12.9, 57.5, 150};
        weather_intensity = precipitation_intensity_class_mean[sensor_data.sensor_view(0).global_ground_truth().environmental_conditions().precipitation()];
        is_rain = true;
    }
    else if(sensor_data.sensor_view(0).global_ground_truth().has_environmental_conditions() &&
            sensor_data.sensor_view(0).global_ground_truth().environmental_conditions().has_fog() &&
            sensor_data.sensor_view(0).global_ground_truth().environmental_conditions().fog() > 3) {
        std::vector<float> fog_visibility_class_mean = {10000, 10000, 10000, 25000, 7000, 3000, 1500, 600, 125, 25};
        weather_intensity = fog_visibility_class_mean[sensor_data.sensor_view(0).global_ground_truth().environmental_conditions().fog()];
        is_fog = true;
    }

    if(((is_rain || is_snow) && weather_intensity > 0) || (is_fog && weather_intensity < 10000)) {
        std::random_device generator;
        std::uniform_real_distribution<> uniform_distrib(0, 1);
        int lidar_frontend_idx = 0;

        float atm_detection_probability = 0;
        if(is_rain) {
            atm_detection_probability = static_cast<float>(profile.det_envir_effects.rain_det_prob_factor * weather_intensity);
        } else if(is_snow) {
            atm_detection_probability = static_cast<float>(profile.det_envir_effects.snow_det_prob_factor * weather_intensity);
        } else if(is_fog) {
            atm_detection_probability = static_cast<float>(profile.det_envir_effects.fog_det_prob_factor * weather_intensity);
        }

        for (auto & lidar_sensor_view : sensor_data.sensor_view(0).lidar_sensor_view()) {
            auto current_lidar_sensor_view_config = profile.beam_center_config.lidar_sensor_view_configuration(lidar_frontend_idx);
            std::vector<int> existing_detection_idx(current_lidar_sensor_view_config.emitted_signal_size(), -1);
            osi3::LidarDetectionData existing_detections = get_beam_indices(sensor_data, existing_detection_idx);
            sensor_data.mutable_feature_data()->mutable_lidar_sensor(lidar_frontend_idx)->clear_detection();
            for (int emitted_signal_idx = 0; emitted_signal_idx < current_lidar_sensor_view_config.emitted_signal_size(); emitted_signal_idx++) {
                osi3::LidarDetection existing_detection;
                if (existing_detection_idx.at(emitted_signal_idx) != -1) {
                    existing_detection.CopyFrom(existing_detections.detection(existing_detection_idx.at(emitted_signal_idx)));
                } else {
                    existing_detection.mutable_position()->set_distance(profile.max_range);
                    existing_detection.set_intensity(0.0);
                }
                std::vector<double> environmental_detection_distances;
                std::vector<double> environmental_detection_intensities;
                const auto &current_beam = current_lidar_sensor_view_config.emitted_signal(emitted_signal_idx);
                int layer_idx = (int)search_closest(profile.beam_center_elevation, current_beam.vertical_angle()/M_PI*180.0);
                // decide by chance if detection from raindrop exists
                auto random_var =  (float)uniform_distrib(generator);
                if (random_var > (1-(atm_detection_probability * profile.det_envir_effects.layer_comp_factors.at(layer_idx)))) {
                    // decide by statistics at what range
                    double distance_distr_mu = 1.362;
                    double distance_distr_sigma = 0.784;
                    std::lognormal_distribution<double> distribution_distance(distance_distr_mu, distance_distr_sigma);
                    double rain_detection_distance = std::abs(distribution_distance(generator));
                    rain_detection_distance = round(rain_detection_distance / profile.detection_sensing_parameters.distance_resolution) * profile.detection_sensing_parameters.distance_resolution;

                    osi3::Spherical3d position_sensor_coord;
                    position_sensor_coord.set_distance(rain_detection_distance);
                    position_sensor_coord.set_azimuth(current_beam.horizontal_angle());
                    position_sensor_coord.set_elevation(current_beam.vertical_angle());
                    std::poisson_distribution<int> distribution_intensity(profile.det_envir_effects.intensity_distr_lambda);
                    double intensity = distribution_intensity(generator)/255.0*100.0;
                    if (is_detection_above_ground(sensor_data, position_sensor_coord, ego_data, sensor_idx)) {
                        environmental_detection_distances.emplace_back(position_sensor_coord.distance());
                        environmental_detection_intensities.emplace_back(intensity);
                    }
                }
                //apply attenuation to existing detection and append to vector
                double attenuation = 0.001;
                if (is_fog) {
                    attenuation = 3.0/weather_intensity;
                } else if (is_rain) {
                    attenuation = weather_intensity * 7.677*pow(10,-5);     //todo: put to profile
                }
                add_attenuated_existing_detection(environmental_detection_intensities, environmental_detection_distances, existing_detection_idx, emitted_signal_idx, existing_detection, attenuation, existing_detection.position().distance());

                //get detection with maximum intensity todo: multi-echo
                add_max_detection(environmental_detection_intensities, environmental_detection_distances, current_sensor, current_beam);
            }
            lidar_frontend_idx++;
        }
    }
}

void DetectionEnvironmentalEffects::add_spray_detections(osi3::SensorData &sensor_data, osi3::LidarDetectionData *current_sensor, int sensor_idx, const TF::EgoData& ego_data) {
    double water_film_height = sensor_data.sensor_view(0).global_ground_truth().lane(0).classification().road_condition().surface_water_film();
    if (water_film_height <= 0.0) return;
    auto current_lidar_sensor_view_config = profile.beam_center_config.lidar_sensor_view_configuration(0);  //todo:consider multiple front-ends
    osi3::MountingPosition mounting_pose = sensor_data.sensor_view(0).lidar_sensor_view(0).view_configuration().mounting_position();

    //todo: consider sensor orientation
    osi3::Vector3d object_min_dimension_in_sensor_coord;
    object_min_dimension_in_sensor_coord.set_x(-ego_data.ego_base.dimension().length()/2.0-ego_data.ego_vehicle_attributes.bbcenter_to_rear().x()-mounting_pose.position().x());
    object_min_dimension_in_sensor_coord.set_y(-ego_data.ego_base.dimension().width()/2.0-ego_data.ego_vehicle_attributes.bbcenter_to_rear().y()-mounting_pose.position().y());
    osi3::Vector3d object_max_dimension_in_sensor_coord;
    object_max_dimension_in_sensor_coord.set_x(ego_data.ego_base.dimension().length()/2.0-ego_data.ego_vehicle_attributes.bbcenter_to_rear().x()-mounting_pose.position().x());
    object_max_dimension_in_sensor_coord.set_y(ego_data.ego_base.dimension().width()/2.0-ego_data.ego_vehicle_attributes.bbcenter_to_rear().y()-mounting_pose.position().y());

    append_spray_cluster(sensor_data, ego_data, mounting_pose);
    std::vector<int> cluster_to_erase = update_spray_cluster(sensor_data, ego_data, mounting_pose);

    std::vector<int> existing_detection_idx(current_lidar_sensor_view_config.emitted_signal_size(), -1);
    osi3::LidarDetectionData existing_detections = get_beam_indices(sensor_data, existing_detection_idx);
    sensor_data.mutable_feature_data()->mutable_lidar_sensor(0)->clear_detection();

    std::random_device generator;
    std::uniform_real_distribution<> uniform_distrib(0, 1);
    std::uniform_real_distribution<> uniform_distrib_0_05(0.0, 0.5);
    std::normal_distribution<> normal_distrib_0_5(0.5, 0.05);

    for (int emitted_signal_idx = 0; emitted_signal_idx < current_lidar_sensor_view_config.emitted_signal_size(); emitted_signal_idx++) {
        osi3::LidarDetection existing_detection;
        if (existing_detection_idx.at(emitted_signal_idx) != -1) {
            existing_detection.CopyFrom(existing_detections.detection(existing_detection_idx.at(emitted_signal_idx)));
            simulate_wet_pavement(sensor_data, ego_data, existing_detection, sensor_idx, water_film_height);
            isPavement = 1;
        } else {
            existing_detection.mutable_position()->set_distance(profile.max_range);
            existing_detection.set_intensity(0.0);
        }

        const auto &current_beam = current_lidar_sensor_view_config.emitted_signal(emitted_signal_idx);
        double max_distance_to_spray_detection = existing_detection.position().distance();
        std::vector<double> spray_detection_distances;
        std::vector<double> spray_detection_intensities;
        double distance_in_spray_cluster = 0.0;
        for (auto &current_spray_cluster: spray_cluster_global) {
            if (current_beam.horizontal_angle() > current_spray_cluster.max_azimuth_rad || current_beam.horizontal_angle() < current_spray_cluster.min_azimuth_rad || current_spray_cluster.age_in_s < 0) {
                continue;
            }
            auto cluster_center_sensor_coord = TF::transform_position_from_world_to_sensor_coordinates(current_spray_cluster.position_global, ego_data, mounting_pose);
            //check if cluster above ego vehicle
            if (cluster_center_sensor_coord.x()+current_spray_cluster.radius>object_min_dimension_in_sensor_coord.x() &&
                    cluster_center_sensor_coord.x()-current_spray_cluster.radius<object_max_dimension_in_sensor_coord.x() &&
                    cluster_center_sensor_coord.y()+current_spray_cluster.radius>object_min_dimension_in_sensor_coord.y() &&
                    cluster_center_sensor_coord.y()-current_spray_cluster.radius<object_max_dimension_in_sensor_coord.y()) {
                continue;
            }

            std::vector<double> dist_to_intersect = intersection_with_sphere(cluster_center_sensor_coord, current_spray_cluster.radius, current_beam.horizontal_angle(),
                                                                             current_beam.vertical_angle());
            if (!dist_to_intersect.empty()) {
                if (dist_to_intersect.size() == 2 && *std::min_element(dist_to_intersect.begin(), dist_to_intersect.end()) < max_distance_to_spray_detection) {
                    double length_beam_in_cluster = dist_to_intersect.at(1) - dist_to_intersect.at(0);
                    distance_in_spray_cluster += length_beam_in_cluster;
                    auto mu = float(-2.3);   //todo: put in profile
                    auto sigma = float(1.1); //todo: put to profile
                    auto detection_distribution = std::lognormal_distribution(mu, sigma);
                    float existence_probability = std::min(float(1.0), detection_distribution(generator));
                    existence_probability *= std::min(float(1.0), std::exp(-current_spray_cluster.age_in_s/current_spray_cluster.time_constant));

                    double random_number = uniform_distrib(generator);
                    if (random_number > (1.0 - existence_probability)) {
                        double cluster_center_range = dist_to_intersect.at(0) / 2.0 + dist_to_intersect.at(1) / 2.0;
                        std::normal_distribution<double> distribution_dist_in_cluster(cluster_center_range, length_beam_in_cluster / 6.0);
                        double current_distance_to_spray_detection = distribution_dist_in_cluster(generator);
                        double current_intensity = normal_distrib_0_5(generator)/255.0*100.0;
                        double mean_attenuation = 0.02;     //todo: put in profile
                        double attenuation_factor = exp(-2.0 * mean_attenuation * distance_in_spray_cluster);
                        double attenuated_intensity = attenuation_factor * current_intensity;
                        osi3::Spherical3d tmp_detection_sensor_sph;
                        tmp_detection_sensor_sph.set_distance(current_distance_to_spray_detection);
                        tmp_detection_sensor_sph.set_azimuth(current_beam.horizontal_angle());
                        tmp_detection_sensor_sph.set_elevation(current_beam.vertical_angle());
                        if (current_distance_to_spray_detection < max_distance_to_spray_detection && is_detection_above_ground(sensor_data, tmp_detection_sensor_sph, ego_data, sensor_idx)) {
                            spray_detection_distances.emplace_back(current_distance_to_spray_detection);
                            spray_detection_intensities.emplace_back(attenuated_intensity);
                        }
                    }
                }
            }
        }
        //apply attenuation depending on distance_in_spray_cluster to existing detection and append to vector
        add_attenuated_existing_detection(spray_detection_intensities, spray_detection_distances, existing_detection_idx, emitted_signal_idx, existing_detection, 0.02, distance_in_spray_cluster);

        //get detection with maximum intensity todo: multi-echo
        add_max_detection(spray_detection_intensities, spray_detection_distances, current_sensor, current_beam);
    }
    //remove clusters, if age is too high
    sort(cluster_to_erase.begin(), cluster_to_erase.end(), std::greater<>());
    for (int current_cluster_idx: cluster_to_erase) {
        spray_cluster_global.erase(spray_cluster_global.begin() + current_cluster_idx);
    }
}

void DetectionEnvironmentalEffects::simulate_wet_pavement(osi3::SensorData &sensor_data, const TF::EgoData &ego_data, LidarDetection &existing_detection, int sensor_idx, double water_film_height) {
    double sensor_height_over_ground;
    double rear_axle_over_ground;
    if (ego_data.ego_vehicle_attributes.has_radius_wheel()) {
        rear_axle_over_ground = ego_data.ego_vehicle_attributes.radius_wheel();
    } else {
        rear_axle_over_ground = 0.3;
    }

    if (sensor_data.sensor_view(sensor_idx).lidar_sensor_view_size() > 0 && sensor_data.sensor_view(sensor_idx).lidar_sensor_view(0).view_configuration().has_mounting_position()) {
        sensor_height_over_ground = sensor_data.sensor_view(sensor_idx).lidar_sensor_view(0).view_configuration().mounting_position().position().z() + rear_axle_over_ground;
    } else {
        sensor_height_over_ground = profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position().position().z() + rear_axle_over_ground;
    }
    double detection_z_coord = sin(existing_detection.position().elevation()) * existing_detection.position().distance();
    double pavement_z_tolerance = 0.2;

    double detection_y_ccord = sin(existing_detection.position().azimuth()) * existing_detection.position().distance();

    //The following simulation is based on Hahner et al. (2022)
    if (detection_z_coord > -sensor_height_over_ground-pavement_z_tolerance && detection_z_coord < -sensor_height_over_ground+pavement_z_tolerance) {
        if (detection_y_ccord > 2.0 && detection_y_ccord < 7.0){
            double Gamma = existing_detection.intensity()/100.0*255.0/100.0;
            double n_air = 1.0003;
            double n_w = 1.33;
            double pavementDepth_m = 0.0005;
            double range = existing_detection.position().distance();

            double a_in = atan(range/sensor_height_over_ground);

            //First reflection parameters from air to water transition
            double a_aout = asin(sin(a_in)*n_air/n_w);   //output angle according to Snell's law (eq. 12)
            //double t_as = 2*n_air*cos(a_in)/(n_air*cos(a_in)+n_air*cos(a_aout));
            double t_ap = 2*n_air*cos(a_in)/(n_w*cos(a_in)+n_air*cos(a_aout));
            double power_fraction_transmittance = cos(a_in)*n_air/n_w/cos(a_aout);
            //t_as = pow(t_as,2)/power_fraction_transmittance;     //eq. 14
            t_ap = pow(t_ap,2)/power_fraction_transmittance;

            //Second reflection parameters from water to air transition
            double a_wout = asin(sin(a_aout)*n_w/n_air);   //output angle according to Snell's law (eq. 12)
            //double r_ws = (n_w*cos(a_aout)-n_air*cos(a_wout))/(n_w*cos(a_aout)+n_air*cos(a_wout));
            //double t_ws = 2*n_w*cos(a_aout)/(n_w*cos(a_aout)+n_air*cos(a_wout));
            double r_wp = (n_air*cos(a_aout)-n_w*cos(a_wout))/(n_air*cos(a_aout)+n_w*cos(a_wout));
            double t_wp = 2*n_w*cos(a_aout)/(n_air*cos(a_aout)+n_w*cos(a_wout));
            power_fraction_transmittance = cos(a_aout)*n_w/n_air/cos(a_wout);
            //r_ws = pow(r_ws,2);  //eq. 13
            //t_ws = pow(t_ws,2)/power_fraction_transmittance;     //eq. 14
            r_wp = pow(r_wp,2);
            t_wp = pow(t_wp,2)/power_fraction_transmittance;

            //double t_s = t_as*Gamma*t_ws/(1-Gamma*r_ws);   //eq. 16
            double t_p = t_ap*Gamma*t_wp/(1-Gamma*r_wp);
            double gamma = fmin(fmax(water_film_height/pavementDepth_m,0),1);     //eq. 19
            double attenuated_intensity = ((1-gamma)*Gamma + gamma * t_p) * 100.0 * 100.0 / 255.0;
            attenuated_intensity = attenuated_intensity * 0.3;

            existing_detection.set_intensity(attenuated_intensity);
        } else {
            existing_detection.set_intensity(existing_detection.intensity()+2.0/255.0*100.0);
        }
    }
}

osi3::LidarDetectionData DetectionEnvironmentalEffects::get_beam_indices(osi3::SensorData &sensor_data, std::vector<int>& existing_detection_idx){
    osi3::LidarDetectionData existing_detections;
    for (int detection_idx = 0; detection_idx < sensor_data.feature_data().lidar_sensor(0).detection_size(); detection_idx++) {
        auto current_detection = sensor_data.feature_data().lidar_sensor(0).detection(detection_idx);
        int emitted_signal_idx = (int)current_detection.beam_id();
        existing_detection_idx.at(emitted_signal_idx) = detection_idx;
        existing_detections.add_detection()->CopyFrom(current_detection);
    }
    return existing_detections;
}

std::vector<int> DetectionEnvironmentalEffects::update_spray_cluster(osi3::SensorData &sensor_data, const TF::EgoData &ego_data, osi3::MountingPosition &mounting_pose){
    double wind_direction_deg = 208.0;    //wind direction clock wise from north
    double wind_speed = 2.0;  //absolute wind speed in m/s
    double wind_direction[] = {sin(wind_direction_deg*M_PI/180.0), cos(wind_direction_deg*M_PI/180.0), 0.0};
    double update_cycle_time_s = ((double) profile.sensor_view_configuration.update_cycle_time().seconds() + profile.sensor_view_configuration.update_cycle_time().nanos() * pow(10, -9));

    for (auto &current_spray_cluster: spray_cluster_global) {
        if (current_spray_cluster.age_in_s > -update_cycle_time_s) {
            auto last_velocity = current_spray_cluster.velocity_in_m_s;
            osi3::Vector3d relative_velocity;
            relative_velocity.set_x(wind_direction[0]*wind_speed + last_velocity.x());
            relative_velocity.set_y(wind_direction[1]*wind_speed + last_velocity.y());
            relative_velocity.set_z(wind_direction[2]*wind_speed + last_velocity.z());
            auto relative_velocity_abs = TF::get_vector_abs(relative_velocity);
            osi3::Vector3d relative_velocity_direction;
            relative_velocity_direction.set_x(relative_velocity.x() / relative_velocity_abs);
            relative_velocity_direction.set_y(relative_velocity.y() / relative_velocity_abs);
            relative_velocity_direction.set_z(relative_velocity.z() / relative_velocity_abs);
            current_spray_cluster.velocity_in_m_s.set_x(current_spray_cluster.drag_factor * (relative_velocity_direction.x() * pow(relative_velocity_abs, 2)) * update_cycle_time_s + last_velocity.x());
            current_spray_cluster.velocity_in_m_s.set_y(current_spray_cluster.drag_factor * (relative_velocity_direction.y() * pow(relative_velocity_abs, 2)) * update_cycle_time_s + last_velocity.y());
            current_spray_cluster.velocity_in_m_s.set_z(current_spray_cluster.drag_factor * (relative_velocity_direction.z() * pow(relative_velocity_abs, 2)) * update_cycle_time_s + last_velocity.z());
            auto last_position = current_spray_cluster.position_global;
            current_spray_cluster.position_global.set_x(last_position.x() + current_spray_cluster.velocity_in_m_s.x() * update_cycle_time_s);
            current_spray_cluster.position_global.set_y(last_position.y() + current_spray_cluster.velocity_in_m_s.y() * update_cycle_time_s);
            current_spray_cluster.position_global.set_z(last_position.z() + current_spray_cluster.velocity_in_m_s.z() * update_cycle_time_s);
        }

        //cluster boundaries and distance
        get_min_max_azimuth_of_cluster(current_spray_cluster, mounting_pose, ego_data);

        //update cluster age
        current_spray_cluster.age_in_s += (float)update_cycle_time_s;
    }

    //sort clusters by distance
    sort(spray_cluster_global.begin(), spray_cluster_global.end(), [](const SprayCluster& a, const SprayCluster& b) {
        return a.dist_to_sensor < b.dist_to_sensor;
    });

    //determine clusters to erase
    std::vector<int> cluster_to_erase;
    int cluster_idx = 0;
    for (auto &current_spray_cluster: spray_cluster_global) {
        if (current_spray_cluster.age_in_s > 4*current_spray_cluster.time_constant) {
            cluster_to_erase.push_back(cluster_idx);
        }
        cluster_idx++;
    }
    return cluster_to_erase;
}

//from https://viclw17.github.io/2018/07/16/raytracing-ray-sphere-intersection/
std::vector<double> DetectionEnvironmentalEffects::intersection_with_sphere(const osi3::Vector3d& center, float radius, double azimuth, double elevation){
    std::vector<double> output;
    osi3::Vector3d origin;
    osi3::Vector3d direction;
    direction.set_x(cos(azimuth) * cos(elevation));
    direction.set_y(sin(azimuth) * cos(elevation));
    direction.set_z(sin(elevation));

    osi3::Vector3d origin_to_cluster_center = TF::vector_translation(origin, center, -1.0);
    double a = TF::dot_product(direction, direction);
    double b = 2.0 * TF::dot_product(origin_to_cluster_center, direction);
    double c = TF::dot_product(origin_to_cluster_center, origin_to_cluster_center) - radius * radius;
    double discriminant = b*b - 4*a*c;
    if(discriminant < 0.0){
        return output;
    }
    else{
        double numerator = -b - sqrt(discriminant);
        if (numerator > 0.0) {
            output.push_back(numerator / (2.0 * a));
        }

        numerator = -b + sqrt(discriminant);
        if (numerator > 0.0) {
            output.push_back(numerator / (2.0 * a));
        }

        return output;
    }
}

void DetectionEnvironmentalEffects::append_spray_cluster(SensorData &sensor_data, const TF::EgoData &ego_data, MountingPosition &mounting_pose) {
    std::vector<SprayVolume> spray_volumes = DetectionEnvironmentalEffects::define_spray_volumes(sensor_data, ego_data, mounting_pose);
    std::random_device generator;
    std::uniform_real_distribution<> uniform_distrib(0, 1);
    std::uniform_real_distribution<> uniform_distrib_05_1(0.5, 1);
    double update_cycle_time_s = ((double) profile.sensor_view_configuration.update_cycle_time().seconds() + profile.sensor_view_configuration.update_cycle_time().nanos() * pow(10, -9));
    for(auto &current_spray_volume : spray_volumes) {
        auto &current_object = *current_spray_volume.corresponding_object;
        //todo: get lane id of current object
        auto water_film_height = (float)sensor_data.sensor_view(0).global_ground_truth().lane(0).classification().road_condition().surface_water_film();
        double object_velocity = std::sqrt(pow(current_object.base().velocity().x(),2)+pow(current_object.base().velocity().y(),2)+pow(current_object.base().velocity().z(),2));
        if (object_velocity > spray_profile.object_velocity_threshold_in_m_s && water_film_height > 0) {
            osi3::Dimension3d spray_vol_dimension = current_spray_volume.volume.dimension();
            //todo: adjust for object class
            double mean_num_clusters = (0.20*water_film_height+0.1) * (object_velocity*3.6-50.0); //todo: put in profile
            //std::normal_distribution<double> distribution_num_clusters(mean_num_clusters, spray_profile.num_clusters_std);
            int num_spray_cluster = round(std::max(0.0,mean_num_clusters));
            for (int cluster_idx = 0; cluster_idx < num_spray_cluster; cluster_idx++) {
                osi3::Vector3d current_spray_cluster_vol_coord;
                current_spray_cluster_vol_coord.set_x(spray_vol_dimension.length() * uniform_distrib(generator) - spray_vol_dimension.length() / 2);
                current_spray_cluster_vol_coord.set_y(spray_vol_dimension.width() * uniform_distrib(generator) - spray_vol_dimension.width() / 2);
                current_spray_cluster_vol_coord.set_z(spray_vol_dimension.height() * uniform_distrib(generator) - spray_vol_dimension.height() / 2);

                // transform to global coord and add to spray_cluster_global
                osi3::Vector3d current_spray_cluster_world_coord = TF::transform_position_from_object_to_world_coordinates(current_spray_cluster_vol_coord,
                                                                                                                           current_spray_volume.volume);
                SprayCluster current_spray_cluster_global;
                current_spray_cluster_global.position_global = current_spray_cluster_world_coord;
                auto mu = float(-1.2);   //todo: put in profile
                auto sigma = float(0.8); //todo: put in profile
                auto distribution_cluster_radius = std::lognormal_distribution<float>(mu, sigma);
                current_spray_cluster_global.radius = std::min(float(1.0),distribution_cluster_radius(generator));
                current_spray_cluster_global.age_in_s = float(-update_cycle_time_s);
                auto time_constant = float(water_film_height*0.02 * (object_velocity*3.6-50) + 0.2);
                std::normal_distribution<float> distribution_time_constant(time_constant, spray_profile.std_time_constant);
                current_spray_cluster_global.time_constant = distribution_time_constant(generator);
                current_spray_cluster_global.object_velocity_in_m_s = object_velocity;
                current_spray_cluster_global.velocity_in_m_s.set_x(current_object.base().velocity().x());
                current_spray_cluster_global.velocity_in_m_s.set_y(current_object.base().velocity().y());
                current_spray_cluster_global.velocity_in_m_s.set_z(current_object.base().velocity().z()+5.0*uniform_distrib(generator));
                auto distribution_drag_factor = std::normal_distribution<float>(-0.12, 0.04);
                current_spray_cluster_global.drag_factor = distribution_drag_factor(generator);
                spray_cluster_global.emplace_back(current_spray_cluster_global);
            }
        }
    }
}

std::vector<DetectionEnvironmentalEffects::SprayVolume>
DetectionEnvironmentalEffects::define_spray_volumes(osi3::SensorData &sensor_data, const TF::EgoData &ego_data, osi3::MountingPosition &mounting_pose) {
    std::vector<SprayVolume> spray_volumes;
    for (auto& current_moving_object : sensor_data.sensor_view(0).global_ground_truth().moving_object()) {
        double abs_velocity = TF::get_vector_abs(current_moving_object.base().velocity());

        if (abs_velocity > spray_profile.object_velocity_threshold_in_m_s ) {
            double update_cycle_time_s = ((double) profile.sensor_view_configuration.update_cycle_time().seconds() + profile.sensor_view_configuration.update_cycle_time().nanos() * pow(10, -9));
            SprayVolume current_spray_volume;
            current_spray_volume.volume.mutable_dimension()->set_length(abs_velocity * update_cycle_time_s);
            current_spray_volume.volume.mutable_dimension()->set_width(3.4);
            current_spray_volume.volume.mutable_dimension()->set_height(1.5);
            current_spray_volume.volume.mutable_orientation()->set_yaw(current_moving_object.base().orientation().yaw());
            double distance_obj_2_spray_x = current_moving_object.base().dimension().length() / 2 + current_spray_volume.volume.dimension().length() / 2;
            current_spray_volume.volume.mutable_position()->set_x(current_moving_object.base().position().x() - cos(current_moving_object.base().orientation().yaw()) * (distance_obj_2_spray_x));
            current_spray_volume.volume.mutable_position()->set_y(current_moving_object.base().position().y() - sin(current_moving_object.base().orientation().yaw()) * (distance_obj_2_spray_x));
            current_spray_volume.volume.mutable_position()->set_z(
                    current_moving_object.base().position().z() - current_moving_object.base().dimension().height() / 2 + current_spray_volume.volume.dimension().height() / 2);

            current_spray_volume.corresponding_object = &current_moving_object;

            spray_volumes.emplace_back(current_spray_volume);

            //only for debugging
            /*auto *debug_object = sensor_data.mutable_moving_object()->Add();
            osi3::DetectedItemHeader current_header;
            current_header.mutable_tracking_id()->set_value(current_moving_object.id().value());
            current_header.set_existence_probability(1);
            debug_object->mutable_header()->CopyFrom(current_header);
            osi3::Vector3d spray_position_ego = TransformationFunctions::transform_position_from_world_to_ego_coordinates(current_spray_volume.volume.position(), ego_data);
            osi3::Orientation3d spray_orientation_ego = TransformationFunctions::calc_relative_orientation_to_local(current_spray_volume.volume.orientation(), ego_data.ego_base.orientation());
            osi3::BaseMoving debug_base;
            debug_base.mutable_position()->set_x(spray_position_ego.x());
            debug_base.mutable_position()->set_y(spray_position_ego.y());
            debug_base.mutable_position()->set_z(spray_position_ego.z());
            debug_base.mutable_orientation()->set_yaw(spray_orientation_ego.yaw());
            debug_base.mutable_dimension()->CopyFrom(current_spray_volume.volume.dimension());
            debug_object->mutable_base()->CopyFrom(debug_base);*/
        }
    }
    return spray_volumes;
}

void
DetectionEnvironmentalEffects::get_min_max_azimuth_of_cluster(SprayCluster &cluster, const osi3::MountingPosition &mounting_pose, const TF::EgoData &ego_data) {
    auto cluster_sensor_coord = TF::transform_position_from_world_to_sensor_coordinates(cluster.position_global, ego_data, mounting_pose);
    auto cluster_sensor_coord_sph = TF::transform_cartesian_to_spherical(cluster_sensor_coord);
    double angle_section_of_radius = atan2(cluster.radius, cluster_sensor_coord_sph.distance());

    cluster.min_azimuth_rad = cluster_sensor_coord_sph.azimuth()-angle_section_of_radius;
    cluster.max_azimuth_rad = cluster_sensor_coord_sph.azimuth()+angle_section_of_radius;
    cluster.dist_to_sensor = cluster_sensor_coord_sph.distance();
}

bool DetectionEnvironmentalEffects::check_if_sensor_in_cluster_volume(SprayCluster &cluster, const osi3::MountingPosition& mounting_pose, const TF::EgoData& ego_data) {
    auto cluster_sensor_coord = TF::transform_position_from_world_to_sensor_coordinates(cluster.position_global, ego_data, mounting_pose);
    auto cluster_sensor_coord_sph = TF::transform_cartesian_to_spherical(cluster_sensor_coord);
    return cluster_sensor_coord_sph.distance() < cluster.radius;
}


void DetectionEnvironmentalEffects::add_sun_blinding_detections(SensorData &sensor_data, osi3::LidarDetectionData *current_sensor, const TF::EgoData &ego_data) {
    //todo: temporary sun definition until corresponding osi message is defined
    Sun sun;
    sun.azimuth = 1.5 * M_PI/180;
    sun.elevation = 19.1 * M_PI/180;
    sun.intensity = 0.0;

    if (is_sun) {
        double update_cycle_time_s = ((double) profile.sensor_view_configuration.update_cycle_time().seconds() + profile.sensor_view_configuration.update_cycle_time().nanos() * pow(10, -9));
        double current_time = (double)sensor_data.sensor_view(0).timestamp().seconds()+sensor_data.sensor_view(0).timestamp().nanos() * pow(10, -9);
        int current_time_step = int(current_time/update_cycle_time_s);
        sun.intensity = weatherSequence.at(current_time_step);
    }

    if (sun.intensity > 0.0) {
        std::random_device generator;
        std::uniform_real_distribution<> uniform_distrib(0, 1);

        int lidar_frontend_idx = 0;
        for (auto & lidar_sensor_view : sensor_data.sensor_view(0).lidar_sensor_view()) {
            osi3::MountingPosition mounting_pose = lidar_sensor_view.view_configuration().mounting_position();
            Orientation3d orientation_world_coord = TF::calc_relative_orientation_from_local(mounting_pose.orientation(), ego_data.ego_base.orientation());
            Sun sun_sensor_coord;
            sun_sensor_coord.azimuth = cos(orientation_world_coord.roll())*(sun.azimuth - orientation_world_coord.yaw()) + sin(orientation_world_coord.roll())*(sun.elevation - orientation_world_coord.yaw());
            sun_sensor_coord.elevation = cos(orientation_world_coord.roll())*(sun.elevation + orientation_world_coord.pitch()) + sin(orientation_world_coord.roll())*(sun.azimuth - orientation_world_coord.pitch());
            if (sun_sensor_coord.azimuth > M_PI) sun_sensor_coord.azimuth -=M_PI;
            double blinding_angle_region = 6.0;
            double min_azimuth = sun_sensor_coord.azimuth - blinding_angle_region/180*M_PI;
            double max_azimuth = sun_sensor_coord.azimuth + blinding_angle_region/180*M_PI;
            double min_elevation = sun_sensor_coord.elevation - blinding_angle_region/180*M_PI;
            double max_elevation = sun_sensor_coord.elevation + blinding_angle_region/180*M_PI;

            auto current_lidar_sensor_view_config = profile.beam_center_config.lidar_sensor_view_configuration(lidar_frontend_idx);
            std::vector<int> existing_detection_idx(current_lidar_sensor_view_config.emitted_signal_size(), -1);
            //osi3::LidarDetectionData existing_detections = get_beam_indices(sensor_data, existing_detection_idx);
            for (int emitted_signal_idx = 0; emitted_signal_idx < current_lidar_sensor_view_config.emitted_signal_size(); emitted_signal_idx++) {
                const auto &current_beam = current_lidar_sensor_view_config.emitted_signal(emitted_signal_idx);

                //todo: consider case, where area includes 0°
                if (current_beam.horizontal_angle() > min_azimuth && current_beam.horizontal_angle() < max_azimuth && current_beam.vertical_angle() > min_elevation && current_beam.vertical_angle() < max_elevation) {
                    double azimuth_distance = sun_sensor_coord.azimuth - current_beam.horizontal_angle();
                    double elevation_distance = sun_sensor_coord.elevation - current_beam.vertical_angle();
                    double angle_distance = sqrt(pow(azimuth_distance,2) + pow(elevation_distance,2));
                    //double detection_probability = -32.82 * pow(angle_distance,2) + 4.26 * angle_distance - 0.09663;    //from fit to measurement
                    double brightness_factor = 1.0;
                    if (sun.intensity < 30000) {
                        brightness_factor = 0.0000333*sun.intensity;   //todo: put in profile
                    }
                    double detection_probability = brightness_factor*(-598.28 * pow(angle_distance-0.05298,2) + 0.281);    //from fit to measurement //todo: put in profile

                    auto random_var =  (float)uniform_distrib(generator);
                    if (random_var > (1-detection_probability)) {
                        double random_var_distance = uniform_distrib(generator);
                        double distance = profile.min_range + (profile.max_range - profile.min_range) * random_var_distance;
                        osi3::Spherical3d blinding_detection_sensor_sph;
                        blinding_detection_sensor_sph.set_azimuth(current_beam.horizontal_angle());
                        blinding_detection_sensor_sph.set_elevation(current_beam.vertical_angle());
                        blinding_detection_sensor_sph.set_distance(distance);
                        double intensity = 63.0 / 255.0 * 100.0;
                        if (distance <= 20) {
                            intensity = 1.0 / 255.0 * 100.0;
                        } else if  (distance > 20 && distance <= 60) {
                            intensity = round(0.475 * distance - 8.5) / 255.0 * 100.0;
                        } else if (distance > 60 && distance <= 80) {
                            intensity = round(2.15 * distance - 109) / 255.0 * 100.0;
                        }
                        if (existing_detection_idx.at(emitted_signal_idx) == -1) {
                            add_new_detection(current_sensor, blinding_detection_sensor_sph, intensity);
                        }
                    }
                }
            }
            lidar_frontend_idx++;
        }
    }
}

bool DetectionEnvironmentalEffects::is_detection_above_ground(osi3::SensorData &sensor_data, const osi3::Spherical3d &position_sensor_coord, const TF::EgoData &ego_data, int sensor_idx){
    double sensor_height_over_ground;
    if(sensor_data.sensor_view(sensor_idx).lidar_sensor_view_size() > 0 && sensor_data.sensor_view(sensor_idx).lidar_sensor_view(0).view_configuration().has_mounting_position()) {
        sensor_height_over_ground = sensor_data.sensor_view(sensor_idx).lidar_sensor_view(0).view_configuration().mounting_position().position().z() + ego_data.ego_base.position().z() + ego_data.ego_vehicle_attributes.bbcenter_to_rear().z();
    } else {
        sensor_height_over_ground = profile.sensor_view_configuration.lidar_sensor_view_configuration(0).mounting_position().position().z() + ego_data.ego_base.position().z() + ego_data.ego_vehicle_attributes.bbcenter_to_rear().z();
    }
    double z_coord_of_environmental_detection = sin(position_sensor_coord.elevation()) * position_sensor_coord.distance();
    if (z_coord_of_environmental_detection < -sensor_height_over_ground){
        return false;
    } else {
        return true;
    }
}

void DetectionEnvironmentalEffects::add_attenuated_existing_detection(std::vector<double> &detection_intensities, std::vector<double> &detection_distances, std::vector<int> existing_detection_idx,
                                                                      int emitted_signal_idx, const osi3::LidarDetection &existing_detection, double attenuation, double attenuation_distance) {
    if (existing_detection_idx.at(emitted_signal_idx) != -1) {
        if (attenuation_distance > 0.0) {
            double attenuation_factor = exp(-2.0 * attenuation * attenuation_distance);
            float receiver_aperture_area = M_PI * pow(profile.receiver_aperture_diameter_m,2) / 4.0;
            float emitted_signal_strength_mW = pow(10.0,profile.max_emitted_signal_strength_in_dBm/10.0);
            double range = existing_detection.position().distance();
            double attenuated_power_mW = attenuation_factor * receiver_aperture_area * emitted_signal_strength_mW * existing_detection.intensity()/100.0*255.0/100.0 / M_PI / pow(range,2);
            double new_intensity = existing_detection.intensity() * attenuation_factor;
            std::random_device generator;
            //std::normal_distribution<double> noise_distribution(attenuated_power_mW, (30.0*pow(10,-9)));
            std::normal_distribution<double> noise_distribution(attenuated_power_mW, (0.285*pow(10,-6))); //todo: put to profile
            double attenuated_power_noise_mW = noise_distribution(generator);
            double attenuated_power_dBm = 10 * log10(attenuated_power_noise_mW);

            double threshold = profile.detection_sensing_parameters.signal_strength_threshold_in_dBm;
            double thres_distance_m = 30.0;     //todo: put to profile
            if (profile.detection_sensing_parameters.range_comp_threshold) {
                if (range > thres_distance_m) {
                    double threshold_mW = pow(10.0,threshold/10.0) * pow(thres_distance_m,2) / pow(range,2);
                    threshold = 10 * std::log10(threshold_mW);
                }
            }
            if (attenuated_power_dBm > threshold) {
                //detection_intensities.emplace_back(existing_detection.intensity());
                detection_intensities.emplace_back(new_intensity);  //todo:set flag in profile if intensity should be reduced
                detection_distances.emplace_back(existing_detection.position().distance());
            }
        } else {
            detection_intensities.emplace_back(existing_detection.intensity());
            detection_distances.emplace_back(existing_detection.position().distance());
        }
    }
}

void DetectionEnvironmentalEffects::add_max_detection(std::vector<double> &detection_intensities, std::vector<double> &detection_distances,osi3::LidarDetectionData *current_sensor, const osi3::SpatialSignalStrength& current_beam){
    if (!detection_intensities.empty()) {
        std::vector<double> power_equivalent;
        for (int detection_idx = 0; detection_idx < detection_intensities.size(); detection_idx++) {
            power_equivalent.emplace_back(detection_intensities.at(detection_idx)/pow(detection_distances.at(detection_idx),2));
        }
        size_t maxElementIndex = std::max_element(power_equivalent.begin(),power_equivalent.end()) - power_equivalent.begin();
        if (detection_distances.at(maxElementIndex) < profile.max_range) {
            osi3::Spherical3d spray_detection_sensor_sph;
            spray_detection_sensor_sph.set_distance(detection_distances.at(maxElementIndex));
            spray_detection_sensor_sph.set_azimuth(current_beam.horizontal_angle());
            spray_detection_sensor_sph.set_elevation(current_beam.vertical_angle());
            add_new_detection(current_sensor, spray_detection_sensor_sph, detection_intensities.at(maxElementIndex));
        }
    }
}

void DetectionEnvironmentalEffects::add_new_detection(osi3::LidarDetectionData *current_sensor, const osi3::Spherical3d &position_sensor_coord, double intensity){
    auto detection = current_sensor->add_detection();
    detection->set_intensity(intensity);
    detection->mutable_position()->set_distance(position_sensor_coord.distance());
    detection->mutable_position()->set_azimuth(position_sensor_coord.azimuth());
    detection->mutable_position()->set_elevation(position_sensor_coord.elevation());
}

//taken from https://alexsm.com/cpp-closest-lower-bound/ by Oleksandr Semeniuta
long search_closest(const std::vector<float>& sorted_array, double x) {

    auto iter_geq = std::lower_bound(
            sorted_array.begin(),
            sorted_array.end(),
            x
    );

    if (iter_geq == sorted_array.begin()) {
        return 0;
    }

    double a = *(iter_geq - 1);
    double b = *(iter_geq);

    if (fabs(x - a) < fabs(x - b)) {
        return iter_geq - sorted_array.begin() - 1;
    }

    return iter_geq - sorted_array.begin();

}