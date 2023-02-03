//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef DETECTION_ENVIRONMENTAL_EFFECTS_STRATEGY_HPP
#define DETECTION_ENVIRONMENTAL_EFFECTS_STRATEGY_HPP

#include <model/include/strategy.hpp>

#include "../../transformation-functions/TransformationFunctions.hpp"

using namespace osi3;

namespace model
{

class DetectionEnvironmentalEffects : public Strategy
{
  public:
    //// Main Functions
    using Strategy::Strategy;
    void apply(SensorData&) override;
    struct SprayVolume
    {
        BaseMoving volume;
        const MovingObject* corresponding_object;
    };

    struct SprayCluster
    {
        osi3::Vector3d position_global;
        float radius = 0.0;
        float age_in_s = 0.0;
        float time_constant = 0.0;
        double object_velocity_in_m_s = 0.0;
        osi3::Vector3d velocity_in_m_s;
        double min_azimuth_rad = 0.0;
        double max_azimuth_rad = 0.0;
        double dist_to_sensor = 0.0;
        double drag_factor = 0.0;
        bool sensor_in_spray_volume = false;
    };

  private:
    std::vector<SprayCluster> spray_cluster_global = {};

    void add_hydrometeor_detections(osi3::SensorData& sensor_data, osi3::LidarDetectionData* current_sensor, int sensor_idx, const TF::EgoData& ego_data);
    void add_spray_detections(osi3::SensorData& sensor_data, osi3::LidarDetectionData* current_sensor, int sensor_idx, const TF::EgoData& ego_data);
    void add_sun_blinding_detections(SensorData& sensor_data, osi3::LidarDetectionData* current_sensor, const TF::EgoData& ego_data);

    std::vector<DetectionEnvironmentalEffects::SprayVolume> define_spray_volumes(osi3::SensorData& sensor_data, const TF::EgoData& ego_data, osi3::MountingPosition& mounting_pose);
    void append_spray_cluster(osi3::SensorData& sensor_data, const TF::EgoData& ego_data, osi3::MountingPosition& mounting_pose);
    static void add_new_detection(osi3::LidarDetectionData* current_sensor, const osi3::Spherical3d& position_sensor_coord, double intensity);
    bool is_detection_above_ground(osi3::SensorData& sensor_data, const osi3::Spherical3d& position_sensor_coord, const TF::EgoData& ego_data, int sensor_idx);
    void add_attenuated_existing_detection(std::vector<double>& detection_intensities,
                                           std::vector<double>& detection_distances,
                                           std::vector<int> existing_detection_idx,
                                           int emitted_signal_idx,
                                           const osi3::LidarDetection& existing_detection,
                                           double attenuation,
                                           double attenuation_distance);
    void add_max_detection(std::vector<double>& detection_intensities,
                           std::vector<double>& detection_distances,
                           osi3::LidarDetectionData* current_sensor,
                           const osi3::SpatialSignalStrength& current_beam);
    static void get_min_max_azimuth_of_cluster(SprayCluster& cluster, const osi3::MountingPosition& mounting_pose, const TF::EgoData& ego_data);
    static bool check_if_sensor_in_cluster_volume(SprayCluster& cluster, const osi3::MountingPosition& mounting_pose, const TF::EgoData& ego_data);
    static std::vector<double> intersection_with_sphere(const osi3::Vector3d& center, float radius, double azimuth, double elevation);

    void simulate_wet_pavement(osi3::SensorData& sensor_data, const TF::EgoData& ego_data, LidarDetection& existing_detection, int sensor_idx, double water_film_height);

    static LidarDetectionData get_beam_indices(SensorData& sensor_data, std::vector<int>& existing_detection_idx);

    std::vector<int> update_spray_cluster(SensorData& sensor_data, const TF::EgoData& ego_data, MountingPosition& mounting_pose);

    int is_pavement = 0;

    bool is_fog = false;
    bool is_rain = false;
    bool is_snow = false;
    bool is_sun = false;
    std::vector<float> weather_sequence = {};
};
}  // namespace model

long search_closest(const std::vector<float>& sorted_array, double x);

#endif  // DETECTION_ENVIRONMENTAL_EFFECTS_STRATEGY_HPP
