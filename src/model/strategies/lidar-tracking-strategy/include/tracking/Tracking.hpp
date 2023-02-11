//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef TRACKING_STRATEGY_HPP
#define TRACKING_STRATEGY_HPP

#include <list>
#include <vector>

#include <model/include/strategy.hpp>

#include "../../transformation-functions/TransformationFunctions.hpp"

using namespace osi3;

namespace model
{
class Tracking : public Strategy
{

  private:
    //// Data from the last time steps
    struct History
    {
        std::list<SensorData> sensor_data;
        std::list<std::vector<bool>> object_was_moving;
        std::list<std::vector<Vector3d>> velocity_reference_point;
        std::list<std::vector<size_t>> pointcloud_segment_size;
    } history;

    //// Data of the current time step
    struct Data
    {
        SensorData sensor_data;
        std::vector<Vector3d> pcl_segment_points_in_ego_coordinates = {};
        Dimension3d pcl_segment_dimension;
        Vector3d pcl_segment_position_in_object;
        double delta_t = 0.0;
        std::vector<bool> object_is_moving = {};
    };

  public:
    using Strategy::Strategy;
    void apply(SensorData&) override;

    void calculate_velocity_as_derivation_of_position(Tracking::Data& data_of_current_time_step,
                                                      DetectedMovingObject* current_moving_object,
                                                      bool object_contained_in_history) const;

    void set_object_dimension_with_tracking(const Dimension3d& current_dimension, Dimension3d* new_dimension, bool object_contained_in_history, int historical_object_no) const;

    static void transform_gt_object_to_ego_coordinate_system(const MovingObject& current_gt_object, DetectedMovingObject* current_moving_object, const TF::EgoData& ego_data);

    static void get_pcl_segment_of_current_object(const LogicalDetectionData& logical_detection_data, Tracking::Data& data_of_current_time_step, size_t gt_object_id);

    static void calculate_object_dimension_and_position_in_object_from_pcl_segment(Tracking::Data& data_of_current_time_step,
                                                                                   const MovingObject& current_gt_object,
                                                                                   const TF::EgoData& ego_data);

    void update_history(Tracking::Data& data_of_current_time_step, const SensorData& sensor_data);

    void determine_object_type(DetectedMovingObject_CandidateMovingObject* current_moving_object_candidate, const MovingObject& current_gt_object);

    static void set_rcs(DetectedMovingObject* current_moving_object);

    void calculate_delta_t_of_current_time_step(Tracking::Data& data_of_current_time_step);

    void find_object_in_history(bool& object_contained_in_history, int& historical_object_no, DetectedMovingObject* current_moving_object, bool& object_tracked_in_history);

    static void limit_existence_probability_to_0_and_1(DetectedMovingObject* current_moving_object);

    static void calculate_dimension_with_lower_bounds(const Dimension3d& lower_bounds_dimension,
                                                      const Dimension3d& current_pcl_segment_dimension,
                                                      Dimension3d* new_pcl_segment_dimension,
                                                      const Vector3d& current_pcl_segment_position_in_object,
                                                      Vector3d* new_pcl_segment_position_in_object);

    static void eliminate_ground_clearance(const Dimension3d& current_dimension,
                                           const Dimension3d& current_gt_object_dimension,
                                           Dimension3d* new_dimension,
                                           Vector3d& current_position_in_object,
                                           Vector3d* new_position_in_object);

    [[maybe_unused]] static void limit_to_gt_height(Vector3d& pcl_segment_position_in_object,
                                                    const Dimension3d& current_dimension,
                                                    const Dimension3d& current_gt_object_dimension,
                                                    Dimension3d* new_dimension);

    void write_data_back_to_osi(SensorData& sensor_data, Data& data_of_current_time_step);

    void calculate_dimension_and_position_from_pcl(const MovingObject& current_gt_object,
                                                   Data& data_of_current_time_step,
                                                   DetectedMovingObject* current_moving_object,
                                                   bool object_contained_in_history,
                                                   int historical_object_no,
                                                   const TF::EgoData& ego_data);

    void continue_tracking_for_current_pcl_segment(Data& data_of_current_time_step,
                                                   DetectedMovingObject* current_moving_object,
                                                   const MovingObject& current_gt_object,
                                                   bool object_tracked_in_history,
                                                   int historical_object_no,
                                                   const TF::EgoData& ego_data);

    void start_tracking_for_current_pcl_segment(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, const MovingObject& current_gt_object);

    void calculate_dimension_and_position_from_history(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, int historical_object_no);

    void calculate_velocity_from_pcl(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, bool object_contained_in_history, int historical_object_no);

    void calculate_orientation_from_history(Tracking::Data& data_of_current_time_step, DetectedMovingObject* current_moving_object, int historical_object_no);

    static void calculate_orientation_from_pcl(Data& data_of_current_time_step, DetectedMovingObject* current_moving_object);

    static std::string get_type_string(const DetectedMovingObject_CandidateMovingObject* current_moving_object_candidate);

    static std::string get_vehicle_type_string(const DetectedMovingObject_CandidateMovingObject* current_moving_object_candidate);
};
}  // namespace model

#endif  // TRACKING_STRATEGY_HPP
