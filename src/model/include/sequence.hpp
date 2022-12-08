//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef OSMPSENSORFRAMEWORK_SEQUENCE_HPP
#define OSMPSENSORFRAMEWORK_SEQUENCE_HPP

#include "strategy.hpp"
#include <vector>
#include <memory>

namespace model {

    class Sequence : public Strategy {
    public:
        Sequence(const Profile &profile, const Log &log, const Alert &alert);

        void apply(osi3::SensorData &data) override;

        void set_switch_for_csv_output(bool switch_for_csv_output);
        void set_switch_for_pcd_output(bool switch_for_pcd_output);
        void set_switch_for_bin_output(bool switch_for_bin_output);
        void set_switch_for_ros_output(bool switch_for_ros_output);

    private:
        std::vector<std::unique_ptr<Strategy>> strategies;
        std::vector<std::unique_ptr<Strategy>> csv_output_strategies;
        std::vector<std::unique_ptr<Strategy>> pcd_output_strategies;
        std::vector<std::unique_ptr<Strategy>> bin_output_strategies;
        std::vector<std::unique_ptr<Strategy>> ros_output_strategies;

        bool csv_output_enabled;
        bool pcd_output_enabled;
        bool bin_output_enabled;
        bool ros_output_enabled;
    };

}

#endif //OSMPSENSORFRAMEWORK_SEQUENCE_HPP
