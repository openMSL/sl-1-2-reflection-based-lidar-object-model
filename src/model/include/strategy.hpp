//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef SENSOR_MODEL_FMU_STRATEGY_HPP
#define SENSOR_MODEL_FMU_STRATEGY_HPP

#include <functional>
#include <model/profiles/profile.hpp>

namespace model {

    typedef std::function<void(const std::string &)> Log;

    typedef std::function<void(const std::string &)> Alert;

    using profile::Profile;

    class Strategy {
    public:
        Strategy(const Profile &profile, const Log &log, const Alert &alert) : profile(profile), log(log), alert(alert) {
        }

        virtual ~Strategy() = default;

        Strategy(const Strategy &) = delete;

        Strategy &operator=(const Strategy &) = delete;

        virtual void apply(osi3::SensorData &) = 0;

    protected:
        const Profile &profile;
        const Log &log;
        const Alert &alert;
    };

}

#endif //SENSOR_MODEL_FMU_STRATEGY_HPP
