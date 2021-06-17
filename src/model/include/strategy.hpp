//
// Copyright Institute of Automotive Engineering
// of Technical University of Darmstadt 2020.
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
