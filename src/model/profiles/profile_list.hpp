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

#ifndef OSMPSENSORFRAMEWORK_PROFILE_LIST_HPP
#define OSMPSENSORFRAMEWORK_PROFILE_LIST_HPP

#include <model/profiles/profile.hpp>
#include <string>

/* TODO add further profiles here */
#include <model/profiles/profile_SCALA_1.hpp>
#include <model/profiles/profile_Ibeo_LUX_2010.hpp>
#include <model/profiles/profile_VLP32.hpp>
#include <model/profiles/profile_VLP16.hpp>
#include <model/profiles/profile_Blickfeld.hpp>

bool CFrameworkPackaging::try_load_profile(const std::string &name) {
    /* TODO add further profile generators here */
    if (name == "SCALA_1") {
        profile = model::profile::SCALA_1::generate();
        return true;
    }
    if (name == "Ibeo_LUX_2010") {
        profile = model::profile::Ibeo_LUX_2010::generate();
        return true;
    }
    if (name == "VLP32") {
        profile = model::profile::VLP32::generate();
        return true;
    }
    if (name == "VLP16") {
        profile = model::profile::VLP16::generate();
        return true;
    }
    if (name == "Blickfeld") {
        profile = model::profile::Blickfeld::generate();
        return true;
    }

    return false;
}

#endif //OSMPSENSORFRAMEWORK_PROFILE_LIST_HPP
