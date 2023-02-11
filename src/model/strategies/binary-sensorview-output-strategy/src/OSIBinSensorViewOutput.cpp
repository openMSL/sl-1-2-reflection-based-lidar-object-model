//
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif

#include "osibinsensorviewoutput/OSIBinSensorViewOutput.hpp"

#include <ctime>
#include <fstream>
#include <iostream>

#ifdef _WIN32
#include <direct.h>
#else
#include <sys/stat.h>
#endif

using namespace model;
using namespace osi3;

void model::OSIBinSensorViewOutput::apply(SensorData& sensor_data)
{
    log("Starting SensorView to .bin output");

    if (sensor_data.sensor_view_size() == 0)
    {
        log("No SensorView received for SensorView to .bin output");
        return;
    }

    if (sensor_data.sensor_view(0).has_global_ground_truth())
    {
        /// Create folder on first call
        if (first_call)
        {
#include <osibinsensorviewoutput/set_bin_file_path_sensorview.cpp>
            first_call = false;
        }
        write_data_to_bin(file_path_tracefile, sensor_data.sensor_view(0));
    }
}

void OSIBinSensorViewOutput::write_data_to_bin(const std::string& path, const SensorView& sensor_view)
{
    typedef unsigned int MessageSizeT;
    std::ofstream bin_file(path, std::ios::binary | std::ios_base::app);

    std::string osi_msg_string_only = sensor_view.SerializeAsString();
    MessageSizeT message_size = osi_msg_string_only.size();
    char character[4];
    memcpy(character, (char*)&message_size, sizeof(MessageSizeT));

    std::string osi_msg_string;
    osi_msg_string += character[0];
    osi_msg_string += character[1];
    osi_msg_string += character[2];
    osi_msg_string += character[3];
    osi_msg_string += osi_msg_string_only;

    bin_file.write(osi_msg_string.c_str(), osi_msg_string.length());
    bin_file.close();
    osi_msg_string.clear();
}
