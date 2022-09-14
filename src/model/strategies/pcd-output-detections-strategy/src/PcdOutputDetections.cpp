//
// Copyright Institute of Automotive Engineering
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

#include "pcdoutputdetections/PcdOutputDetections.hpp"
#include <fstream>
#include <iostream>
#include <ctime>

#ifdef _WIN32
    #include <math.h>
    #include <direct.h>
#else
    #include <cmath>
    #include <sys/stat.h>
#endif

using namespace model;
using namespace osi3;

void model::PcdOutputDetections::apply(SensorData &sensor_data) {
    log("Starting .pcd output for detections");
    
    if(sensor_data.sensor_view().size() == 0)
        return;

    auto time_nanos = sensor_data.sensor_view(0).global_ground_truth().timestamp().nanos();
    auto time_seconds = sensor_data.sensor_view(0).global_ground_truth().timestamp().seconds();
    double timestamp = (double)time_seconds + (double) time_nanos / 1000000000;
    
	if (!sensor_data.has_feature_data())
		return;

    size_t no_of_detections = 0;
    size_t no_of_sensors = 0;
    if (sensor_data.sensor_view(0).lidar_sensor_view().size() > 0) {
        for (int sensor_idx = 0; sensor_idx < sensor_data.feature_data().lidar_sensor_size(); sensor_idx++) {
            no_of_detections = no_of_detections + sensor_data.feature_data().lidar_sensor(sensor_idx).detection_size();
            no_of_sensors = sensor_data.feature_data().lidar_sensor_size();
        }
    } else if (sensor_data.sensor_view(0).radar_sensor_view().size() > 0) {
        for (int sensor_idx = 0; sensor_idx < sensor_data.feature_data().radar_sensor_size(); sensor_idx++) {
            no_of_detections = no_of_detections + sensor_data.feature_data().radar_sensor(sensor_idx).detection_size();
            no_of_sensors = sensor_data.feature_data().radar_sensor_size();
        }
    }
	if (no_of_detections == 0) {
		log("No detections for .pcd output at timestamp " + std::to_string(timestamp));
		return;
	}
            
    if (first_call) {
        #include <pcdoutputdetections/set_pcd_file_path_detections.cpp>
        first_call = false;
    }

    /// Header needed in every file
    std::string filename = "Detections_";
    filename.append(std::to_string(timestamp));
    filename.append(".pcd");
    #if defined(_WIN32)
        std::string path = path_string + "\\" + filename;
    #else
        std::string path = path_string + "/" + filename;
    #endif
    writePcdHeader(path, sensor_data, no_of_sensors);
        
    for (int sensor_idx = 0; sensor_idx < no_of_sensors; sensor_idx++) {
        /// Run through all detections
        if (sensor_data.sensor_view(0).lidar_sensor_view().size() > 0) {
            for (const auto &detection: sensor_data.feature_data().lidar_sensor(sensor_idx).detection()) {
                Vector3d point_cartesian_sensor;
                point_cartesian_sensor.set_x(detection.position().distance() * cos(detection.position().elevation()) * cos(detection.position().azimuth()));
                point_cartesian_sensor.set_y(detection.position().distance() * cos(detection.position().elevation()) * sin(detection.position().azimuth()));
                point_cartesian_sensor.set_z(detection.position().distance() * sin(detection.position().elevation()));
                auto intensity = float(detection.has_intensity() ? detection.intensity() : detection.echo_pulse_width());
                write2Pcd(path, point_cartesian_sensor.x(), point_cartesian_sensor.y(), point_cartesian_sensor.z(), intensity);
            }
        } else if (sensor_data.sensor_view(0).radar_sensor_view().size() > 0) {
            for (const auto &detection: sensor_data.feature_data().radar_sensor(sensor_idx).detection()) {
                Vector3d point_cartesian_sensor;
                point_cartesian_sensor.set_x(detection.position().distance() * cos(detection.position().elevation()) * cos(detection.position().azimuth()));
                point_cartesian_sensor.set_y(detection.position().distance() * cos(detection.position().elevation()) * sin(detection.position().azimuth()));
                point_cartesian_sensor.set_z(detection.position().distance() * sin(detection.position().elevation()));
                auto intensity = float(detection.rcs());
                write2Pcd(path, point_cartesian_sensor.x(), point_cartesian_sensor.y(), point_cartesian_sensor.z(), intensity);
            }
        }
    }
}

void PcdOutputDetections::writePcdHeader(const std::string& path, const SensorData& sensor_data, const size_t& no_of_sensors) {
	size_t no_of_points = 0;
    for (int sensor_idx = 0; sensor_idx < no_of_sensors; sensor_idx++) {
        if (sensor_data.sensor_view(0).lidar_sensor_view().size() > 0)
            no_of_points += sensor_data.feature_data().lidar_sensor(sensor_idx).detection().size();
        else if (sensor_data.sensor_view(0).radar_sensor_view().size() > 0)
            no_of_points += sensor_data.feature_data().radar_sensor(sensor_idx).detection().size();
    }

    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << "# .PCD v0.7 - Point Cloud Data file format" << std::endl;
    my_file << "VERSION " << "0.7" << std::endl;
    my_file << "FIELDS " << "x y z intensity" << std::endl;
    my_file << "SIZE " << "4 4 4 4" << std::endl;
    my_file << "TYPE " << "F F F F" << std::endl;
    my_file << "COUNT " << "1 1 1 1" << std::endl;
    my_file << "WIDTH " << no_of_points << std::endl;
    my_file << "HEIGHT " << "1" << std::endl;
    my_file << "VIEWPOINT " << "0 0 0 1 0 0 0" << std::endl;
    my_file << "POINTS " << no_of_points << std::endl;
    my_file << "DATA " << "ascii" << std::endl;
    my_file.close();
}

void PcdOutputDetections::write2Pcd(const std::string& path, double x, double y, double z, double intensity) {
    std::fstream my_file;
    my_file.open(path, std::ios::app);
    my_file << x << " " << y << " " << z << " " << intensity << std::endl;
    my_file.close();
}
