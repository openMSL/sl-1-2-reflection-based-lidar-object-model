//
// Copyright 2018 PMSF IT Consulting - Pierre R. Mai
// Copyright 2022 Technical University of Darmstadt - FZD
// SPDX-License-Identifier: MPL-2.0
//

#ifndef FMU_SHARED_OBJECT
#define FMI2_FUNCTION_PREFIX FrameworkPackaging_
#endif

#include "fmi2Functions.h"

/*
 * Logging Control
 *
 * Logging is controlled via three definitions:
 *
 * - If PRIVATE_LOG_PATH_MODEL is defined it gives the name of a file
 *   that is to be used as a private log file.
 * - If PUBLIC_LOGGING_MODEL is defined then we will (also) log to
 *   the FMI logging facility where appropriate.
 * - If VERBOSE_FMI_LOGGING_MODEL is defined then logging of basic
 *   FMI calls is enabled, which can get very verbose.
 */

/*
 * Variable Definitions
 *
 * Define FMI_*_LAST_IDX to the zero-based index of the last variable
 * of the given type (0 if no variables of the type exist).  This
 * ensures proper space allocation, initialisation and handling of
 * the given variables in the template code.  Optionally you can
 * define FMI_TYPENAME_VARNAME_IDX definitions (e.g. FMI_REAL_MYVAR_IDX)
 * to refer to individual variables inside your code, or for example
 * FMI_REAL_MYARRAY_OFFSET and FMI_REAL_MYARRAY_SIZE definitions for
 * array variables.
 */

/* Boolean Variables */
#define FMI_BOOLEAN_SWITCH_FOR_CSV_OUTPUT_IDX 0
#define FMI_BOOLEAN_SWITCH_FOR_PCD_OUTPUT_IDX 1
#define FMI_BOOLEAN_SWITCH_FOR_BIN_OUTPUT_IDX 2
#define FMI_BOOLEAN_SWITCH_FOR_ROS_OUTPUT_IDX 3
#define FMI_BOOLEAN_LAST_IDX FMI_BOOLEAN_SWITCH_FOR_ROS_OUTPUT_IDX
#define FMI_BOOLEAN_VARS (FMI_BOOLEAN_LAST_IDX + 1)

/* Integer Variables */
#define FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX 0
#define FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX 1
#define FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX 2
#define FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX 3
#define FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX 4
#define FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX 5
#define FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX 6
#define FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX 7
#define FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX 8
#define FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX 9
#define FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX 10
#define FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX 11
#define FMI_INTEGER_LAST_IDX FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX
#define FMI_INTEGER_VARS (FMI_INTEGER_LAST_IDX + 1)

/* Real Variables */
#define FMI_REAL_VARS 0

/* String Variables */
#define FMI_STRING_PROFILE_IDX 0
#define FMI_STRING_LAST_IDX FMI_STRING_PROFILE_IDX
#define FMI_STRING_VARS (FMI_STRING_LAST_IDX + 1)

#undef min
#undef max

#include <cstdarg>
#include <fstream>
#include <set>
#include <string>
#include <utility>

#include <model/include/sequence.hpp>

#include "osi_sensordata.pb.h"
#include "osi_sensorview.pb.h"

/* FMU Class */
class CFrameworkPackaging
{
  public:
    /* FMI2 Interface mapped to C++ */
    CFrameworkPackaging(fmi2String theinstanceName,
                        fmi2Type thefmuType,
                        fmi2String thefmuGUID,
                        fmi2String thefmuResourceLocation,
                        const fmi2CallbackFunctions* thefunctions,
                        fmi2Boolean thevisible,
                        fmi2Boolean theloggingOn);

    ~CFrameworkPackaging();

    fmi2Status SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[]);

    static fmi2Component Instantiate(fmi2String instanceName,
                                     fmi2Type fmuType,
                                     fmi2String fmuGUID,
                                     fmi2String fmuResourceLocation,
                                     const fmi2CallbackFunctions* functions,
                                     fmi2Boolean visible,
                                     fmi2Boolean loggingOn);

    fmi2Status SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime);

    fmi2Status EnterInitializationMode();

    fmi2Status ExitInitializationMode();

    fmi2Status DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component);

    fmi2Status Terminate();

    fmi2Status Reset();

    void FreeInstance();

    fmi2Status GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]);

    fmi2Status GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]);

    fmi2Status GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]);

    fmi2Status GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[]);

    fmi2Status SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]);

    fmi2Status SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]);

    fmi2Status SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]);

    fmi2Status SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]);

  protected:
    /* Internal Implementation */
    fmi2Status doInit();

    static fmi2Status doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime, fmi2Boolean stopTimeDefined, fmi2Real stopTime);

    fmi2Status doEnterInitializationMode();

    fmi2Status doExitInitializationMode();

    fmi2Status doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize, fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component);

    static fmi2Status doTerm();

    static void doFree();

  protected:
    /* Private File-based Logging just for Debugging */
#ifdef PRIVATE_LOG_PATH_MODEL
    static std::ofstream private_log_file;
#endif

    static void fmi_verbose_log_global(const char* format, ...)
    {
#ifdef VERBOSE_FMI_LOGGING_MODEL
#ifdef PRIVATE_LOG_PATH_MODEL
        va_list ap;
        va_start(ap, format);
        char buffer[1024];
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH_MODEL, std::ios::out | std::ios::app);
        if (private_log_file.is_open())
        {
#ifdef _WIN32
            vsnprintf_s(buffer, 1024, format, ap);
#else
            vsnprintf(buffer, 1024, format, ap);
#endif
            private_log_file << "FrameworkPackaging"
                             << "::Global:FMI: " << buffer << std::endl;
            private_log_file.flush();
        }
#endif
#endif
    }

    void internal_log(const char* category, const fmi2Status status, const char* format, va_list arg)
    {
#if defined(PRIVATE_LOG_PATH_MODEL) || defined(PUBLIC_LOGGING_MODEL)
        char buffer[1024];
#ifdef _WIN32
        vsnprintf_s(buffer, 1024, format, arg);
#else
        vsnprintf(buffer, 1024, format, arg);
#endif
#ifdef PRIVATE_LOG_PATH_MODEL
        if (!private_log_file.is_open())
            private_log_file.open(PRIVATE_LOG_PATH_MODEL, std::ios::out | std::ios::app);
        if (private_log_file.is_open())
        {
            private_log_file << "FrameworkPackaging"
                             << "::" << instanceName << "<" << ((void*)this) << ">:" << category << ": " << buffer << std::endl;
            private_log_file.flush();
        }
#endif
#ifdef PUBLIC_LOGGING_MODEL
        if (loggingOn && loggingCategories.count(category))
            functions.logger(functions.componentEnvironment, instanceName.c_str(), status, category, buffer);
#endif
#endif
    }

    void fmi_verbose_log(const char* format, ...)
    {
#if defined(VERBOSE_FMI_LOGGING_MODEL) && (defined(PRIVATE_LOG_PATH_MODEL) || defined(PUBLIC_LOGGING_MODEL))
        va_list ap;
        va_start(ap, format);
        internal_log("FMI", format, ap);
        va_end(ap);
#endif
    }

    /* Normal Logging */
    void normal_log(const char* category, const char* format, ...)
    {
#if defined(PRIVATE_LOG_PATH_MODEL) || defined(PUBLIC_LOGGING_MODEL)
        va_list ap;
        va_start(ap, format);
        internal_log(category, fmi2Status::fmi2OK, format, ap);
        va_end(ap);
#endif
    }

    void error_log(const char* category, const char* format, ...)
    {
#if defined(PRIVATE_LOG_PATH_MODEL) || defined(PUBLIC_LOGGING_MODEL)
        va_list ap;
        va_start(ap, format);
        internal_log(category, fmi2Status::fmi2Error, format, ap);
        va_end(ap);
#endif
    }

  protected:
    /* Members */
    std::string instanceName;
    fmi2Type fmuType;
    std::string fmuGUID;
    std::string fmuResourceLocation;
    bool visible;
    bool loggingOn;
    std::set<std::string> loggingCategories;
    fmi2CallbackFunctions functions;
#if FMI_BOOLEAN_VARS
    fmi2Boolean boolean_vars[FMI_BOOLEAN_VARS]{};
#else
    fmi2Boolean boolean_vars[1]{};
#endif
#if FMI_INTEGER_VARS
    fmi2Integer integer_vars[FMI_INTEGER_VARS]{};
#else
    fmi2Integer integer_vars[1]{};
#endif
#if FMI_REAL_VARS
    fmi2Real real_vars[FMI_REAL_VARS]{};
#else
    fmi2Real real_vars[1]{};
#endif
#if FMI_STRING_VARS
    std::string string_vars[FMI_STRING_VARS]{};
#else
    std::string string_vars[1]{};
#endif
    bool simulation_started;
    std::string currentOutputBuffer;
    std::string lastOutputBuffer;
    std::string currentConfigRequestBuffer;
    std::string lastConfigRequestBuffer;

    /* Simple Accessors */
    std::string fmi_profile()
    {
        return string_vars[FMI_STRING_PROFILE_IDX];
    }

    void set_fmi_profile(std::string value)
    {
        string_vars[FMI_STRING_PROFILE_IDX] = std::move(value);
    }

    bool fmi_switch_for_csv_output()
    {
        return boolean_vars[FMI_BOOLEAN_SWITCH_FOR_CSV_OUTPUT_IDX];
    }

    void set_fmi_switch_for_csv_output(bool value)
    {
        boolean_vars[FMI_BOOLEAN_SWITCH_FOR_CSV_OUTPUT_IDX] = value;
    }

    bool fmi_switch_for_pcd_output()
    {
        return boolean_vars[FMI_BOOLEAN_SWITCH_FOR_PCD_OUTPUT_IDX];
    }

    void set_fmi_switch_for_pcd_output(bool value)
    {
        boolean_vars[FMI_BOOLEAN_SWITCH_FOR_PCD_OUTPUT_IDX] = value;
    }

    bool fmi_switch_for_bin_output()
    {
        return boolean_vars[FMI_BOOLEAN_SWITCH_FOR_BIN_OUTPUT_IDX];
    }

    void set_fmi_switch_for_bin_output(bool value)
    {
        boolean_vars[FMI_BOOLEAN_SWITCH_FOR_BIN_OUTPUT_IDX] = value;
    }

    bool fmi_switch_for_ros_output()
    {
        return boolean_vars[FMI_BOOLEAN_SWITCH_FOR_ROS_OUTPUT_IDX];
    }

    void set_fmi_switch_for_ros_output(bool value)
    {
        boolean_vars[FMI_BOOLEAN_SWITCH_FOR_ROS_OUTPUT_IDX] = value;
    }

    /* Protocol Buffer Accessors */
    bool get_fmi_sensor_view_config(osi3::SensorViewConfiguration& data);

    void set_fmi_sensor_view_config_request(const osi3::SensorViewConfiguration& data);

    [[maybe_unused]] void reset_fmi_sensor_view_config_request();

    bool get_fmi_sensor_view_in(osi3::SensorView& data);

    void set_fmi_sensor_data_out(const osi3::SensorData& data);

    void reset_fmi_sensor_data_out();

    /* Refreshing of Calculated Parameters */
    void refresh_fmi_sensor_view_config_request();

  private:
    void load_profile_or_complain(const std::string& name);
    bool try_load_profile(const std::string& name);
    void set_output_switches(const bool csv_switch, const bool pcd_switch, const bool bin_switch, const bool ros_switch);

    bool csv_output_enabled = false;
    bool pcd_output_enabled = false;
    bool bin_output_enabled = false;
    bool ros_output_enabled = false;

    bool is_profile_loaded = false;
    model::profile::Profile profile;
    model::Sequence sequence_of_strategies;
    model::Log message_sink;
    model::Alert alert_sink;
};
