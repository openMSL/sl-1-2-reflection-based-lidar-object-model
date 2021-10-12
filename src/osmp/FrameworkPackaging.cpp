/* FMU Framework for FMI 2.0 Co-Simulation FMUs using FZD strategies
 *
 * (C) 2020 Technical University of Darmstadt
 *
 * based on OSMPDummySensor.cpp from
 *
 * PMSF FMU Framework for FMI 2.0 Co-Simulation FMUs
 *
 * (C) 2016 -- 2018 PMSF IT Consulting Pierre R. Mai
 *
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include "FrameworkPackaging.h"

/*
 * Debug Breaks
 *
 * If you define DEBUG_BREAKS the FMU will automatically break
 * into an attached Debugger on all major computation functions.
 * Note that the FMU is likely to break all environments if no
 * Debugger is actually attached when the breaks are triggered.
 */
#if defined(DEBUG_BREAKS) && !defined(NDEBUG)
#if defined(__has_builtin) && !defined(__ibmxl__)
#if __has_builtin(__builtin_debugtrap)
#define DEBUGBREAK() __builtin_debugtrap()
#elif __has_builtin(__debugbreak)
#define DEBUGBREAK() __debugbreak()
#endif
#endif
#if !defined(DEBUGBREAK)
#if defined(_MSC_VER) || defined(__INTEL_COMPILER)
#include <intrin.h>
#define DEBUGBREAK() __debugbreak()
#else
#include <signal.h>
#if defined(SIGTRAP)
#define DEBUGBREAK() raise(SIGTRAP)
#else
#define DEBUGBREAK() raise(SIGABRT)
#endif
#endif
#endif
#else
#define DEBUGBREAK()
#endif

#include <string>
#include <algorithm>
#include <cstdint>

#ifdef PRIVATE_LOG_PATH
ofstream CFrameworkPackaging::private_log_file;
#endif

/*
 * ProtocolBuffer Accessors
 */

void *decode_integer_to_pointer(fmi2Integer hi, fmi2Integer lo) {
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr{};
    myaddr.base.lo = lo;
    myaddr.base.hi = hi;
    return reinterpret_cast<void *>(myaddr.address);
#elif PTRDIFF_MAX == INT32_MAX
    return reinterpret_cast<void*>(lo);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

void encode_pointer_to_integer(const void *ptr, fmi2Integer &hi, fmi2Integer &lo) {
#if PTRDIFF_MAX == INT64_MAX
    union addrconv {
        struct {
            int lo;
            int hi;
        } base;
        unsigned long long address;
    } myaddr{};
    myaddr.address = reinterpret_cast<unsigned long long>(ptr);
    hi = myaddr.base.hi;
    lo = myaddr.base.lo;
#elif PTRDIFF_MAX == INT32_MAX
    hi=0;
    lo=reinterpret_cast<int>(ptr);
#else
#error "Cannot determine 32bit or 64bit environment!"
#endif
}

bool CFrameworkPackaging::get_fmi_sensor_view_config(osi3::SensorViewConfiguration &data) {
    if (integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX] > 0) {
        void *buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX],
                                                 integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX]);
        normal_log("OSMP", "SensorViewConfiguration: Got %08X %08X, reading from %p ...", integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASEHI_IDX],
                   integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_BASELO_IDX], buffer);
        data.ParseFromArray(buffer, integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_SIZE_IDX]);
        return true;
    } else {
        return false;
    }
}

void CFrameworkPackaging::set_fmi_sensor_view_config_request(const osi3::SensorViewConfiguration &data) {
    data.SerializeToString(&currentConfigRequestBuffer);
    encode_pointer_to_integer(currentConfigRequestBuffer.data(),
                              integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX],
                              integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX] = (fmi2Integer) currentConfigRequestBuffer.length();
    normal_log("OSMP", "SensorViewConfiguration: Providing %08X %08X, writing from %p ...",
               integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX],
               integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX], currentConfigRequestBuffer.data());
    swap(currentConfigRequestBuffer, lastConfigRequestBuffer);
}

void CFrameworkPackaging::reset_fmi_sensor_view_config_request() {
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX] = 0;
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX] = 0;
    integer_vars[FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX] = 0;
}

bool CFrameworkPackaging::get_fmi_sensor_view_in(osi3::SensorView &data) {
    if (integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX] > 0) {
        void *buffer = decode_integer_to_pointer(integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],
                                                 integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX]);
        normal_log("OSMP", "----------------------------------------------------------------------------------------------------");
        normal_log("OSMP", "Got %08X %08X, reading from %p ...", integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASEHI_IDX],
                   integer_vars[FMI_INTEGER_SENSORVIEW_IN_BASELO_IDX], buffer);
        data.ParseFromArray(buffer, integer_vars[FMI_INTEGER_SENSORVIEW_IN_SIZE_IDX]);
        return true;
    } else {
        return false;
    }
}

void CFrameworkPackaging::set_fmi_sensor_data_out(const osi3::SensorData &data) {
    data.SerializeToString(&currentOutputBuffer);
    encode_pointer_to_integer(currentOutputBuffer.data(), integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],
                              integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX]);
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = (fmi2Integer) currentOutputBuffer.length();
    normal_log("OSMP", "Providing %08X %08X, writing from %p ...", integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX],
               integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX], currentOutputBuffer.data());
    normal_log("OSMP", "----------------------------------------------------------------------------------------------------");
    swap(currentOutputBuffer, lastOutputBuffer);
}

void CFrameworkPackaging::reset_fmi_sensor_data_out() {
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_SIZE_IDX] = 0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASEHI_IDX] = 0;
    integer_vars[FMI_INTEGER_SENSORDATA_OUT_BASELO_IDX] = 0;
}

void CFrameworkPackaging::load_profile_or_complain(const std::string &name) {
    if (try_load_profile(name)) {
        is_profile_loaded = true;
        normal_log("OSMP", "Loaded profile: '%s'", name.c_str());
    } else {
        error_log("OSMP", "Could not load profile '%s'", name.c_str());
    }
}

void CFrameworkPackaging::refresh_fmi_sensor_view_config_request() {
    osi3::SensorViewConfiguration config;
    if (get_fmi_sensor_view_config(config))
        set_fmi_sensor_view_config_request(config);
    else {
        load_profile_or_complain(fmi_profile());
        config.Clear();
        config.MergeFrom(profile.sensor_view_configuration);
        set_fmi_sensor_view_config_request(config);
    }
}

/*
 * Actual Core Content
 */

fmi2Status CFrameworkPackaging::doInit() {
    DEBUGBREAK()

    /* Booleans */
    for (int & boolean_var : boolean_vars)
        boolean_var = fmi2False;

    /* Integers */
    for (int & integer_var : integer_vars)
        integer_var = 0;

    /* Reals */
    for (double & real_var : real_vars)
        real_var = 0.0;

    /* Strings */
    for (auto & string_var : string_vars)
        string_var = "";

    return fmi2OK;
}

fmi2Status CFrameworkPackaging::doStart(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime,
                                    fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    DEBUGBREAK()

    return fmi2OK;
}

fmi2Status CFrameworkPackaging::doEnterInitializationMode() {
    DEBUGBREAK()

    /*
     * Pre-initialisation of the profile in case the master does not care about the model description.
     * If this is not the case, the value is overwritten anyway.
     */
    #include <model/profiles/init_profile.hpp>

    return fmi2OK;
}

fmi2Status CFrameworkPackaging::doExitInitializationMode() {
    DEBUGBREAK()

    /*
     * The profile was only preloaded when a request was made to configure the sensor view.
     */
    if (!is_profile_loaded) {
		normal_log("OSMP", "Config request was not queried, therefore default profile's config will be loaded!");
        load_profile_or_complain(fmi_profile());
    }

    osi3::SensorViewConfiguration config;
    if (!get_fmi_sensor_view_config(config))
        normal_log("OSI",
                   "Received no valid SensorViewConfiguration from Simulation Environment, assuming everything checks out.");
    else {
        normal_log("OSI", "Received SensorViewConfiguration for Sensor Id %llu", config.sensor_id().value());
        normal_log("OSI", "SVC Ground Truth FoV Horizontal %f, FoV Vertical %f, Range %f",
                   config.field_of_view_horizontal(), config.field_of_view_vertical(), config.range());
        normal_log("OSI", "SVC Mounting Position: (%f, %f, %f)", config.mounting_position().position().x(),
                   config.mounting_position().position().y(), config.mounting_position().position().z());
        normal_log("OSI", "SVC Mounting Orientation: (%f, %f, %f)", config.mounting_position().orientation().roll(),
                   config.mounting_position().orientation().pitch(), config.mounting_position().orientation().yaw());
    }

    return fmi2OK;
}

#include <model/profiles/profile_list.hpp>

fmi2Status CFrameworkPackaging::doCalc(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize,
                                   fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component) {
    DEBUGBREAK()

    osi3::SensorView currentIn;
    osi3::SensorData currentOut;
    if (get_fmi_sensor_view_in(currentIn)) {
        /* Clear Output */
        currentOut.Clear();
        currentOut.mutable_version()->CopyFrom(osi3::InterfaceVersion::descriptor()->file()->options().GetExtension(osi3::current_interface_version));
        /* Copy of SensorView */
        currentOut.add_sensor_view()->CopyFrom(currentIn);

        strategy.apply(currentOut);

        /* Serialize */
        set_fmi_sensor_data_out(currentOut);
    } else {
        /* We have no valid input, so no valid output */
        normal_log("OSI", "No valid input, therefore providing no valid output.");
        reset_fmi_sensor_data_out();
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::doTerm() {
    DEBUGBREAK()

    return fmi2OK;
}

void CFrameworkPackaging::doFree() {
    DEBUGBREAK()
}

/*
 * Generic C++ Wrapper Code
 */

CFrameworkPackaging::CFrameworkPackaging(fmi2String theinstanceName, fmi2Type thefmuType, fmi2String thefmuGUID,
                                 fmi2String thefmuResourceLocation, const fmi2CallbackFunctions *thefunctions,
                                 fmi2Boolean thevisible, fmi2Boolean theloggingOn)
        : instanceName(theinstanceName),
          fmuType(thefmuType),
          fmuGUID(thefmuGUID),
          fmuResourceLocation(thefmuResourceLocation),
          functions(*thefunctions),
          visible(!!thevisible),
          loggingOn(!!theloggingOn),
          simulation_started(false),
          message_sink([this](const std::string &message) -> void { normal_log("OSI", "%s", message.c_str()); }),
          alert_sink([this](const std::string &message) -> void { error_log("OSI", "%s", message.c_str()); }),
          strategy(profile, message_sink, alert_sink) {
    loggingCategories.clear();
    loggingCategories.insert("FMI");
    loggingCategories.insert("OSMP");
    loggingCategories.insert("OSI");
}

CFrameworkPackaging::~CFrameworkPackaging() = default;


fmi2Status
CFrameworkPackaging::SetDebugLogging(fmi2Boolean theloggingOn, size_t nCategories, const fmi2String categories[]) {
    fmi_verbose_log("fmi2SetDebugLogging(%s)", theloggingOn ? "true" : "false");
    loggingOn = theloggingOn != 0;
    if (categories && (nCategories > 0)) {
        loggingCategories.clear();
        for (size_t i = 0; i < nCategories; i++) {
            if (categories[i] == std::string("FMI"))
                loggingCategories.insert("FMI");
            else if (categories[i] == std::string("OSMP"))
                loggingCategories.insert("OSMP");
            else if (categories[i] == std::string("OSI"))
                loggingCategories.insert("OSI");
        }
    } else {
        loggingCategories.clear();
        loggingCategories.insert("FMI");
        loggingCategories.insert("OSMP");
        loggingCategories.insert("OSI");
    }
    return fmi2OK;
}

fmi2Component CFrameworkPackaging::Instantiate(fmi2String instanceName, fmi2Type fmuType, fmi2String fmuGUID,
                                           fmi2String fmuResourceLocation, const fmi2CallbackFunctions *functions,
                                           fmi2Boolean visible, fmi2Boolean loggingOn) {
    auto *myc = new CFrameworkPackaging(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions,
                                               visible, loggingOn);

    if (myc->doInit() != fmi2OK) {
        fmi_verbose_log_global(R"(fmi2Instantiate("%s",%d,"%s","%s","%s",%d,%d) = NULL (doInit failure))",
                               instanceName, fmuType, fmuGUID,
                               (fmuResourceLocation != nullptr) ? fmuResourceLocation : "<NULL>",
                               "FUNCTIONS", visible, loggingOn);
        delete myc;
        return nullptr;
    } else {
        fmi_verbose_log_global(R"(fmi2Instantiate("%s",%d,"%s","%s","%s",%d,%d) = %p)",
                               instanceName, fmuType, fmuGUID,
                               (fmuResourceLocation != nullptr) ? fmuResourceLocation : "<NULL>",
                               "FUNCTIONS", visible, loggingOn, myc);
        return (fmi2Component) myc;
    }
}

fmi2Status CFrameworkPackaging::SetupExperiment(fmi2Boolean toleranceDefined, fmi2Real tolerance, fmi2Real startTime,
                                            fmi2Boolean stopTimeDefined, fmi2Real stopTime) {
    fmi_verbose_log("fmi2SetupExperiment(%d,%g,%g,%d,%g)", toleranceDefined, tolerance, startTime, stopTimeDefined,
                    stopTime);
    return doStart(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

fmi2Status CFrameworkPackaging::EnterInitializationMode() {
    fmi_verbose_log("fmi2EnterInitializationMode()");
    return doEnterInitializationMode();
}

fmi2Status CFrameworkPackaging::ExitInitializationMode() {
    fmi_verbose_log("fmi2ExitInitializationMode()");
    simulation_started = true;
    return doExitInitializationMode();
}

fmi2Status CFrameworkPackaging::DoStep(fmi2Real currentCommunicationPoint, fmi2Real communicationStepSize,
                                   fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component) {
    fmi_verbose_log("fmi2DoStep(%g,%g,%d)", currentCommunicationPoint, communicationStepSize,
                    noSetFMUStatePriorToCurrentPointfmi2Component);
    return doCalc(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

fmi2Status CFrameworkPackaging::Terminate() {
    fmi_verbose_log("fmi2Terminate()");
    return doTerm();
}

fmi2Status CFrameworkPackaging::Reset() {
    fmi_verbose_log("fmi2Reset()");

    doFree();
    simulation_started = false;
    return doInit();
}

void CFrameworkPackaging::FreeInstance() {
    fmi_verbose_log("fmi2FreeInstance()");
    doFree();
}

fmi2Status CFrameworkPackaging::GetReal(const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    fmi_verbose_log("fmi2GetReal(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_REAL_VARS)
            value[i] = real_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::GetInteger(const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    fmi_verbose_log("fmi2GetInteger(...)");
    bool need_refresh = !simulation_started;
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_INTEGER_VARS) {
            if (need_refresh && (vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASEHI_IDX ||
                                 vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_BASELO_IDX ||
                                 vr[i] == FMI_INTEGER_SENSORVIEW_CONFIG_REQUEST_SIZE_IDX)) {
                refresh_fmi_sensor_view_config_request();
                need_refresh = false;
            }
            value[i] = integer_vars[vr[i]];
        } else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::GetBoolean(const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    fmi_verbose_log("fmi2GetBoolean(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_BOOLEAN_VARS)
            value[i] = boolean_vars[vr[i]];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::GetString(const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) {
    fmi_verbose_log("fmi2GetString(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_STRING_VARS)
            value[i] = string_vars[vr[i]].c_str();
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::SetReal(const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    fmi_verbose_log("fmi2SetReal(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_REAL_VARS)
            real_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::SetInteger(const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    fmi_verbose_log("fmi2SetInteger(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_INTEGER_VARS)
            integer_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::SetBoolean(const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    fmi_verbose_log("fmi2SetBoolean(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_BOOLEAN_VARS)
            boolean_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

fmi2Status CFrameworkPackaging::SetString(const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {
    fmi_verbose_log("fmi2SetString(...)");
    for (size_t i = 0; i < nvr; i++) {
        if (vr[i] < FMI_STRING_VARS)
            string_vars[vr[i]] = value[i];
        else
            return fmi2Error;
    }
    return fmi2OK;
}

/*
 * FMI 2.0 Co-Simulation Interface API
 */

extern "C" {

FMI2_Export const char *fmi2GetTypesPlatform() {
    return fmi2TypesPlatform;
}

FMI2_Export const char *fmi2GetVersion() {
    return fmi2Version;
}

FMI2_Export fmi2Status
fmi2SetDebugLogging(fmi2Component c, fmi2Boolean loggingOn, size_t nCategories, const fmi2String categories[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->SetDebugLogging(loggingOn, nCategories, categories);
}

/*
* Functions for Co-Simulation
*/
FMI2_Export fmi2Component fmi2Instantiate(fmi2String instanceName,
                                          fmi2Type fmuType,
                                          fmi2String fmuGUID,
                                          fmi2String fmuResourceLocation,
                                          const fmi2CallbackFunctions *functions,
                                          fmi2Boolean visible,
                                          fmi2Boolean loggingOn) {
    return CFrameworkPackaging::Instantiate(instanceName, fmuType, fmuGUID, fmuResourceLocation, functions, visible,
                                        loggingOn);
}

FMI2_Export fmi2Status fmi2SetupExperiment(fmi2Component c,
                                           fmi2Boolean toleranceDefined,
                                           fmi2Real tolerance,
                                           fmi2Real startTime,
                                           fmi2Boolean stopTimeDefined,
                                           fmi2Real stopTime) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->SetupExperiment(toleranceDefined, tolerance, startTime, stopTimeDefined, stopTime);
}

FMI2_Export fmi2Status fmi2EnterInitializationMode(fmi2Component c) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->EnterInitializationMode();
}

FMI2_Export fmi2Status fmi2ExitInitializationMode(fmi2Component c) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->ExitInitializationMode();
}

FMI2_Export fmi2Status fmi2DoStep(fmi2Component c,
                                  fmi2Real currentCommunicationPoint,
                                  fmi2Real communicationStepSize,
                                  fmi2Boolean noSetFMUStatePriorToCurrentPointfmi2Component) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->DoStep(currentCommunicationPoint, communicationStepSize, noSetFMUStatePriorToCurrentPointfmi2Component);
}

FMI2_Export fmi2Status fmi2Terminate(fmi2Component c) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->Terminate();
}

FMI2_Export fmi2Status fmi2Reset(fmi2Component c) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->Reset();
}

FMI2_Export void fmi2FreeInstance(fmi2Component c) {
    auto *myc = (CFrameworkPackaging *) c;
    myc->FreeInstance();
    delete myc;
}

/*
 * Data Exchange Functions
 */
FMI2_Export fmi2Status fmi2GetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Real value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->GetReal(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2GetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Integer value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->GetInteger(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2GetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2Boolean value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->GetBoolean(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2GetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, fmi2String value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->GetString(vr, nvr, value);
}

FMI2_Export fmi2Status fmi2SetReal(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Real value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->SetReal(vr, nvr, value);
}

FMI2_Export fmi2Status
fmi2SetInteger(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Integer value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->SetInteger(vr, nvr, value);
}

FMI2_Export fmi2Status
fmi2SetBoolean(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2Boolean value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->SetBoolean(vr, nvr, value);
}

FMI2_Export fmi2Status
fmi2SetString(fmi2Component c, const fmi2ValueReference vr[], size_t nvr, const fmi2String value[]) {
    auto *myc = (CFrameworkPackaging *) c;
    return myc->SetString(vr, nvr, value);
}

/*
 * Unsupported Features (FMUState, Derivatives, Async DoStep, Status Enquiries)
 */
FMI2_Export fmi2Status fmi2GetFMUstate(fmi2Component c, fmi2FMUstate *FMUstate) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SetFMUstate(fmi2Component c, fmi2FMUstate FMUstate) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2FreeFMUstate(fmi2Component c, fmi2FMUstate *FMUstate) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SerializedFMUstateSize(fmi2Component c, fmi2FMUstate FMUstate, size_t *size) {
    return fmi2Error;
}

FMI2_Export fmi2Status
fmi2SerializeFMUstate(fmi2Component c, fmi2FMUstate FMUstate, fmi2Byte serializedState[], size_t size) {
    return fmi2Error;
}

FMI2_Export fmi2Status
fmi2DeSerializeFMUstate(fmi2Component c, const fmi2Byte serializedState[], size_t size, fmi2FMUstate *FMUstate) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2GetDirectionalDerivative(fmi2Component c,
                                                    const fmi2ValueReference vUnknown_ref[], size_t nUnknown,
                                                    const fmi2ValueReference vKnown_ref[], size_t nKnown,
                                                    const fmi2Real dvKnown[],
                                                    fmi2Real dvUnknown[]) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2SetRealInputDerivatives(fmi2Component c,
                                                   const fmi2ValueReference vr[],
                                                   size_t nvr,
                                                   const fmi2Integer order[],
                                                   const fmi2Real value[]) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2GetRealOutputDerivatives(fmi2Component c,
                                                    const fmi2ValueReference vr[],
                                                    size_t nvr,
                                                    const fmi2Integer order[],
                                                    fmi2Real value[]) {
    return fmi2Error;
}

FMI2_Export fmi2Status fmi2CancelStep(fmi2Component c) {
    return fmi2OK;
}

FMI2_Export fmi2Status fmi2GetStatus(fmi2Component c, const fmi2StatusKind s, fmi2Status *value) {
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetRealStatus(fmi2Component c, const fmi2StatusKind s, fmi2Real *value) {
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetIntegerStatus(fmi2Component c, const fmi2StatusKind s, fmi2Integer *value) {
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetBooleanStatus(fmi2Component c, const fmi2StatusKind s, fmi2Boolean *value) {
    return fmi2Discard;
}

FMI2_Export fmi2Status fmi2GetStringStatus(fmi2Component c, const fmi2StatusKind s, fmi2String *value) {
    return fmi2Discard;
}

}
