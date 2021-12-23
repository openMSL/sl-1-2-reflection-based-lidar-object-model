cmake_minimum_required(VERSION 3.12)
project(osmp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(LINK_WITH_SHARED_OSI OFF CACHE BOOL "Link FMU with shared OSI library instead of statically linking")
set(PUBLIC_LOGGING ON CACHE BOOL "Enable logging via FMI logger")
set(PRIVATE_LOGGING OFF CACHE BOOL "Enable private logging to file")
if (WIN32)
    set(PRIVATE_LOG_PATH "C:/tmp/${MODEL_NAME}.log" CACHE FILEPATH "Path to write private log file to")
else ()
    set(PRIVATE_LOG_PATH "/tmp/${MODEL_NAME}.log" CACHE FILEPATH "Path to write private log file to")
endif ()
set(VERBOSE_FMI_LOGGING OFF CACHE BOOL "Enable detailed FMI function logging")
set(DEBUG_BREAKS OFF CACHE BOOL "Enable debugger traps for debug builds of FMU")

file(READ ../model/profiles/profile_list.conf FIRST_PROFILE)
string(REPLACE "\n" ";" FIRST_PROFILE ${FIRST_PROFILE})
list(GET FIRST_PROFILE 0 FIRST_PROFILE)

file(READ variableNamingConvention.conf FMU_VARIABLE_NAMING_CONVENTION)
string(REPLACE "\n" "" FMU_VARIABLE_NAMING_CONVENTION ${FMU_VARIABLE_NAMING_CONVENTION})

string(TIMESTAMP FMUTIMESTAMP UTC)
string(MD5 FMUGUID modelDescription.in.xml)
configure_file(modelDescription.in.xml modelDescription.xml @ONLY)

find_package(Protobuf 2.6.1 REQUIRED)
add_library(${MODEL_NAME} SHARED FrameworkPackaging.cpp $<TARGET_OBJECTS:model_obj>)
set_target_properties(${MODEL_NAME} PROPERTIES PREFIX "")
target_compile_definitions(${MODEL_NAME} PRIVATE "FMU_SHARED_OBJECT")
target_compile_definitions(${MODEL_NAME} PRIVATE "FMU_GUID=\"${FMUGUID}\"")
if (LINK_WITH_SHARED_OSI)
    target_link_libraries(FrameworkPackaging open_simulation_interface model_obj)
else ()
    target_link_libraries(${MODEL_NAME} open_simulation_interface_pic model_obj)
endif ()
if (PRIVATE_LOGGING)
    file(TO_NATIVE_PATH ${PRIVATE_LOG_PATH} PRIVATE_LOG_PATH_NATIVE)
    string(REPLACE "\\" "\\\\" PRIVATE_LOG_PATH_ESCAPED ${PRIVATE_LOG_PATH_NATIVE})
    target_compile_definitions(${MODEL_NAME} PRIVATE
            "PRIVATE_LOG_PATH=\"${PRIVATE_LOG_PATH_ESCAPED}\"")
endif ()
target_compile_definitions(${MODEL_NAME} PRIVATE
        $<$<BOOL:${PUBLIC_LOGGING}>:PUBLIC_LOGGING>
        $<$<BOOL:${VERBOSE_FMI_LOGGING}>:VERBOSE_FMI_LOGGING>
        $<$<BOOL:${DEBUG_BREAKS}>:DEBUG_BREAKS>)

if (WIN32)
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "win64")
    else ()
        set(FMI_BINARIES_PLATFORM "win32")
    endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "linux64")
    else ()
        set(FMI_BINARIES_PLATFORM "linux32")
    endif ()
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    if (CMAKE_SIZEOF_VOID_P EQUAL 8)
        set(FMI_BINARIES_PLATFORM "darwin64")
    else ()
        set(FMI_BINARIES_PLATFORM "darwin32")
    endif ()
endif ()

add_custom_command(TARGET ${MODEL_NAME}
    POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
    COMMAND ${CMAKE_COMMAND} -E make_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu"
    COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${MODEL_NAME}> $<$<BOOL:${MSVC}>:$<$<CONFIG:Debug>:$<TARGET_PDB_FILE:${MODEL_NAME}>>> "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
    COMMAND ${CMAKE_COMMAND} -E copy "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/" "${FMU_INSTALL_DIR}/${MODEL_NAME}/"
    COMMAND ${CMAKE_COMMAND} -E chdir "${CMAKE_CURRENT_BINARY_DIR}/buildfmu" ${CMAKE_COMMAND} -E tar "cfv" "${FMU_INSTALL_DIR}/${MODEL_NAME}.fmu" --format=zip "modelDescription.xml" "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/binaries/${FMI_BINARIES_PLATFORM}"
    #COMMAND ${CMAKE_COMMAND} -E tar "xzf" "${FMU_INSTALL_DIR}/${MODEL_NAME}.fmu"

)

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    add_custom_command(TARGET ${MODEL_NAME}
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_directory "${CMAKE_CURRENT_BINARY_DIR}/buildfmu/" "${FMU_INSTALL_DIR}/${MODEL_NAME}/"
            )
endif ()