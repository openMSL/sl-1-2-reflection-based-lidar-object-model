cmake_minimum_required(VERSION 3.12)
project(model)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(INCLUDE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)
set(INCLUDE_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
set(PROFILE_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/profiles)
set(PROFILE_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/profiles)

file(READ ../model/profiles/profile_list.conf PROFILES)
string(REPLACE "\n" ";" PROFILES ${PROFILES})
list(GET PROFILES 0 FIRST_PROFILE)

foreach(PROFILE_NAME IN ITEMS ${PROFILES})
    configure_file(${PROFILE_SOURCE_DIR}/profile_${PROFILE_NAME}.hpp.in ${PROFILE_TARGET_DIR}/profile_${PROFILE_NAME}.hpp)
endforeach()

macro(add_fmu_strategy STRATEGY_NAME ENTRY_POINT)
    list(APPEND FMU_STRATEGIES ${STRATEGY_NAME})
    set(FMU_STRATEGIES ${FMU_STRATEGIES} PARENT_SCOPE)

    string(TOLOWER ${ENTRY_POINT} ENTRY_POINT_DIR)
    list(APPEND STRATEGY_INCLUDES "#include \"${ENTRY_POINT_DIR}/${ENTRY_POINT}.hpp\"")
    set(STRATEGY_INCLUDES ${STRATEGY_INCLUDES} PARENT_SCOPE)

    list(APPEND STRATEGY_INSTANCES "strategies.emplace_back(new ${ENTRY_POINT}(profile, log, alert))\;")
    set(STRATEGY_INSTANCES ${STRATEGY_INSTANCES} PARENT_SCOPE)
endmacro()

macro (add_profile_part)
    foreach(PROFILE_NAME IN ITEMS ${ARGN})
        file(READ ${CMAKE_CURRENT_SOURCE_DIR}/profile_${PROFILE_NAME}.hpp.in STRATEGY_PROFILE_PART)
        string(REGEX REPLACE ";" "\\\\;" STRATEGY_PROFILE_PART "${STRATEGY_PROFILE_PART}")
        file(APPEND ${PROFILE_TARGET_DIR}/profile_${PROFILE_NAME}.hpp ${STRATEGY_PROFILE_PART})
    endforeach()
    file(READ ${CMAKE_CURRENT_SOURCE_DIR}/profile_struct.hpp.in STRUCT)
    string(REGEX REPLACE ";" "\\\\;" STRUCT "${STRUCT}")
    list(APPEND PROFILE_STRUCTS "${STRUCT}")
    set(PROFILE_STRUCTS ${PROFILE_STRUCTS} PARENT_SCOPE)
endmacro()

add_library(model::strategy INTERFACE IMPORTED)
set_target_properties(model::strategy PROPERTIES
                      INTERFACE_COMPILE_FEATURES "cxx_std_17"
                      INTERFACE_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_SOURCE_DIR}/..;${CMAKE_CURRENT_BINARY_DIR}/.."
                      )

file(READ strategies/sequence.conf strategies)
string(REPLACE "\n" ";" strategies ${strategies})
foreach(strategy ${strategies})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/strategies/${strategy})
        message("Found strategy ${strategy}")
        add_subdirectory(strategies/${strategy})
    else()
	message(FATAL_ERROR "Invalid strategy defined: ${strategy}")
    endif()
endforeach()

# Lists items are separated by a semicolon, replace with newline
string(REPLACE ";" "\n" STRATEGY_INCLUDES "${STRATEGY_INCLUDES}")
string(REPLACE ";" ";\n" STRATEGY_INSTANCES "${STRATEGY_INSTANCES}")
configure_file(${INCLUDE_SOURCE_DIR}/sequence.cpp.in ${INCLUDE_TARGET_DIR}/sequence.cpp)
configure_file(${PROFILE_SOURCE_DIR}/profile.hpp.in ${PROFILE_TARGET_DIR}/profile.hpp)
configure_file(${PROFILE_SOURCE_DIR}/init_profile.hpp.in ${PROFILE_TARGET_DIR}/init_profile.hpp)
foreach(PROFILE_NAME IN ITEMS ${PROFILES})
    file(APPEND ${PROFILE_TARGET_DIR}/profile_${PROFILE_NAME}.hpp "\n        return ${PROFILE_NAME};\n    }\n}")
endforeach()

add_library(${PROJECT_NAME}_obj OBJECT ${INCLUDE_TARGET_DIR}/sequence.cpp)
target_link_libraries(${PROJECT_NAME}_obj INTERFACE open_simulation_interface_obj PUBLIC ${FMU_STRATEGIES})
target_include_directories(${PROJECT_NAME}_obj PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
set_property(TARGET ${PROJECT_NAME}_obj PROPERTY POSITION_INDEPENDENT_CODE ON)
