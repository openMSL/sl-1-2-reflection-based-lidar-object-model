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

macro(add_fmu_csv_output_strategy STRATEGY_NAME ENTRY_POINT)
    list(APPEND FMU_CSV_OUTPUT_STRATEGIES ${STRATEGY_NAME})
    set(FMU_CSV_OUTPUT_STRATEGIES ${FMU_CSV_OUTPUT_STRATEGIES} PARENT_SCOPE)

    string(TOLOWER ${ENTRY_POINT} ENTRY_POINT_DIR)
    list(APPEND STRATEGY_INCLUDES "#include \"${ENTRY_POINT_DIR}/${ENTRY_POINT}.hpp\"")
    set(STRATEGY_INCLUDES ${STRATEGY_INCLUDES} PARENT_SCOPE)

    list(APPEND CSV_OUTPUT_STRATEGY_INSTANCES "csv_output_strategies.emplace_back(new ${ENTRY_POINT}(profile, log, alert))\;")
    set(CSV_OUTPUT_STRATEGY_INSTANCES ${CSV_OUTPUT_STRATEGY_INSTANCES} PARENT_SCOPE)
endmacro()

macro(add_fmu_pcd_output_strategy STRATEGY_NAME ENTRY_POINT)
    list(APPEND FMU_PCD_OUTPUT_STRATEGIES ${STRATEGY_NAME})
    set(FMU_PCD_OUTPUT_STRATEGIES ${FMU_PCD_OUTPUT_STRATEGIES} PARENT_SCOPE)

    string(TOLOWER ${ENTRY_POINT} ENTRY_POINT_DIR)
    list(APPEND STRATEGY_INCLUDES "#include \"${ENTRY_POINT_DIR}/${ENTRY_POINT}.hpp\"")
    set(STRATEGY_INCLUDES ${STRATEGY_INCLUDES} PARENT_SCOPE)

    list(APPEND PCD_OUTPUT_STRATEGY_INSTANCES "pcd_output_strategies.emplace_back(new ${ENTRY_POINT}(profile, log, alert))\;")
    set(PCD_OUTPUT_STRATEGY_INSTANCES ${PCD_OUTPUT_STRATEGY_INSTANCES} PARENT_SCOPE)
endmacro()

macro(add_fmu_ros_output_strategy STRATEGY_NAME ENTRY_POINT)
    list(APPEND FMU_ROS_OUTPUT_STRATEGIES ${STRATEGY_NAME})
    set(FMU_ROS_OUTPUT_STRATEGIES ${FMU_ROS_OUTPUT_STRATEGIES} PARENT_SCOPE)

    string(TOLOWER ${ENTRY_POINT} ENTRY_POINT_DIR)
    list(APPEND STRATEGY_INCLUDES "#include \"${ENTRY_POINT_DIR}/${ENTRY_POINT}.hpp\"")
    set(STRATEGY_INCLUDES ${STRATEGY_INCLUDES} PARENT_SCOPE)

    list(APPEND ROS_OUTPUT_STRATEGY_INSTANCES "ros_output_strategies.emplace_back(new ${ENTRY_POINT}(profile, log, alert))\;")
    set(ROS_OUTPUT_STRATEGY_INSTANCES ${ROS_OUTPUT_STRATEGY_INSTANCES} PARENT_SCOPE)
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
message("---> Configuring model core strategies from sequence.conf")
string(REPLACE "\n" ";" strategies ${strategies})
foreach(strategy ${strategies})
    if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/strategies/${strategy})
        message("Found strategy ${strategy}")
        add_subdirectory(strategies/${strategy})
    else()
	    message(FATAL_ERROR "Invalid strategy defined: ${strategy}")
    endif()
endforeach()

file(READ strategies/csv_output_sequence.conf csv_output_strategies)
if(csv_output_strategies)
message("---> Configuring .csv output strategies from csv_output_sequence.conf")
    string(REPLACE "\n" ";" csv_output_strategies ${csv_output_strategies})
    foreach(strategy ${csv_output_strategies})
        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/strategies/${strategy})
            message("Found strategy ${strategy}")
            add_subdirectory(strategies/${strategy})
        else()
            message(FATAL_ERROR "Invalid strategy defined: ${strategy}")
        endif()
    endforeach()
endif()

file(READ strategies/pcd_output_sequence.conf pcd_output_strategies)
if(pcd_output_strategies)
message("---> Configuring .pcd output strategies from pcd_output_sequence.conf")
    string(REPLACE "\n" ";" pcd_output_strategies ${pcd_output_strategies})
    foreach(strategy ${pcd_output_strategies})
        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/strategies/${strategy})
            message("Found strategy ${strategy}")
            add_subdirectory(strategies/${strategy})
        else()
            message(FATAL_ERROR "Invalid strategy defined: ${strategy}")
        endif()
    endforeach()
endif()

if (${CMAKE_SYSTEM_NAME} MATCHES "Linux") # Windows support for ROS output strategies currrently under developement!
file(READ strategies/ros_output_sequence.conf ros_output_strategies)
if(ros_output_strategies)
    message("---> Configuring ROS output strategies from ros_output_sequence.conf")
    string(REPLACE "\n" ";" ros_output_strategies ${ros_output_strategies})
            
    ### find our os (wins, centos, ubuntu, etc)
    set(VAR_OS "")
    if(CMAKE_SYSTEM_NAME MATCHES "Linux")
        message("Found OS for ROS-output-strategies: Linux")
        set(AVAILABLE_ROS_VERSIONS "noetic;melodic")
        set(ROS_FOUND FALSE)
        foreach(version ${AVAILABLE_ROS_VERSIONS})
            if(NOT ROS_FOUND)
                if(EXISTS /opt/ros/${version}/)
                    message("Found ROS version ${version}")
                    set(FOUND_ROS_DISTRO ${version})
                    set(ROS_FOUND TRUE)
                endif()
            endif()
        endforeach()
        
        if(ROS_FOUND)
            if(FOUND_ROS_DISTRO MATCHES "noetic")
                list(APPEND CMAKE_PREFIX_PATH /opt/ros/noetic/)
                set(PYTHONPATH /opt/ros/noetic/lib/python3/dist-packages)
            elseif(FOUND_ROS_DISTRO MATCHES "melodic")
                list(APPEND CMAKE_PREFIX_PATH /opt/ros/melodic/)
                set(PYTHONPATH /opt/ros/melodic/lib/python2.7/dist-packages)
                message("Hi")
            endif()
        else()
            message("ROS distro is unknown!")
        endif()
    elseif(CMAKE_SYSTEM_NAME MATCHES "Windows")
        message("Found OS for ROS-output-strategies: Windows")
        set(VAR_OS "wins")
        set(PYTHON_EXECUTABLE           C:/Python310/python.exe) # Or wherever Python is located
        list(APPEND CMAKE_PREFIX_PATH   C:/opt/ros/noetic/x64/) # Or wherever ROS is located
        set(PCL_COMMON_LIBRARY          C:/opt/ros/noetic/x64/lib) # Or wherever PCL common libs are located
        set(PCL_COMMON_INCLUDE_DIR      C:/opt/ros/noetic/x64/include/pcl/common) # Or wherever PCL common includes are located
        set(BOOST_INCLUDEDIR            C:/opt/ros/noetic/x64/include/boost) # Or wherever Boost includes are located
    else()
        message(FATAL "No OS detected!")
    endif()
    
    #set(CATKIN_ENABLE_TESTING OFF CACHE BOOL "Let catkin create tests" FORCE)
    find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        tf
    )

    # see here: https://github.com/PointCloudLibrary/pcl/issues/3680
    # when this is fixed, we can remove the following 3 lines.
    if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
        set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
    endif()
    find_package(PCL REQUIRED COMPONENTS common)

    if(CMAKE_SYSTEM_NAME MATCHES "Windows")
        string(APPEND PCL_COMMON_LIBRARIES /pcl_common_release.lib)
    endif()

    foreach(strategy ${ros_output_strategies})
        if(IS_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/strategies/${strategy})
            message("Found strategy ${strategy}")
            add_subdirectory(strategies/${strategy})
        else()
            message(FATAL_ERROR "Invalid strategy defined: ${strategy}")
        endif()
    endforeach()
endif()
endif()

# Lists items are separated by a semicolon, replace with newline
string(REPLACE ";" "\n" STRATEGY_INCLUDES "${STRATEGY_INCLUDES}")
string(REPLACE ";" ";\n" STRATEGY_INSTANCES "${STRATEGY_INSTANCES}")
if(CSV_OUTPUT_STRATEGY_INSTANCES)
    string(REPLACE ";" ";\n" CSV_OUTPUT_STRATEGY_INSTANCES "${CSV_OUTPUT_STRATEGY_INSTANCES}")
endif()
if(PCD_OUTPUT_STRATEGY_INSTANCES)
    string(REPLACE ";" ";\n" PCD_OUTPUT_STRATEGY_INSTANCES "${PCD_OUTPUT_STRATEGY_INSTANCES}")
endif()
if(ROS_OUTPUT_STRATEGY_INSTANCES)
    string(REPLACE ";" ";\n" ROS_OUTPUT_STRATEGY_INSTANCES "${ROS_OUTPUT_STRATEGY_INSTANCES}")
endif()
configure_file(${INCLUDE_SOURCE_DIR}/sequence.cpp.in ${INCLUDE_TARGET_DIR}/sequence.cpp)
configure_file(${PROFILE_SOURCE_DIR}/profile.hpp.in ${PROFILE_TARGET_DIR}/profile.hpp)
configure_file(${PROFILE_SOURCE_DIR}/init_profile.cpp.in ${PROFILE_TARGET_DIR}/init_profile.cpp)
foreach(PROFILE_NAME IN ITEMS ${PROFILES})
    file(APPEND ${PROFILE_TARGET_DIR}/profile_${PROFILE_NAME}.hpp "\n        return ${PROFILE_NAME};\n    }\n}")
endforeach()

add_library(${PROJECT_NAME}_obj OBJECT ${INCLUDE_TARGET_DIR}/sequence.cpp)
target_link_libraries(${PROJECT_NAME}_obj INTERFACE open_simulation_interface_obj PUBLIC ${FMU_STRATEGIES} ${FMU_CSV_OUTPUT_STRATEGIES} ${FMU_PCD_OUTPUT_STRATEGIES} ${FMU_ROS_OUTPUT_STRATEGIES})
target_include_directories(${PROJECT_NAME}_obj PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
set_property(TARGET ${PROJECT_NAME}_obj PROPERTY POSITION_INDEPENDENT_CODE ON)
