cmake_minimum_required(VERSION 3.12)
project(ros_gt_objects)

# TODO MODIFY THE FOLLOWING LINES AS NEEDED
set(STRATEGY_ENTRY_POINT ros_gt_objects)
set(STRATEGY_SOURCES src/ros_gt_objects.cpp)
set(STRATEGY_EXTRA_PUBLIC_LIBS_OR_TARGETS ${PCL_COMMON_LIBRARIES} ${catkin_LIBRARIES})
set(STRATEGY_EXTRA_PRIVATE_LIBS_OR_TARGETS "")
set(STRATEGY_EXTRA_INCLUDE_DIRECTORIES ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
set(STRATEGY_EXTRA_DEPENDENCIES ${catkin_EXPORTED_TARGETS})
#set(STRATEGY_PROFILE_EXTENSION ${PROFILES})

# no need to modify commands below this line
set(CMAKE_CXX_STANDARD 17)

if(NOT COMMAND add_fmu_ros_output_strategy)
    message(FATAL_ERROR "This project directory has to be included by OSI FMU platform, it can't be build out of that context!")
endif()

add_fmu_ros_output_strategy(${PROJECT_NAME} ${STRATEGY_ENTRY_POINT})
if(STRATEGY_PROFILE_EXTENSION)
    add_profile_part(${STRATEGY_PROFILE_EXTENSION})
endif()

add_library(${PROJECT_NAME} STATIC ${STRATEGY_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${STRATEGY_EXTRA_PRIVATE_LIBS_OR_TARGETS} PUBLIC model::strategy open_simulation_interface_obj ${STRATEGY_EXTRA_PUBLIC_LIBS_OR_TARGETS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${STRATEGY_EXTRA_INCLUDE_DIRECTORIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
add_dependencies(${PROJECT_NAME} ${STRATEGY_EXTRA_DEPENDENCIES})
