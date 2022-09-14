cmake_minimum_required(VERSION 3.12)
project(CsvOutputGTObjects)

set(CSV_GTOBJECTS_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(CSV_GTOBJECTS_TARGET_DIR ${CMAKE_CURRENT_BINARY_DIR}/src)
if (WIN32)
    set(CSV_PATH C:/TEMP)
elseif (${CMAKE_SYSTEM_NAME} MATCHES "Linux")
    set(CSV_PATH /tmp)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/include/csvoutputgtobjects/set_csv_file_path_gtobjects.cpp.in ${CMAKE_CURRENT_BINARY_DIR}/include/csvoutputgtobjects/set_csv_file_path_gtobjects.cpp)

# TODO MODIFY THE FOLLOWING THREE LINES AS NEEDED
set(STRATEGY_ENTRY_POINT CsvOutputGTObjects)
set(STRATEGY_SOURCES ${CSV_GTOBJECTS_SOURCE_DIR}/CsvOutputGTObjects.cpp ../transformation-functions/TransformationFunctions.cpp)
set(STRATEGY_EXTRA_PUBLIC_LIBS_OR_TARGETS "")
set(STRATEGY_EXTRA_PRIVATE_LIBS_OR_TARGETS "")
set(STRATEGY_EXTRA_INCLUDE_DIRECTORIES ${CMAKE_CURRENT_BINARY_DIR}/include)
#set(STRATEGY_PROFILE_EXTENSION ${PROFILES})

# no need to modify commands below this line
set(CMAKE_CXX_STANDARD 17)

if(NOT COMMAND add_fmu_csv_output_strategy)
    message(FATAL_ERROR "This project directory has to be included by OSI FMU platform, it can't be build out of that context!")
endif()

add_fmu_csv_output_strategy(${PROJECT_NAME} ${STRATEGY_ENTRY_POINT})
if(STRATEGY_PROFILE_EXTENSION)
    add_profile_part(${STRATEGY_PROFILE_EXTENSION})
endif()


add_library(${PROJECT_NAME} STATIC ${STRATEGY_SOURCES})
target_link_libraries(${PROJECT_NAME} PRIVATE ${STRATEGY_EXTRA_PRIVATE_LIBS_OR_TARGETS} PUBLIC model::strategy open_simulation_interface_obj ${STRATEGY_EXTRA_PUBLIC_LIBS_OR_TARGETS})
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include ${STRATEGY_EXTRA_INCLUDE_DIRECTORIES})
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)

