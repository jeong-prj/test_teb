cmake_minimum_required(VERSION 3.0.2)
project(test_teb)

#add_compile_options(-std=c++11)
set (CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(TEST_TEB_PROJ_DIRECTORY ${PROJECT_SOURCE_DIR} )

find_package(OpenCV 3.2 REQUIRED)
find_package(Eigen3 REQUIRED)
#find_package(Boost REQUIRED COMPONENTS system thread)

###

find_package(Boost REQUIRED COMPONENTS system thread graph filesystem)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
find_package(SUITESPARSE REQUIRED)

find_package(G2O REQUIRED)

#find_package(Eigen3 REQUIRED)



set(EXTERNAL_INCLUDE_DIRS ${Eigen_INCLUDE_DIRS} ${SUITESPARSE_INCLUDE_DIRS} ${G2O_INCLUDE_DIR})
set(EXTERNAL_LIBS ${SUITESPARSE_LIBRARIES} ${G2O_LIBRARIES})

###

#find_package(OpenMP REQUIRED)
#find_package(OpenMP)
#if (OPENMP_FOUND)
#    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
#    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
#    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
#endif()


file(GLOB_RECURSE SRC_FILES
#        src/*.cpp
#        example/teb_basics.cpp
        example/teb_overall.cpp
#        src/graph_search.cpp
#        src/homotopy_class_planner.cpp
        src/obstacles.cpp
        src/optimal_planner.cpp
        src/recovery_behaviors.cpp
        src/teb_config.cpp
#        src/teb_local_planner_ros.cpp
#        src/test_optim_node.cpp
        src/timed_elastic_band.cpp

        src/sub_src/duration.cpp
        src/sub_src/footprint.cpp
        src/sub_src/array_parser.cpp
        src/sub_src/costmap_math.cpp

#        src/xml_/*.cpp

#        src/sub_src/*.c
        )


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${TEST_TEB_PROJ_DIRECTORY}/include
)

###
#find_package(catkin REQUIRED COMPONENTS
#        base_local_planner
#        costmap_2d
#        costmap_converter
#        cmake_modules
#        dynamic_reconfigure
#        geometry_msgs
#        interactive_markers
#        message_generation
#        nav_core
#        nav_msgs
#        mbf_costmap_core
#        mbf_msgs
#        roscpp
#        std_msgs
#        pluginlib
#        tf2
#        tf2_eigen
#        tf2_geometry_msgs
#        tf2_ros
#        visualization_msgs
#        )

#catkin_package(
#        INCLUDE_DIRS include ${EXTERNAL_INCLUDE_DIRS}
#        LIBRARIES teb_local_planner ${EXTERNAL_LIBS}
#        CATKIN_DEPENDS
#        base_local_planner
#        costmap_2d
#        costmap_converter
#        dynamic_reconfigure
#        geometry_msgs
#        interactive_markers
#        message_runtime
#        nav_core
#        nav_msgs
#        pluginlib
#        roscpp
#        mbf_costmap_core
#        std_msgs
#        tf2
#        tf2_ros
#        visualization_msgs
#        DEPENDS SUITESPARSE G2O
#)

###

# build mpbb lib
add_library(test_teb SHARED ${SRC_FILES} include/ros/impl/duration.h)
target_link_libraries(test_teb
        -dpthread
        ${Boost_LIBRARIES}
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${EXTERNAL_LIBS}

        )

set_target_properties( test_teb PROPERTIES
        LIBRARY_OUTPUT_DIRECTORY ${TEST_TEB_PROJ_DIRECTORY}/lib
        )

#add_executable(test_teb_run ${TEST_TEB_PROJ_DIRECTORY}/example/teb_basics.cpp include/ros/impl/duration.h)
#target_link_libraries( test_teb_run test_teb)
#set_target_properties( test_teb_run PROPERTIES
#        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

add_executable(teb_overall_run ${TEST_TEB_PROJ_DIRECTORY}/example/teb_overall.cpp include/ros/impl/duration.h)
target_link_libraries( teb_overall_run test_teb)
set_target_properties( teb_overall_run PROPERTIES
        RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)


#add_executable(ffp_vs_wfd ${FD_PROJ_DIRECTORY}/example/ffp_vs_wfd.cpp ${SRC_FILES} )
#target_link_libraries(ffp_vs_wfd ${OpenCV_LIBS})
#set_target_properties( ffp_vs_wfd PROPERTIES
#  RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
