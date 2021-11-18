cmake_minimum_required(VERSION 3.0.2)
project(naverlabs_indoor_lidar_api)

set(CMAKE_CXX_STANDARD 17) # 17: to use filesystem

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif()

find_package(hdf5 COMPONENTS CXX QUIET)
find_package(Ceres REQUIRED)
if(NOT hdf5_FOUND)
    find_package(HDF5 COMPONENTS CXX REQUIRED)
    if(HDF5_DIR)
        set(HDF5_INCLUDE_DIRS ${HDF5_DIR})
    endif()
    if(CMAKE_CROSSCOMPILING)
        set(HDF5_INCLUDE_DIRS ${CMAKE_SYSROOT}/${HDF5_INCLUDE_DIRS})
        set(HDF5_INCLUDE_DIR ${CMAKE_SYSROOT}/${HDF5_INCLUDE_DIRS})
    endif()
    if(NOT CMAKE_CROSSCOMPILING)
        set(HDF5_LIBRARIES
            ${HDF5_hdf5_LIBRARIES_RELEASE}
            ${HDF5_hdf5_cpp_LIBRARY_RELEASE}
            ${HDF5_CXX_LIBRARY_hdf5}
            ${HDF5_CXX_LIBRARY_hdf5_cpp}
            )
    else()
        set(HDF5_LIB_DIR  ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/hdf5/serial)
        set(HDF5_LIBRARIES ${HDF5_LIB_DIR}/libhdf5.so ${HDF5_LIB_DIR}/libhdf5_cpp.so)
    endif()

    include_directories(${HDF5_INCLUDE_DIRS})
else()
    if(hdf5_DIR)
        set(HDF5_INCLUDE_DIRS ${hdf5_DIR})
    endif()
    set(HDF5_LIBRARIES hdf5-shared hdf5_cpp-shared)
endif()

##########################
find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        geometry_msgs
        message_generation
        sensor_msgs
        nav_msgs
        pcl_ros
)
find_package(OpenMP)
find_package(PCL 1.8 REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

#add_message_files(
#        FILES
#)
#
#generate_messages(
#        DEPENDENCIES
#        std_msgs
#        geometry_msgs
#        sensor_msgs
#        nav_msgs
#)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  src
  ${CERES_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(labs_src
)

## benchmark
add_executable(labs_benchmark src/data_publisher.cpp src/nvutils.cpp)
target_include_directories(labs_benchmark
        PUBLIC
        ${HDF5_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        )
target_link_libraries(labs_benchmark
        PUBLIC
        ${HDF5_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        ${catkin_LIBRARIES}
        Eigen3::Eigen
        stdc++fs
        )
#
#
#add_executable(labs_rot_benchmark src/labs_rot_benchmark.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(labs_rot_benchmark
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(labs_rot_benchmark
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#
#add_executable(labs_for_paper src/viz_for_paper/viz_for_loop_in_labs.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(labs_for_paper
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(labs_for_paper
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#add_executable(kitti_benchmark src/kitti_benchmark.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(kitti_benchmark
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(kitti_benchmark
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#
#add_executable(kitti_loop_viz src/viz_for_paper/viz_kitti_loop.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(kitti_loop_viz
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(kitti_loop_viz
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#add_executable(loop_text_save src/loop_text_save.cpp src/nvutils.cpp
#        ${rgst_src}
#        #        src/registrator/GloballyOptimalIcp.cpp
#        )
#target_include_directories(loop_text_save
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(loop_text_save
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#
#add_executable(save_fpfh src/save_fpfh.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(save_fpfh
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(save_fpfh
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#
#add_executable(save_fpfh_for_loop src/save_fpfh_for_loop.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(save_fpfh_for_loop
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(save_fpfh_for_loop
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#add_executable(viz_feature_matching src/viz_for_paper/viz_feature_matching.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(viz_feature_matching
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(viz_feature_matching
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#add_executable(viz_gicp src/viz_for_paper/viz_for_video_gicp.cpp src/nvutils.cpp ${rgst_src})
#target_include_directories(viz_gicp
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(viz_gicp
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
#
#
#add_executable(main_benchmark src/benchmark.cpp src/nvutils.cpp ${rgst_src}
#        )
#target_include_directories(main_benchmark
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#target_link_libraries(main_benchmark
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        ceres
#        )
#
#add_executable(run_example examples/quatro_examples.cpp src/nvutils.cpp ${rgst_src}
#        )
#
#target_include_directories(run_example
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
#
#target_link_libraries(run_example
#        PUBLIC
#        ${HDF5_LIBRARIES}
#        ${PCL_LIBRARY_DIRS}
#        ${catkin_LIBRARIES}
#        Eigen3::Eigen teaserpp::teaser_registration teaserpp::teaser_io
#        stdc++fs
#        )
