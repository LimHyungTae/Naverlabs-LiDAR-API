cmake_minimum_required(VERSION 3.0.2)
project(naverlabs_api)

set(CMAKE_CXX_STANDARD 17) # 17: to use filesystem

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif ()

find_package(hdf5 COMPONENTS CXX QUIET)
if (NOT hdf5_FOUND)
    find_package(HDF5 COMPONENTS CXX REQUIRED)
    if (HDF5_DIR)
        set(HDF5_INCLUDE_DIRS ${HDF5_DIR})
    endif ()
    if (CMAKE_CROSSCOMPILING)
        set(HDF5_INCLUDE_DIRS ${CMAKE_SYSROOT}/${HDF5_INCLUDE_DIRS})
        set(HDF5_INCLUDE_DIR ${CMAKE_SYSROOT}/${HDF5_INCLUDE_DIRS})
    endif ()
    if (NOT CMAKE_CROSSCOMPILING)
        set(HDF5_LIBRARIES
                ${HDF5_hdf5_LIBRARIES_RELEASE}
                ${HDF5_hdf5_cpp_LIBRARY_RELEASE}
                ${HDF5_CXX_LIBRARY_hdf5}
                ${HDF5_CXX_LIBRARY_hdf5_cpp}
                )
    else ()
        set(HDF5_LIB_DIR ${CMAKE_SYSROOT}/usr/lib/aarch64-linux-gnu/hdf5/serial)
        set(HDF5_LIBRARIES ${HDF5_LIB_DIR}/libhdf5.so ${HDF5_LIB_DIR}/libhdf5_cpp.so)
    endif ()

    include_directories(${HDF5_INCLUDE_DIRS})
else ()
    if (hdf5_DIR)
        set(HDF5_INCLUDE_DIRS ${hdf5_DIR})
    endif ()
    set(HDF5_LIBRARIES hdf5-shared hdf5_cpp-shared)
endif ()

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
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif ()

add_message_files(
        FILES
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
)


include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

include_directories(
        include
        src
        ${CERES_INCLUDE_DIRS}
        ${catkin_INCLUDE_DIRS}
)

## benchmark
add_executable(pub_lidar src/data_publisher.cpp src/nvutils.cpp)
target_link_libraries(pub_lidar
        PUBLIC
        ${HDF5_LIBRARIES}
        ${PCL_LIBRARY_DIRS}
        ${catkin_LIBRARIES}
        stdc++fs
        )
#target_include_directories(pub_lidar
#        PUBLIC
#        ${HDF5_INCLUDE_DIR}
#        ${PCL_INCLUDE_DIRS}
#        )
add_dependencies(pub_lidar naverlabs_api_generate_messages_cpp)

