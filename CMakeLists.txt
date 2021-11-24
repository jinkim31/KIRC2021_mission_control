cmake_minimum_required(VERSION 3.0.2)
project(mission_control)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        tf
        sequence
)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message (STATUS "OPENMP FOUND")
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
    set (OpenMP_FLAGS ${OpenMP_CXX_FLAGS})
    set (OpenMP_LIBRARIES gomp)
endif()


catkin_package(
  DEPENDS roscpp std_msgs OpenMP
)

include_directories(${catkin_INCLUDE_DIRS})
add_executable(mission_control
        src/main.cpp
        Algorithm/algorithm.cpp
        Algorithm/algorithm.h
        LidarProcessing2D/lidar_2d_processing.cpp
        Algorithm/custom_blocks.cpp
        Algorithm/custom_blocks.h
        )
target_compile_options(mission_control PRIVATE ${OpenMP_FLAGS})
add_dependencies(mission_control ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mission_control ${catkin_LIBRARIES} ${OpenMP_LIBS})