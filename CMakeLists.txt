cmake_minimum_required(VERSION 3.8)
project(point_pillars_ros)

# set(CMAKE_MODULE_PATH ./build_flags/cmake)
# set flags for CUDA availability
option(CUDA_AVAIL "CUDA available" OFF)
find_package(CUDA)
find_package(yaml-cpp REQUIRED)

if (CUDA_FOUND)
  message("CUDA is available!")
  message("CUDA Libs: ${CUDA_LIBRARIES}")
  message("CUDA Headers: ${CUDA_INCLUDE_DIRS}")
  set(CUDA_AVAIL ON)
else()
  message("CUDA NOT FOUND OR INCOMPATIBLE CMAKE VERSION FOUND")
  set(CUDA_AVAIL OFF)
endif ()

# set flags for TensorRT availability
option(TRT_AVAIL "TensorRT available" OFF)
# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
find_library(NVONNXPARSERS NAMES nvonnxparser)

if(NVINFER AND NVPARSERS AND NVONNXPARSERS)
  message("TensorRT is available!")
  message("NVINFER: ${NVINFER}")
  message("NVPARSERS: ${NVPARSERS}")
  message("NVONNXPARSERS: ${NVONNXPARSERS}")
  set(TRT_AVAIL ON)
else()
  message("TensorRT is NOT Available")
  set(TRT_AVAIL OFF)
endif()

# set flags for CUDNN availability
option(CUDNN_AVAIL "CUDNN available" OFF)
# try to find the CUDNN module
find_library(CUDNN_LIBRARY
  NAMES libcudnn.so${__cudnn_ver_suffix} libcudnn${__cudnn_ver_suffix}.dylib ${__cudnn_lib_win_name}
  PATHS $ENV{LD_LIBRARY_PATH} ${__libpath_cudart} ${CUDNN_ROOT_DIR} ${PC_CUDNN_LIBRARY_DIRS} ${CMAKE_INSTALL_PREFIX}
  PATH_SUFFIXES lib lib64 bin
  DOC "CUDNN library."
)

if(CUDNN_LIBRARY)
  message("CUDNN is available!")
  message("CUDNN_LIBRARY: ${CUDNN_LIBRARY}")
  set(CUDNN_AVAIL ON)
else()
  message("CUDNN is NOT Available")
  set(CUDNN_AVAIL OFF)
endif()

if(TRT_AVAIL AND CUDA_AVAIL AND CUDNN_AVAIL)
  find_package(ament_cmake_auto REQUIRED)
  find_package(PCL REQUIRED)
  ament_auto_find_build_dependencies()

  include_directories(
    include/point_pillars_ros
    ${CUDA_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
  )

  ament_auto_add_library(point_pillars_lib SHARED
    src/point_pillars.cpp
  )

  cuda_add_library(point_pillars_cuda_lib SHARED
    src/boolvfe.cu
    src/multi_head.cu
    src/nms.cu
    src/pp_preprocess.cu
    src/pp_scatter.cu
  )

  target_link_libraries(point_pillars_lib
    ${NVINFER}
    ${NVONNXPARSERS}
    ${CUDA_LIBRARIES}
    ${CUBLAS_LIBRARIES}
    ${CUDA_curand_LIBRARY}
    ${CUDNN_LIBRARY}
    yaml-cpp
    point_pillars_cuda_lib
  )

  target_include_directories(point_pillars_lib
    PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
  )

  target_include_directories(point_pillars_lib
    SYSTEM PUBLIC
      ${CUDA_INCLUDE_DIRS}
  )

  ament_auto_add_library(${PROJECT_NAME}_component SHARED
    src/${PROJECT_NAME}_component.cpp
  )
  rclcpp_components_register_nodes(${PROJECT_NAME}_component "PointPillarsROS")
  target_compile_options(${PROJECT_NAME}_component PUBLIC -Wall)
  ament_auto_add_executable(${PROJECT_NAME} src/main.cpp)

  target_include_directories(${PROJECT_NAME}_component PRIVATE ${PCL_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME}_component ${PCL_LIBRARIES})
  target_link_libraries(${PROJECT_NAME}_component point_pillars_lib)
  target_compile_definitions(${PROJECT_NAME}_component PRIVATE ${PCL_DEFINITIONS})

  ament_auto_package(
    INSTALL_TO_SHARE
    config
  )
  
  install(
    TARGETS point_pillars_cuda_lib
    DESTINATION lib
  )
else()
  find_package(ament_cmake_auto REQUIRED)
  ament_auto_find_build_dependencies()

  ament_auto_package(
    INSTALL_TO_SHARE
      config
  )
  message("PointPillars won't be built, CUDA and/or TensorRT were not found.")
endif()
