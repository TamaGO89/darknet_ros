cmake_minimum_required(VERSION 2.8.12)
project(darknet_ros)

set(BUILD_AS_CPP ON)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/darknet ${CMAKE_BINARY_DIR}/darknet)

# Set c++11 cmake flags
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
set(CMAKE_C_FLAGS "-Wall -Wno-unused-result -Wno-unknown-pragmas -Wno-unused-variable -Wfatal-errors -fPIC ${CMAKE_C_FLAGS}")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Define path of darknet folder here.
find_path(DARKNET_PATH
  NAMES "README.md"
  HINTS "${CMAKE_CURRENT_SOURCE_DIR}/darknet/")
message(STATUS "Darknet path dir = ${DARKNET_PATH}")
add_definitions(-DDARKNET_FILE_PATH="${DARKNET_PATH}")

# Find CUDA
find_package(CUDA QUIET)
if (CUDA_FOUND)
  find_package(CUDA REQUIRED)
  message(STATUS "CUDA Version: ${CUDA_VERSION_STRINGS}")
  message(STATUS "CUDA Libararies: ${CUDA_LIBRARIES}")
  set(
    CUDA_NVCC_FLAGS
    ${CUDA_NVCC_FLAGS};
    -O3
    -gencode arch=compute_30,code=sm_30
    -gencode arch=compute_35,code=sm_35
    -gencode arch=compute_50,code=[sm_50,compute_50]
    -gencode arch=compute_52,code=[sm_52,compute_52]
    -gencode arch=compute_61,code=sm_61
    -gencode arch=compute_62,code=sm_62
  )
  add_definitions(-DGPU)
else()
  list(APPEND LIBRARIES "m")
endif()

# Find X11
message ( STATUS "Searching for X11..." )
find_package ( X11 REQUIRED )
if ( X11_FOUND )
    include_directories (SYSTEM ${X11_INCLUDE_DIR} )
  link_libraries ( ${X11_LIBRARIES} )
  message ( STATUS " X11_INCLUDE_DIR: " ${X11_INCLUDE_DIR} )
  message ( STATUS " X11_LIBRARIES: " ${X11_LIBRARIES} )
endif ( X11_FOUND )

# Find rquired packeges
find_package(Boost REQUIRED COMPONENTS thread)
find_package(OpenCV REQUIRED)
include_directories(SYSTEM ${OpenCV_INCLUDE_DIRS})
find_package(catkin REQUIRED
  COMPONENTS
    cv_bridge
    roscpp
    rospy
    std_msgs
    actionlib
    image_transport
    # Messages
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    message_generation
)

add_message_files(
  FILES
    BoundingBox.msg
    BoundingBoxes.msg
    ObjectCount.msg
)
add_action_files(
  FILES
    CheckForObjects.action
)
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    std_msgs
)

# Enable OPENCV in darknet
add_definitions(-DOPENCV)
add_definitions(-O4 -g)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    actionlib
    rospy
    std_msgs
    image_transport
    # Messages
    actionlib_msgs
    geometry_msgs
    sensor_msgs
    message_runtime
  DEPENDS
    Boost
)

include_directories(
  include
)
include_directories(SYSTEM
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(PROJECT_LIB_FILES
    src/YoloObjectDetector.cpp
    src/image_interface.cpp)
set(DARKNET_CORE_HEADERS
    ${DARKNET_PATH}/include/darknet.h             ${DARKNET_PATH}/include/yolo_v2_class.hpp
    ${DARKNET_PATH}/src/activation_layer.h        ${DARKNET_PATH}/src/im2col.h
    ${DARKNET_PATH}/src/activations.h             ${DARKNET_PATH}/src/image.h
    ${DARKNET_PATH}/src/avgpool_layer.h           ${DARKNET_PATH}/src/layer.h
    ${DARKNET_PATH}/src/batchnorm_layer.h         ${DARKNET_PATH}/src/list.h
    ${DARKNET_PATH}/src/blas.h                    ${DARKNET_PATH}/src/local_layer.h
    ${DARKNET_PATH}/src/box.h                     ${DARKNET_PATH}/src/lstm_layer.h
    ${DARKNET_PATH}/src/col2im.h                  ${DARKNET_PATH}/src/matrix.h
    ${DARKNET_PATH}/src/connected_layer.h         ${DARKNET_PATH}/src/maxpool_layer.h
    ${DARKNET_PATH}/src/convolutional_layer.h     ${DARKNET_PATH}/src/network.h
    ${DARKNET_PATH}/src/cost_layer.h              ${DARKNET_PATH}/src/normalization_layer.h
    ${DARKNET_PATH}/src/crnn_layer.h              ${DARKNET_PATH}/src/option_list.h
    ${DARKNET_PATH}/src/crop_layer.h              ${DARKNET_PATH}/src/parser.h
    ${DARKNET_PATH}/src/dark_cuda.h               ${DARKNET_PATH}/src/region_layer.h
    ${DARKNET_PATH}/src/data.h                    ${DARKNET_PATH}/src/reorg_layer.h
    ${DARKNET_PATH}/src/deconvolutional_layer.h   ${DARKNET_PATH}/src/rnn_layer.h
    ${DARKNET_PATH}/src/demo.h                    ${DARKNET_PATH}/src/route_layer.h
    ${DARKNET_PATH}/src/detection_layer.h         ${DARKNET_PATH}/src/shortcut_layer.h
    ${DARKNET_PATH}/src/dropout_layer.h           ${DARKNET_PATH}/src/softmax_layer.h
    ${DARKNET_PATH}/src/gemm.h                    ${DARKNET_PATH}/src/tree.h
    ${DARKNET_PATH}/src/gru_layer.h               ${DARKNET_PATH}/src/utils.h
    ${DARKNET_PATH}/src/upsample_layer.h          ${DARKNET_PATH}/src/yolo_layer.h
    ${DARKNET_PATH}/src/version.h                 ${DARKNET_PATH}/src/scale_channels_layer.h
    ${DARKNET_PATH}/src/sam_layer.h               ${DARKNET_PATH}/src/reorg_old_layer.h
    ${DARKNET_PATH}/src/image_opencv.h            ${DARKNET_PATH}/src/httplib.h
    ${DARKNET_PATH}/src/http_stream.h             ${DARKNET_PATH}/src/gettimeofday.h
    ${DARKNET_PATH}/src/getopt.h                  ${DARKNET_PATH}/src/darkunistd.h
    ${DARKNET_PATH}/src/conv_lstm_layer.h         ${DARKNET_PATH}/src/classifier.h)
set(DARKNET_CORE_FILES
    ${DARKNET_PATH}/src/activation_layer.c        ${DARKNET_PATH}/src/activations.c       ${DARKNET_PATH}/src/art.c                     ${DARKNET_PATH}/src/avgpool_layer.c
    ${DARKNET_PATH}/src/captcha.c                 ${DARKNET_PATH}/src/cifar.c             ${DARKNET_PATH}/src/batchnorm_layer.c         ${DARKNET_PATH}/src/list.c
    ${DARKNET_PATH}/src/blas.c                    ${DARKNET_PATH}/src/local_layer.c       ${DARKNET_PATH}/src/box.c                     ${DARKNET_PATH}/src/lstm_layer.c
    ${DARKNET_PATH}/src/classifier.c              ${DARKNET_PATH}/src/matrix.c            ${DARKNET_PATH}/src/coco.c                    ${DARKNET_PATH}/src/maxpool_layer.c
    ${DARKNET_PATH}/src/col2im.c                  ${DARKNET_PATH}/src/network.c           ${DARKNET_PATH}/src/compare.c                 ${DARKNET_PATH}/src/normalization_layer.c
    ${DARKNET_PATH}/src/connected_layer.c         ${DARKNET_PATH}/src/option_list.c       ${DARKNET_PATH}/src/conv_lstm_layer.c         ${DARKNET_PATH}/src/parser.c
    ${DARKNET_PATH}/src/convolutional_layer.c     ${DARKNET_PATH}/src/region_layer.c      ${DARKNET_PATH}/src/cost_layer.c              ${DARKNET_PATH}/src/reorg_layer.c
    ${DARKNET_PATH}/src/cpu_gemm.c                ${DARKNET_PATH}/src/rnn_vid.c           ${DARKNET_PATH}/src/crnn_layer.c              ${DARKNET_PATH}/src/rnn.c
    ${DARKNET_PATH}/src/crop_layer.c              ${DARKNET_PATH}/src/sam_layer.c         ${DARKNET_PATH}/src/dark_cuda.c               ${DARKNET_PATH}/src/scale_channels_layer.c
    ${DARKNET_PATH}/src/darknet.c                 ${DARKNET_PATH}/src/shortcut_layer.c    ${DARKNET_PATH}/src/data.c                    ${DARKNET_PATH}/src/super.c
    ${DARKNET_PATH}/src/deconvolutional_layer.c   ${DARKNET_PATH}/src/rnn_layer.c         ${DARKNET_PATH}/src/demo.c                    ${DARKNET_PATH}/src/route_layer.c
    ${DARKNET_PATH}/src/detection_layer.c         ${DARKNET_PATH}/src/swag.c              ${DARKNET_PATH}/src/detector.c                ${DARKNET_PATH}/src/tag.c
    ${DARKNET_PATH}/src/dice.c                    ${DARKNET_PATH}/src/voxel.c             ${DARKNET_PATH}/src/dropout_layer.c           ${DARKNET_PATH}/src/softmax_layer.c
    ${DARKNET_PATH}/src/gaussian_yolo_layer.c     ${DARKNET_PATH}/src/writing.c           ${DARKNET_PATH}/src/getopt.c                  ${DARKNET_PATH}/src/nightmare.c
    ${DARKNET_PATH}/src/gettimeofday.c            ${DARKNET_PATH}/src/yolo_v2_class.cpp   ${DARKNET_PATH}/src/gemm.c                    ${DARKNET_PATH}/src/tree.c
    ${DARKNET_PATH}/src/go.c                      ${DARKNET_PATH}/src/yolo.c              ${DARKNET_PATH}/src/gru_layer.c               ${DARKNET_PATH}/src/utils.c
    ${DARKNET_PATH}/src/http_stream.cpp           ${DARKNET_PATH}/src/reorg_old_layer.c   ${DARKNET_PATH}/src/im2col.c                  ${DARKNET_PATH}/src/image_opencv.cpp
    ${DARKNET_PATH}/src/image.c                   ${DARKNET_PATH}/src/layer.c             ${DARKNET_PATH}/src/upsample_layer.c          ${DARKNET_PATH}/src/yolo_layer.c)
set(DARKNET_CUDA_FILES
    ${DARKNET_PATH}/src/activation_kernels.cu
    ${DARKNET_PATH}/src/avgpool_layer_kernels.cu
    ${DARKNET_PATH}/src/blas_kernels.cu
    ${DARKNET_PATH}/src/col2im_kernels.cu
    ${DARKNET_PATH}/src/convolutional_kernels.cu
    ${DARKNET_PATH}/src/crop_layer_kernels.cu
    ${DARKNET_PATH}/src/deconvolutional_kernels.cu
    ${DARKNET_PATH}/src/dropout_layer_kernels.cu
    ${DARKNET_PATH}/src/im2col_kernels.cu
    ${DARKNET_PATH}/src/maxpool_layer_kernels.cu
    ${DARKNET_PATH}/src/network_kernels.cu)

if (CUDA_FOUND)

  link_directories(
    ${CUDA_TOOLKIT_ROOT_DIR}/lib64)

  cuda_add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES}
    ${DARKNET_CORE_FILES}
    ${DARKNET_CUDA_FILES})
  target_link_libraries(${PROJECT_NAME}_lib
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    cuda
    cudart
    cublas
    curand
    dark)
  cuda_add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp)

else()

  add_library(${PROJECT_NAME}_lib
    ${PROJECT_LIB_FILES}
    ${DARKNET_CORE_FILES})
  target_link_libraries(${PROJECT_NAME}_lib
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
    dark)
  add_executable(${PROJECT_NAME}
    src/yolo_object_detector_node.cpp)

endif()

target_link_libraries(${PROJECT_NAME}
  m
  pthread
  stdc++
  ${Boost_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  dark
  ${PROJECT_NAME}_lib
)

install(TARGETS ${PROJECT_NAME}_lib
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(
  DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(
  DIRECTORY ${DARKNET_PATH}/src/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(DIRECTORY config launch yolo_network_config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Download yolov2-tiny.weights
set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
set(FILE "${PATH}/yolov2-tiny.weights")
message(STATUS "Checking and downloading yolov2-tiny.weights if needed ...")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget -q https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov2-tiny.weights -P ${PATH})
endif()

# Download yolov3.weights
set(FILE "${PATH}/yolov3.weights")
message(STATUS "Checking and downloading yolov3.weights if needed ...")
if (NOT EXISTS "${FILE}")
  message(STATUS "... file does not exist. Downloading now ...")
  execute_process(COMMAND wget -q https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov3.weights -P ${PATH})
endif()

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  # Download yolov2.weights
  set(PATH "${CMAKE_CURRENT_SOURCE_DIR}/yolo_network_config/weights")
  set(FILE "${PATH}/yolov2.weights")
  message(STATUS "Checking and downloading yolov2.weights if needed ...")
  if (NOT EXISTS "${FILE}")
    message(STATUS "... file does not exist. Downloading now ...")
    execute_process(COMMAND wget -q https://github.com/leggedrobotics/darknet_ros/releases/download/1.1.4/yolov2.weights -P ${PATH})
  endif()

  find_package(rostest REQUIRED)

  # Object detection in images.
  add_rostest_gtest(${PROJECT_NAME}_object_detection-test
    test/object_detection.test
    test/test_main.cpp
    test/ObjectDetection.cpp
  )
  target_link_libraries(${PROJECT_NAME}_object_detection-test
    ${catkin_LIBRARIES}
    dark
  )
endif()

#########################
###   CLANG TOOLING   ###
#########################
find_package(cmake_clang_tools QUIET)
if (cmake_clang_tools_FOUND)
  message(STATUS "Run clang tooling")
  add_clang_tooling(
    TARGETS ${PROJECT_NAME}
    SOURCE_DIRS ${CMAKE_CURRENT_LIST_DIR}/src ${CMAKE_CURRENT_LIST_DIR}/include ${CMAKE_CURRENT_LIST_DIR}/test
    CT_HEADER_DIRS ${CMAKE_CURRENT_LIST_DIR}/include
    CF_WERROR
  )
endif (cmake_clang_tools_FOUND)
