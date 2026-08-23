cmake_minimum_required(VERSION 3.22.0)

set(lib_name NNdetect)

find_package(OpenVINO REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(${lib_name}_lib SHARED src/NNDetector.cpp)
target_compile_features(${lib_name}_lib PRIVATE cxx_std_20)
target_include_directories(${lib_name}_lib PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)
target_link_libraries(
    ${lib_name}_lib
    PRIVATE
        app_interface_lib
        class_loader_lib
        config_loader_lib
        google_logger_lib
        img_viz_lib
        openvino::runtime
        opencv_core
        opencv_imgproc
)
