set(FIRE_CONTROL_SYSTEM_PLUGIN_TARGET fire_control_system_plugin_lib)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)

add_library(${FIRE_CONTROL_SYSTEM_PLUGIN_TARGET} SHARED
    src/RunePlannerControl.cpp
)

target_include_directories(${FIRE_CONTROL_SYSTEM_PLUGIN_TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

target_link_libraries(${FIRE_CONTROL_SYSTEM_PLUGIN_TARGET} PUBLIC
    app_interface_lib
    class_loader_lib
    config_loader_lib
    google_logger_lib
    power_rune_interface_lib
    img_viz_lib
    opencv_imgproc
)
