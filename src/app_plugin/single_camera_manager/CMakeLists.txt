set(CAMERA_MANAGER_PLUGIN_TARGET single_camera_manager_plugin_lib)

find_package(OpenCV REQUIRED COMPONENTS core videoio)

add_library(${CAMERA_MANAGER_PLUGIN_TARGET} SHARED
    src/ReplayCameraManager.cpp
)

target_include_directories(${CAMERA_MANAGER_PLUGIN_TARGET} PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/config
)

target_link_libraries(${CAMERA_MANAGER_PLUGIN_TARGET} PUBLIC
    app_interface_lib
    class_loader_lib
    config_loader_lib
    google_logger_lib
    opencv_videoio
)
