#新打符模块的camkelists
cmake_minimum_required(VERSION 3.22.0)
set(lib_name power_rune)


#找包
find_package(OpenCV REQUIRED)#opencv
find_package(Eigen3 REQUIRED)#Eigen
find_package(Ceres REQUIRED)#ceres
find_package(glog REQUIRED)#glog
find_package(Boost REQUIRED)#boost



#收集源文件fire_control_system_SRC
file(GLOB_RECURSE power_rune_src ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c)
list(APPEND power_rune_src ${CMAKE_CURRENT_SOURCE_DIR}/interface/power_rune_interface.cpp)

#生成接口库
add_library(${lib_name}_interface_lib INTERFACE)
target_include_directories(${lib_name}_interface_lib INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/interface
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(${lib_name}_interface_lib INTERFACE ${OpenCV_LIBS})#interface
target_link_libraries(${lib_name}_interface_lib INTERFACE time_lib)#Eigen
target_link_libraries(${lib_name}_interface_lib INTERFACE transform_tools_lib)#TFTree


#生成动态库
add_library(${lib_name}_lib SHARED ${power_rune_src})

#头文件路径
target_include_directories(${lib_name}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/config)#自己的头文件
target_include_directories(${lib_name}_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)#自己的头文件
target_include_directories(${lib_name}_lib PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/interface)

target_include_directories(${lib_name}_lib PUBLIC ${EIGEN3_INCLUDE_DIRS})#Eigen头文件
target_include_directories(${lib_name}_lib PUBLIC ${Boost_INCLUDE_DIRS})#boost头文件

#链接库
target_link_libraries(${lib_name}_lib PRIVATE time_lib)#时间戳
target_link_libraries(${lib_name}_lib PRIVATE Eigen3::Eigen)#Eigen
target_link_libraries(${lib_name}_lib PUBLIC Ceres::ceres)#ceres
target_link_libraries(${lib_name}_lib PRIVATE ${OpenCV_LIBS})#opencv
target_link_libraries(${lib_name}_lib PRIVATE threads_lib)#线程相关
target_link_libraries(${lib_name}_lib PRIVATE img_viz_lib)#图像可视化
target_link_libraries(${lib_name}_lib PUBLIC transform_tools_lib)#坐标变换
target_link_libraries(${lib_name}_lib PUBLIC function_lib)#临时对接用
target_link_libraries(${lib_name}_lib PUBLIC config_loader_lib)#配置文件读取
target_link_libraries(${lib_name}_lib PUBLIC foxglove_viz)#foxglove可视化发布
target_link_libraries(${lib_name}_lib PUBLIC foxglove_interface_lib)#Foxglove FlatBuffers 类型
target_link_libraries(${lib_name}_lib PRIVATE glog::glog)#glog
target_link_libraries(${lib_name}_lib PRIVATE ${Boost_LIBRARIES})#boost

target_link_libraries(${lib_name}_interface_lib INTERFACE ${lib_name}_lib)
