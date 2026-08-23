#pragma once
#include <vector>
#include <opencv2/core.hpp>
#include<Eigen/Core>

extern const std::vector<cv::Point3f> anchor_inactive_points_world;//未激活的符叶的模型锚框点
extern const std::vector<cv::Point3f> anchor_big_power_rune_active_points_world;//大符已激活的模型锚框点
extern const std::vector<cv::Point3f> anchor_small_power_rune_active_points_world;//小符已激活的模型锚框点

extern const std::vector<Eigen::Vector3d> inactive_points_world;//未激活的符叶的模型点
extern const std::vector<Eigen::Vector3d> inactive_points_world112;//未激活的符叶的模型点112点采样
extern const std::vector<Eigen::Vector3d> big_power_rune_active_points_world;//大符已激活的模型点
extern const std::vector<Eigen::Vector3d> small_power_rune_active_points_world;//小符已激活的模型点

extern const std::vector<Eigen::Vector3d> plane_points;//用于拟合平面的点

extern const std::vector<Eigen::Vector3d> power_rune_box_exoskeleton_points;//符的外骨架(主要用于可视化)

//下面是新版本代码
extern const std::vector<Eigen::Vector3d> inactive_points_model_1;//未激活的点(1号位)
extern const std::vector<Eigen::Vector3d> inactive_points_model_2;//未激活的点(2号位)
extern const std::vector<Eigen::Vector3d> inactive_points_model_3;//未激活的点(3号位)
extern const std::vector<Eigen::Vector3d> inactive_points_model_4;//未激活的点(4号位)
extern const std::vector<Eigen::Vector3d> inactive_points_model_5;//未激活的点(5号位)

extern const std::vector<Eigen::Vector3d> small_power_rune_active_armor_model_1;//小符已激活的点(装甲板)(1号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_armor_model_2;//小符已激活的点(装甲板)(2号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_armor_model_3;//小符已激活的点(装甲板)(3号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_armor_model_4;//小符已激活的点(装甲板)(4号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_armor_model_5;//小符已激活的点(装甲板)(5号位)

extern const std::vector<Eigen::Vector3d> small_power_rune_active_light_arm_model_1;//小符已激活的点(灯臂)(1号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_light_arm_model_2;//小符已激活的点(灯臂)(2号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_light_arm_model_3;//小符已激活的点(灯臂)(3号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_light_arm_model_4;//小符已激活的点(灯臂)(4号位)
extern const std::vector<Eigen::Vector3d> small_power_rune_active_light_arm_model_5;//小符已激活的点(灯臂)(5号位)

extern const std::vector<Eigen::Vector3d> big_power_rune_active_points_model_1;//大符已激活的点(1号位)
extern const std::vector<Eigen::Vector3d> big_power_rune_active_points_model_2;//大符已激活的点(2号位)
extern const std::vector<Eigen::Vector3d> big_power_rune_active_points_model_3;//大符已激活的点(3号位)
extern const std::vector<Eigen::Vector3d> big_power_rune_active_points_model_4;//大符已激活的点(4号位)
extern const std::vector<Eigen::Vector3d> big_power_rune_active_points_model_5;//大符已激活的点(5号位)
