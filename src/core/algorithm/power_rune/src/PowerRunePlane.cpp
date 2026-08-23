#include <chrono>
#include <cmath>
#include <random>
#include <algorithm>
#include <array>
#include <iostream>
#include <optional>
#include <string>
#include <utility>

#include "PowerRunePlane.hpp"
#include "json.hpp"
#include "img_viz.hpp"
#include "PnPVariable.hpp"
#include "common/points_world.hpp"
#include "common/power_rune_function.hpp"
#include "function.hpp"
#include "common/PowerRuneDiagnostics.hpp"
#include "common/PowerRuneVisualizeManager.hpp"

namespace
{
//根据网络结果补充未激活目标
//TODO：将这几个函数整合为成员函数

bool is_big_inactive_blade(const SingleRuneBlade2D &single_rune_blade_2D)
{
    return single_rune_blade_2D.rune_state == RuneState::BigInactive;
}

bool is_big_inactive_blade_usable_for_projection(const SingleRuneBlade2D &single_rune_blade_2D)
{
    return is_big_inactive_blade(single_rune_blade_2D)
        && single_rune_blade_2D.is_armor_module_usable
        && single_rune_blade_2D.is_light_arm_usable
        && single_rune_blade_2D.is_center_R_usable;
}

std::optional<Eigen::Vector2d> calculate_nn_direction_vector(const SingleRuneBlade2D &single_rune_blade_2D)
{
    const RuneInfo &rune_info = single_rune_blade_2D.rune_info;
    // 网络侧没有传统轮廓时，用五点关键点近似“符心 -> 靶心”的方向。
    const cv::Point2f armor_center =
        (cv::Point2f(rune_info.top) + cv::Point2f(rune_info.left) + cv::Point2f(rune_info.right) + cv::Point2f(rune_info.bottom)) * 0.25f;
    const cv::Point2f rune_center = rune_info.point_R;

    Eigen::Vector2d direction_vector(
        static_cast<double>(armor_center.x - rune_center.x),
        static_cast<double>(armor_center.y - rune_center.y));
    const double norm = direction_vector.norm();
    if (norm <= 1e-6)
    {
        return std::nullopt;
    }

    return direction_vector / norm;
}

std::optional<int> try_infer_supplemented_big_inactive_location_num(const RefinedRuneObservation &refined_rune_observation,
                                                                    const std::vector<int> &inactive_target_location_nums)
{
    // 只处理“大符网络识别到两片未激活，但传统投影只保留了1号位”这一条补全链路。
    if (!refined_rune_observation.is_big_rune || inactive_target_location_nums.size() != 1 || inactive_target_location_nums.front() != 1)
    {
        return std::nullopt;
    }

    std::vector<const SingleRuneBlade2D *> big_inactive_blades;
    std::vector<const SingleRuneBlade2D *> usable_big_inactive_blades;
    for (const auto &single_rune_blade_2D : refined_rune_observation.rune_blade_2D)
    {
        if (!is_big_inactive_blade(single_rune_blade_2D))
        {
            continue;
        }

        big_inactive_blades.emplace_back(&single_rune_blade_2D);
        if (is_big_inactive_blade_usable_for_projection(single_rune_blade_2D))
        {
            usable_big_inactive_blades.emplace_back(&single_rune_blade_2D);
        }
    }

    if (big_inactive_blades.size() != 2 || usable_big_inactive_blades.size() != 1)
    {
        return std::nullopt;
    }

    const SingleRuneBlade2D *reference_blade = usable_big_inactive_blades.front();
    const SingleRuneBlade2D *supplemented_blade = nullptr;
    for (const SingleRuneBlade2D *blade_ptr : big_inactive_blades)
    {
        if (blade_ptr != reference_blade)
        {
            supplemented_blade = blade_ptr;
            break;
        }
    }

    if (supplemented_blade == nullptr)
    {
        return std::nullopt;
    }

    const std::optional<Eigen::Vector2d> reference_direction_opt = calculate_nn_direction_vector(*reference_blade);
    const std::optional<Eigen::Vector2d> supplemented_direction_opt = calculate_nn_direction_vector(*supplemented_blade);
    if (!reference_direction_opt.has_value() || !supplemented_direction_opt.has_value())
    {
        return std::nullopt;
    }

    const Eigen::Vector2d &reference_direction = reference_direction_opt.value();
    const Eigen::Vector2d &supplemented_direction = supplemented_direction_opt.value();
    // 以可投影的未激活目标为参考，量化另一片网络目标相对它的扇区位置。
    double theta = std::atan2(
        reference_direction.x() * supplemented_direction.y() - reference_direction.y() * supplemented_direction.x(),
        reference_direction.dot(supplemented_direction));

    constexpr std::array<std::pair<int, double>, 4> kLocationAngles = {{
        {2, 0.2 * CV_2PI},
        {3, 0.4 * CV_2PI},
        {4, 0.6 * CV_2PI},
        {5, 0.8 * CV_2PI},
    }};
    const double half_interval = static_cast<double>(J_POWER_RUNE.config_["project"]["half_interval"]);
    for (const auto &[location_num, location_theta] : kLocationAngles)
    {
        if (std::abs(PRF::calculate_delta_phase<PRF::rad>(theta, location_theta)) <= half_interval)
        {
            return location_num;
        }
    }

    return std::nullopt;
}

} // namespace

PowerRunePlane::PowerRunePlane()
{
    //初始化模型点
    m_anchor_inactive_points_world_ptr = std::make_shared<std::vector<cv::Point3f>>(anchor_inactive_points_world);
    m_plane_points_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(plane_points);
    m_power_rune_box_exoskeleton_points_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(power_rune_box_exoskeleton_points);
}

void PowerRunePlane::set_debug_rune_targets(std::vector<RuneTarget> rune_targets)
{
    m_debug_rune_targets = std::move(rune_targets);
}

void PowerRunePlane::update_power_rune_plane(const RefinedRuneObservation &refined_rune_observation)
{
    m_inactive_targets.is_vaild = false;//默认目标无法使用

    //新的投影类计算姿态
    if(!m_projector.caculate_pose(refined_rune_observation.rune_blade_2D,
                                  refined_rune_observation.tf_tree,
                                  refined_rune_observation.ori_img))
    {
        // 说明计算失败
        return;
    }

    //获取姿态
    Eigen::Vector<double, 6> pose = m_projector.get_pose();
    Eigen::Vector3d rvec(pose[0], pose[1], pose[2]);
    Eigen::Vector3d tvec(pose[3], pose[4], pose[5]);

    //转相机系用
    double theta = rvec.norm();
    Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
    if (theta > 1e-12)
    {
        Eigen::Vector3d axis = rvec / theta;
        rotation = Eigen::AngleAxisd(theta, axis).toRotationMatrix();
    }

    // 转车系用
    transform_tools::TF tf_car_from_camera = refined_rune_observation.tf_tree.calculate_relative_TF(car_frame, camera_frame);
    Eigen::Matrix3d R_tf = tf_car_from_camera.get_rotation();
    Eigen::Vector3d T_tf = tf_car_from_camera.get_offset();
    T_tf *= 0.001;

    //构造所有未激活目标对应的平面点
    std::vector<std::vector<Eigen::Vector3d>> plane_points_vector;
    std::vector<int> inactive_target_location_nums = m_projector.get_inactive_target_location_nums();
    if (inactive_target_location_nums.empty())
    {
        inactive_target_location_nums.emplace_back(1);
    }
    else if (const std::optional<int> supplemented_location_num =
                 try_infer_supplemented_big_inactive_location_num(refined_rune_observation, inactive_target_location_nums);
             supplemented_location_num.has_value()
             && std::find(inactive_target_location_nums.begin(),
                          inactive_target_location_nums.end(),
                          supplemented_location_num.value()) == inactive_target_location_nums.end())
    {
        // 姿态仍然只由传统可投影目标给出，这里只补一个额外未激活目标进入后续相位/运动估计。
        inactive_target_location_nums.emplace_back(supplemented_location_num.value());
    }
    for (const int inactive_target_location_num : inactive_target_location_nums)
    {
        //根据位置进行旋转，构造该未激活目标对应的平面点
        std::vector<Eigen::Vector3d> plane_points = *(m_plane_points_ptr);
        const double theta_rotate = (inactive_target_location_num - 1) * (CV_2PI / 5.0);
        const double cos_theta = std::cos(theta_rotate);
        const double sin_theta = std::sin(theta_rotate);
        for (auto &point : plane_points)
        {
            const double x = point.x();
            const double y = point.y();
            point.x() = x * cos_theta - y * sin_theta;
            point.y() = x * sin_theta + y * cos_theta;
        }
        plane_points_vector.emplace_back(std::move(plane_points));
    }
    
    //转系
    for (auto &plane_points : plane_points_vector)
    {
        for (auto &plane_point : plane_points)
        {
            plane_point = rotation * plane_point + tvec;// 转相机系
            plane_point = R_tf * plane_point + T_tf;// 转车系
        }
    }

    Eigen::Vector3d model_plane_normal = Eigen::Vector3d(0, 0, 1); // 模型平面和模型平面的法向量(已弃用)
    model_plane_normal = rotation * model_plane_normal;// 转换到相机系
    model_plane_normal = R_tf * model_plane_normal;// 转车系

    if (VizTopic::PowerRuneCar::enabled())
    {
    // 车系下的三维重建
    constexpr size_t kMaxVisualizedInactiveTargets = 2;

    // 车系下模型平面的点
    std::array<foxglove::schemas::SpherePrimitive, 20> model_plane_points_car;
    for (auto &sphere : model_plane_points_car)
    {
        //默认所有的数值都是0
        sphere.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{0.0, 0.0, 0.0}};
        sphere.size = foxglove::schemas::Vector3{0.0, 0.0, 0.0};
        sphere.color = foxglove::schemas::Color{0.0f, 0.0f, 0.0f, 0.0f};
    }

    //根据plane_points_vector中的数据进行赋值
    int sphere_index = 0;
    for (const auto &plane_points: plane_points_vector)
    {
        for (const auto &plane_point: plane_points)
        {
            if (sphere_index >= static_cast<int>(model_plane_points_car.size()))
            {
                LOG(ERROR) << "[PowerRuneCar]超出array大小";
                ++sphere_index;
                continue;
            }

            model_plane_points_car[sphere_index].pose =
                foxglove::schemas::Pose{foxglove::schemas::Vector3{
                    plane_point.z(),
                    -plane_point.x(),
                    -plane_point.y()}};
            model_plane_points_car[sphere_index].size = foxglove::schemas::Vector3{0.05, 0.05, 0.05};
            // 符心和靶心用其他颜色
            if (sphere_index == 0 || sphere_index == 1 || sphere_index == 10 || sphere_index == 11)
            {
                model_plane_points_car[sphere_index].color = foxglove::schemas::Color{0, 0, 1, 1};
            }
            else
            {
                model_plane_points_car[sphere_index].color = foxglove::schemas::Color{1, 0, 0, 1};
            }
            sphere_index++;
        }
    }
    //可视化
    Viz::publish_spheres<VizTopic::PowerRuneCar>(model_plane_points_car);

    // 车系下法向量
    std::array<foxglove::schemas::ArrowPrimitive, kMaxVisualizedInactiveTargets> power_rune_normal_car;

    for (auto &arrow : power_rune_normal_car)
    {
        arrow.pose = foxglove::schemas::Pose{};
        arrow.pose->position = foxglove::schemas::Vector3{};
        arrow.pose->orientation = foxglove::schemas::Quaternion{};
        arrow.shaft_length = 0.0;
        arrow.shaft_diameter = 0.0;
        arrow.head_length = 0.0;
        arrow.head_diameter = 0.0;
        arrow.color = foxglove::schemas::Color{0.0f, 0.0f, 0.0f, 0.0f};
    }

    // 判断法向量方向
    bool is_facing_camera_car = model_plane_normal.z() > 0;
    foxglove::schemas::Color schemas_color_car = is_facing_camera_car ? foxglove::schemas::Color{1.0f, 0.0f, 0.0f, 1.0f}  // 红色：朝里
                                                                      : foxglove::schemas::Color{0.0f, 0.0f, 1.0f, 1.0f}; // 蓝色：朝外
    auto dirtion_vector2quaternion = [&](const Eigen::Vector3d &dir) -> foxglove::schemas::Quaternion
    {
        Eigen::Vector3d from(1.0, 0.0, 0.0);   // Foxglove 默认箭头方向 +X
        Eigen::Vector3d to = dir.normalized(); // 目标方向

        Eigen::Quaterniond q;
        if (to.dot(from) < -0.999999)
        {
            // 反向（180度）特判，任选一个不共线轴
            q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0, 1.0, 0.0));
        }
        else
        {
            q.setFromTwoVectors(from, to);
        }

        foxglove::schemas::Quaternion out;
        out.x = q.x();
        out.y = q.y();
        out.z = q.z();
        out.w = q.w();
        return out;
    };
    for (size_t target_idx = 0; target_idx < std::min(plane_points_vector.size(), kMaxVisualizedInactiveTargets); ++target_idx)
    {
        const auto &plane_points_car = plane_points_vector[target_idx];
        if (plane_points_car.size() <= 1)
        {
            continue;
        }

        // ------------------ 法向量 ------------------
        power_rune_normal_car[target_idx].pose->position->x = plane_points_car[1].z();
        power_rune_normal_car[target_idx].pose->position->y = -plane_points_car[1].x();
        power_rune_normal_car[target_idx].pose->position->z = -plane_points_car[1].y();

        power_rune_normal_car[target_idx].pose->orientation =
            dirtion_vector2quaternion(Eigen::Vector3d(
                model_plane_normal.z(),
                -model_plane_normal.x(),
                -model_plane_normal.y()));

        power_rune_normal_car[target_idx].shaft_length = 0.25;
        power_rune_normal_car[target_idx].shaft_diameter = 0.01;
        power_rune_normal_car[target_idx].head_length = 0.08;
        power_rune_normal_car[target_idx].head_diameter = 0.03;
        power_rune_normal_car[target_idx].color = schemas_color_car;
    }

    Viz::publish_arrows<VizTopic::PowerRuneNormalCar>(power_rune_normal_car);

    }

    if (VizTopic::PowerRuneCamera::enabled())
    {
    // 相机系下的三维重建
    // 相机系下模型平面的点
    const Eigen::Matrix3d R_camera_from_car = R_tf.transpose();
    std::vector<Eigen::Vector3d> plane_points_camera;
    if (plane_points_vector.empty())
    {
        LOG(ERROR) << "[PowerRuneCamera]plane_points_vector为空";
    }
    else
    {
        plane_points_camera.reserve(plane_points_vector.front().size());
        for (const auto &plane_point_car : plane_points_vector.front())
        {
            plane_points_camera.emplace_back(R_camera_from_car * (plane_point_car - T_tf));
        }
    }

    std::array<foxglove::schemas::SpherePrimitive, 10> model_plane_points_camera;
    for (auto &sphere : model_plane_points_camera)
    {
        sphere.pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{0.0, 0.0, 0.0}};
        sphere.size = foxglove::schemas::Vector3{0.0, 0.0, 0.0};
        sphere.color = foxglove::schemas::Color{0.0f, 0.0f, 0.0f, 0.0f};
    }

    const int camera_points_count = std::min(static_cast<int>(plane_points_camera.size()),
                                             static_cast<int>(model_plane_points_camera.size()));
    if (plane_points_camera.size() > model_plane_points_camera.size())
    {
        LOG(ERROR) << "[PowerRuneCamera]超出array大小";
    }
    for (int i = 0; i < camera_points_count; i++)
    {
        model_plane_points_camera[i].pose = foxglove::schemas::Pose{foxglove::schemas::Vector3{plane_points_camera[i].z(), -plane_points_camera[i].x(), -plane_points_camera[i].y()}};
        model_plane_points_camera[i].size = foxglove::schemas::Vector3{0.05, 0.05, 0.05};

        if (i == 0 || i == 1)
        {
            model_plane_points_camera[i].color = foxglove::schemas::Color{0, 0, 1, 1};
        }
        else
        {
            model_plane_points_camera[i].color = foxglove::schemas::Color{1, 0, 0, 1};
        }
    }
    if (plane_points_camera.size() < model_plane_points_camera.size())
    {
        LOG(ERROR) << "[PowerRuneCamera]点数不足array大小";
    }
    Viz::publish_spheres<VizTopic::PowerRuneCamera>(model_plane_points_camera);

    // 车系下法向量
    std::array<foxglove::schemas::ArrowPrimitive, 1> power_rune_normal_camera;
    for (int i = 0; i < 1; ++i)
    {
        power_rune_normal_camera[i].pose = foxglove::schemas::Pose{};
        power_rune_normal_camera[i].pose->position = foxglove::schemas::Vector3{};
        power_rune_normal_camera[i].pose->orientation = foxglove::schemas::Quaternion{};
    }

    // ------------------ 法向量 ------------------
    if (plane_points_camera.size() <= 1)
    {
        LOG(ERROR) << "[PowerRuneNormalCamera]点数不足";
    }
    else
    {
        power_rune_normal_camera[0].pose->position->x = plane_points_camera[1].z();
        power_rune_normal_camera[0].pose->position->y = -plane_points_camera[1].x();
        power_rune_normal_camera[0].pose->position->z = -plane_points_camera[1].y();

        Eigen::Vector3d model_plane_normal_camera = R_camera_from_car * model_plane_normal;
        auto dirtion_vector2quaternion = [&](const Eigen::Vector3d &dir) -> foxglove::schemas::Quaternion
        {
            Eigen::Vector3d from(1.0, 0.0, 0.0);   // Foxglove 默认箭头方向 +X
            Eigen::Vector3d to = dir.normalized(); // 目标方向

            Eigen::Quaterniond q;
            if (to.dot(from) < -0.999999)
            {
                // 反向（180度）特判，任选一个不共线轴
                q = Eigen::AngleAxisd(M_PI, Eigen::Vector3d(0.0, 1.0, 0.0));
            }
            else
            {
                q.setFromTwoVectors(from, to);
            }

            foxglove::schemas::Quaternion out;
            out.x = q.x();
            out.y = q.y();
            out.z = q.z();
            out.w = q.w();
            return out;
        };
        power_rune_normal_camera[0].pose->orientation =
            dirtion_vector2quaternion(Eigen::Vector3d(
                model_plane_normal_camera.z(),
                -model_plane_normal_camera.x(),
                -model_plane_normal_camera.y()));

        power_rune_normal_camera[0].shaft_length = 0.25;
        power_rune_normal_camera[0].shaft_diameter = 0.01;
        power_rune_normal_camera[0].head_length = 0.08;
        power_rune_normal_camera[0].head_diameter = 0.03;

        // 判断法向量方向
        bool is_facing_camera_camera = model_plane_normal_camera.z() > 0;
        foxglove::schemas::Color schemas_color_camera = is_facing_camera_camera ? foxglove::schemas::Color{1.0f, 0.0f, 0.0f, 1.0f}  // 红色：朝里
                                                                                : foxglove::schemas::Color{0.0f, 0.0f, 1.0f, 1.0f}; // 蓝色：朝外
        power_rune_normal_camera[0].color = schemas_color_camera;
    }

    Viz::publish_arrows<VizTopic::PowerRuneNormalCamera>(power_rune_normal_camera);
    }

    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD_REPROJECT_IMG"])
    {

    // 用于优化的点
    cv::Mat project_img = refined_rune_observation.ori_img.clone();
    std::vector<cv::Point3d> object_points;
    std::vector<cv::Point2d> image_points;

    std::vector<const Eigen::Vector3d *> sampled_model_points = m_projector.get_sampled_model_points();
    for (const auto &point : sampled_model_points)
    {
        Eigen::Vector3d object_point = rotation * (*point) + tvec;
        object_points.emplace_back(object_point.x(), object_point.y(), object_point.z());
    }

    // 外框架点
    size_t exoskeleton_start_idx = object_points.size();
    for (const auto &point : *m_power_rune_box_exoskeleton_points_ptr)
    {
        Eigen::Vector3d object_point = rotation * point + tvec;
        object_points.emplace_back(object_point.x(), object_point.y(), object_point.z());
    }

    // 法向量
    Eigen::Vector3d armor_center_normal_start = {0, -0.7, 0};
    Eigen::Vector3d armor_center_normal_end = {0, -0.7, 1};
    armor_center_normal_start = rotation * armor_center_normal_start + tvec;
    armor_center_normal_end = rotation * armor_center_normal_end + tvec;
    object_points.emplace_back(armor_center_normal_start.x(), armor_center_normal_start.y(), armor_center_normal_start.z());
    object_points.emplace_back(armor_center_normal_end.x(), armor_center_normal_end.y(), armor_center_normal_end.z());

    // 判断法向量是否朝向相机
    Eigen::Vector3d normal_cam = armor_center_normal_end - armor_center_normal_start;
    bool facing_camera = normal_cam.z() > 0;

    // 投影
    cv::projectPoints(object_points, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), CAM, DIS, image_points);

    // 可视化用于优化的点
    for (size_t i = 0; i < image_points.size() - 2; ++i)
    {
        cv::circle(project_img, image_points[i], 2, cv::Scalar(0, 255, 0), -1);
    }

	    // 可视化外框架：5个独立正方体，每8个点对应1个正方体
	    constexpr size_t exo_cube_count = 5;
	    constexpr size_t exo_points_per_cube = 8;
	    const size_t exo_total_points = exo_cube_count * exo_points_per_cube;
	    if (exoskeleton_start_idx + exo_total_points <= image_points.size())
	    {
	        const cv::Scalar cube_default_color(255, 255, 255);
	        const cv::Scalar cube_highlight_color(0, 0, 255);
	        const cv::Scalar center_color(0, 255, 255);
	        std::vector<cv::Point2d> cube_centers;
	        cube_centers.reserve(exo_cube_count);

	        std::array<bool, exo_cube_count> highlight_cube{};
	        for (const int inactive_target_location_num : inactive_target_location_nums)
	        {
	            const int highlight_idx = inactive_target_location_num - 1;
	            if (highlight_idx >= 0 && highlight_idx < static_cast<int>(exo_cube_count))
	            {
	                highlight_cube[static_cast<size_t>(highlight_idx)] = true;
	            }
	        }

	        for (size_t cube_idx = 0; cube_idx < exo_cube_count; ++cube_idx)
	        {
	            const cv::Scalar cube_color = highlight_cube[cube_idx] ? cube_highlight_color : cube_default_color;
	            const size_t base = exoskeleton_start_idx + cube_idx * exo_points_per_cube;
	            const size_t f_lt = base + 0;
	            const size_t f_rt = base + 1;
	            const size_t f_rb = base + 2;
            const size_t f_lb = base + 3;
            const size_t b_lt = base + 4;
            const size_t b_rt = base + 5;
            const size_t b_rb = base + 6;
            const size_t b_lb = base + 7;

            // 前面
	            cv::line(project_img, image_points[f_lt], image_points[f_rt], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[f_rt], image_points[f_rb], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[f_rb], image_points[f_lb], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[f_lb], image_points[f_lt], cube_color, 2, cv::LINE_AA);

	            // 后面
	            cv::line(project_img, image_points[b_lt], image_points[b_rt], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[b_rt], image_points[b_rb], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[b_rb], image_points[b_lb], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[b_lb], image_points[b_lt], cube_color, 2, cv::LINE_AA);

	            // 侧棱
	            cv::line(project_img, image_points[f_lt], image_points[b_lt], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[f_rt], image_points[b_rt], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[f_rb], image_points[b_rb], cube_color, 2, cv::LINE_AA);
	            cv::line(project_img, image_points[f_lb], image_points[b_lb], cube_color, 2, cv::LINE_AA);

            const cv::Point2d center =
                (image_points[f_lt] + image_points[f_rt] + image_points[f_rb] + image_points[f_lb] +
                 image_points[b_lt] + image_points[b_rt] + image_points[b_rb] + image_points[b_lb]) *
                0.125;
            cube_centers.emplace_back(center);
            cv::circle(project_img, center, 3, cube_color, -1);
        }

        // 中心连线：按旋转生成顺序连接成五边形
        constexpr std::array<size_t, 5> center_order = {0, 1, 2, 3, 4};
        for (size_t i = 0; i < center_order.size(); ++i)
        {
            const size_t cur = center_order[i];
            const size_t nxt = center_order[(i + 1) % center_order.size()];
            cv::line(project_img, cube_centers[cur], cube_centers[nxt], center_color, 2, cv::LINE_AA);
        }
    }

    // 可视化法向量
    cv::Point2d start = image_points[image_points.size() - 2];
    cv::Point2d end = image_points[image_points.size() - 1];
    cv::Scalar color = facing_camera ? cv::Scalar(0, 0, 255)  // 红色：朝里
                                     : cv::Scalar(255, 0, 0); // 蓝色：朝外
    cv::arrowedLine(project_img, start, end, color, 2);

    // 上次目标在本帧的可视化
    std::vector<Eigen::Vector3d> rune_targets_now;
    std::vector<RuneTarget> rune_targets;
    rune_targets.reserve(m_debug_rune_targets.size());
    for (const auto &debug_rune_target : m_debug_rune_targets)
    {
        const double age_s = function::timestampMinus(
                                 refined_rune_observation.timestamp,
                                 debug_rune_target.capture_timestamp) *
                             0.001;
        if (age_s >= 0.0 && age_s <= 0.5)
        {
            rune_targets.emplace_back(debug_rune_target);
        }
    }
    if (!rune_targets.empty())
    {
        // 变换到该帧
        rune_targets_now.reserve(rune_targets.size());
        for (const auto &rune_target : rune_targets)
        {

            double phase;
            if (!rune_target.is_big_rune)
            {
                double time_interval = function::timestampMinus(refined_rune_observation.timestamp, rune_target.capture_timestamp) * 0.001;
                phase = rune_target.phase + rune_target.angular_velocity * time_interval;
            }
            else
            {
                const auto &motion = rune_target.big_rune_motion_model;
                //phase(t) = A*cos(ωt) + B*sin(ωt) + b*t + C
                double t = function::timestampMinus(refined_rune_observation.timestamp,rune_target.big_rune_motion_model.reference_timestamp) * 0.001;
                phase = motion.phase_cos_coefficient * std::cos(motion.speed_angular_frequency * t)
                + motion.phase_sin_coefficient * std::sin(motion.speed_angular_frequency * t)
                + motion.phase_linear_velocity * t
                + motion.phase_constant_offset_radians;
                //std::cout<<"phase"<<phase<<std::endl;
            }

            // Rodrigues公式在平面内的简化形式
            Eigen::Vector3d u0 = rune_target.start_vector.normalized();
            Eigen::Vector3d n = rune_target.rune_plane_world_normal.normalized();
            Eigen::Vector3d direction = u0 * std::cos(phase) + n.cross(u0) * std::sin(phase);
            Eigen::Vector3d rune_target_now = rune_target.rune_center + (double)J_POWER_RUNE.config_["rune_ballistic_model"]["radius"] * direction;
            rune_targets_now.push_back(std::move(rune_target_now));
        }
    }
    if (!rune_targets_now.empty())
    {
        const Eigen::Matrix3d R_cam_from_car = R_tf.transpose();
        const Eigen::Vector3d T_cam_from_car = -R_cam_from_car * T_tf;

        std::vector<cv::Point3d> rune_targets_cam;
        rune_targets_cam.reserve(rune_targets_now.size());
        for (const auto &target_car : rune_targets_now)
        {
            Eigen::Vector3d target_cam = R_cam_from_car * target_car + T_cam_from_car;
            if (target_cam.z() <= 1e-6)
            {
                continue;
            }
            rune_targets_cam.emplace_back(target_cam.x(), target_cam.y(), target_cam.z());
        }

        if (!rune_targets_cam.empty())
        {
            std::vector<cv::Point2d> rune_targets_img;
            rune_targets_img.reserve(rune_targets_cam.size());
            cv::projectPoints(rune_targets_cam, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), CAM, DIS, rune_targets_img);
            for (const auto &pt : rune_targets_img)
            {
                cv::line(project_img, pt + cv::Point2d(-6, 0), pt + cv::Point2d(6, 0), cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
                cv::line(project_img, pt + cv::Point2d(0, -6), pt + cv::Point2d(0, 6), cv::Scalar(0, 165, 255), 2, cv::LINE_AA);
            }
        }
    }

    ImgViz::enqueue_image_zero_copy("PowerRune/Projection", project_img);
    }

    if (plane_points_vector.empty())
    {
        LOG(ERROR) << "[PowerRunePlane]plane_points_vector为空";
        return;
    }

    // 更新窗口数据
    update_plane_points_window(PlanePoints(plane_points_vector.front(), refined_rune_observation.timestamp));
    
    //构造未激活目标
    m_inactive_targets.rune_pieces.clear();
    for (const auto &plane_points : plane_points_vector)
    {
        if (plane_points.size() <= 1)
        {
            LOG(ERROR) << "[PowerRunePlane]plane_points点数不足";
            continue;
        }
        m_inactive_targets.rune_pieces.emplace_back(plane_points[0], plane_points[1]); // 符心,靶心
    }
    



    // 运行到此处有两种情况：m_plane_points_window没有插入新的数据，m_plane_points_window插入了新的数据
    // 平面更新的条件：m_plane_points_window的数据量足够大，并且存在未激活目标
    // 未激活目标优化的条件：平面正常更新
    if (!m_inactive_targets.rune_pieces.empty())
    {
        if (!estimate_power_rune_plane())
        {
            // 无法更新平面
            return;
        }

        // 运行到此处说明可以更新m_inactive_targets
        optimize_inactive_targets();
        m_inactive_targets.capture_timestamp = refined_rune_observation.timestamp;
        m_inactive_targets.is_vaild = true; // 此时未激活目标才有效有效


        
    }
    else
    {
        // 无未激活的数据，不进行更新
        return;
    }
}


void PowerRunePlane::update_plane_points_window(PlanePoints &&plane_points)
{

    //实际调试的时候发现如果对多帧进行平面拟合，效果反而更差
    m_plane_points_window.clear();
    //插入新数据
    m_plane_points_window.emplace_back(std::move(plane_points));

    // //删除不在窗口内的数据
    // const double window_settling_time =  J_POWER_RUNE.config_["plane"]["window_settling_time"];
    // while (!m_plane_points_window.empty())
    // {
    //     double max_window_time = function::timestampMinus(m_plane_points_window.back().timestamp_capture,m_plane_points_window.front().timestamp_capture) / 1000.0;
        
    //     if (max_window_time > window_settling_time)
    //     {
    //         m_plane_points_window.pop_front();
    //     }
    //     else
    //     {
    //         break;
    //     }
        
    // }
    
}

bool PowerRunePlane::estimate_power_rune_plane()    
{
    // 判断有没有拟合的价值(例如1s内的窗口中却只有一两个数据,说明可能是刚开始观测或者失去了观测)
    if (m_plane_points_window.size() < (int)J_POWER_RUNE.config_["plane"]["min_plane_points_window_size"])
    {
        //不更新平面
        return false;
    }

    //收集所有点同时计算质心
    std::vector<Eigen::Vector3d> all_points;
    Eigen::Vector3d points_centroid = Eigen::Vector3d::Zero();
    for (const auto& plane_points : m_plane_points_window)
    {
        for (const auto& point : plane_points.plane_points)
        {
            points_centroid += point;
            all_points.emplace_back(point);
        }
    }
    points_centroid /= all_points.size();


    //构造去中心化矩阵
    Eigen::MatrixXd decentralized_matrix(all_points.size(), 3);
    for (size_t i = 0; i < all_points.size(); ++i)
    {
        decentralized_matrix.row(i) = (all_points[i] - points_centroid).transpose();
    }

    //SVD
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(decentralized_matrix, Eigen::ComputeThinV);

    //计算法向量并且约束朝前
    Eigen::Vector3d normal = svd.matrixV().col(2).normalized();
    if (normal.dot(Eigen::Vector3d(0,0,1)) < 0)
    {
        normal = -normal;
    }
    

    //赋值
    m_power_rune_plane = Eigen::Hyperplane<double, 3>(normal, points_centroid);
    return true;

}

void PowerRunePlane::optimize_inactive_targets()
{
    //将符心与靶心进行投影
    for (auto &rune_piece: m_inactive_targets.rune_pieces)
    {
        //符心
        rune_piece.armor_center = m_power_rune_plane.projection(rune_piece.armor_center);

        //靶心
        rune_piece.rune_center= m_power_rune_plane.projection(rune_piece.rune_center);
    }

    //所在的平面
    m_inactive_targets.power_rune_plane = m_power_rune_plane;
    
}

const InactiveTargets PowerRunePlane::get_inactive_targets() const
{
    return m_inactive_targets;
}


Projector::Projector()
{
    cv::Size max_img_size((int)J_POWER_RUNE.config_["project"]["max_img_width"], (int)J_POWER_RUNE.config_["project"]["max_img_hight"]);
    m_binary_buffer = cv::Mat(max_img_size, CV_8UC1); //预分配内存
    m_max_chamfer_residual = (double)J_POWER_RUNE.config_["project"]["max_chamfer_residual"];

    //为指针赋值
    m_anchor_inactive_points_world_ptr = std::make_shared<std::vector<cv::Point3f>>(anchor_inactive_points_world);

    m_inactive_points_model_1_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(inactive_points_model_1);
    m_inactive_points_model_2_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(inactive_points_model_2);
    m_inactive_points_model_3_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(inactive_points_model_3);
    m_inactive_points_model_4_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(inactive_points_model_4);
    m_inactive_points_model_5_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(inactive_points_model_5);

    m_small_power_rune_active_armor_model_1_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_armor_model_1);
    m_small_power_rune_active_armor_model_2_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_armor_model_2);
    m_small_power_rune_active_armor_model_3_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_armor_model_3);
    m_small_power_rune_active_armor_model_4_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_armor_model_4);
    m_small_power_rune_active_armor_model_5_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_armor_model_5);

    m_small_power_rune_active_light_arm_model_1_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_light_arm_model_1);
    m_small_power_rune_active_light_arm_model_2_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_light_arm_model_2);
    m_small_power_rune_active_light_arm_model_3_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_light_arm_model_3);
    m_small_power_rune_active_light_arm_model_4_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_light_arm_model_4);
    m_small_power_rune_active_light_arm_model_5_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(small_power_rune_active_light_arm_model_5);

    m_big_power_rune_active_points_model_1_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(big_power_rune_active_points_model_1);
    m_big_power_rune_active_points_model_2_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(big_power_rune_active_points_model_2);
    m_big_power_rune_active_points_model_3_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(big_power_rune_active_points_model_3);
    m_big_power_rune_active_points_model_4_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(big_power_rune_active_points_model_4);
    m_big_power_rune_active_points_model_5_ptr = std::make_shared<std::vector<Eigen::Vector3d>>(big_power_rune_active_points_model_5);
}

bool Projector::caculate_pose(const std::vector<SingleRuneBlade2D> &rune_blade_2D,
                              const transform_tools::TFTree &tf_tree,
                              const cv::Mat &ori_img)
{
    m_tf_tree = tf_tree;
    (void)ori_img;
    
    //根据语义构建符模型
    if (!construct_rune_correspondence(rune_blade_2D))
    {
        return false;
    }

    //根据m_rune_correspondence构建距离变换函数
    calculate_distance_transfor();

    //进行优化
    if (!minimize_chamfer_distance())
    {
        return false;
    }

    return true;
}


bool Projector::construct_rune_correspondence(const std::vector<SingleRuneBlade2D> &rune_blade_2D)
{
    //判断是否至少有一个未激活目标
    bool has_usable_inactive_blade = false;
    for (const auto &single_rune_blade_2D : rune_blade_2D)
    {
        if (is_valid_for_projection(single_rune_blade_2D))
        {
            if (single_rune_blade_2D.rune_state == RuneState::BigInactive)
            {
                has_usable_inactive_blade = true;
            }
            if (single_rune_blade_2D.rune_state == RuneState::SmallInactive)
            {
                has_usable_inactive_blade = true;
            }
            
        }
    }
    if(!has_usable_inactive_blade)
    {
        //std::cout<<"[Projector]无可用的未激活目标"<<std::endl;
        return false;
    }

    //清空旧数据
    m_rune_correspondence.clear();
    m_sampled_model_points.clear();
    m_another_inactivate_target_location_num = -1;


    //1号位需要保证是未激活目标
    int inactiave_index = -1;
    for (int i = 0; i < rune_blade_2D.size(); i++)
    {
        if (!is_valid_for_projection(rune_blade_2D[i]))
        {
            continue;
        }
        
        if (rune_blade_2D[i].rune_state == RuneState::BigInactive || rune_blade_2D[i].rune_state == RuneState::SmallInactive)
        {
            //设置符心
            m_rune_center = cv::fitEllipse(rune_blade_2D[i].constrained_contours.center_R_opt.value()).center;//外部保证有值

            //设置1号位(未激活目标)
            set_location_x(1,rune_blade_2D[i]);
            inactiave_index = i;
            break;
        }
    }

    //为其他位置设置轮廓-模型对
    for (int i = 0; i < rune_blade_2D.size(); i++)
    {
        if(i == inactiave_index)
        {
            //跳过已经设置好的1号位
            continue;
        }
        set_location(rune_blade_2D[i]);
    }

    return true;
}

void Projector::set_location(const SingleRuneBlade2D &single_rune_blade_2D)
{
    //不满足条件直接跳过
    if (!is_valid_for_projection(single_rune_blade_2D))
    {
        return;
    }

    //获取1号位的方向向量:
    const Projector::RuneBladeCorrespondence* location_1_correspondence = get_location_x_correspondence(1);
    if (!location_1_correspondence)
    {
        LOG(ERROR) << "[Projector]没有设置1号位却对其进行了读取";
        throw std::runtime_error("[Projector]没有设置1号位却对其进行了读取");
    }
    Eigen::Vector3d direction_vector_1 =  (*location_1_correspondence).direction_vector;

    //计算方向向量
    Eigen::Vector3d direction_vector = calculate_direction_vector(single_rune_blade_2D);

    //计算角度(由direction_vector_1转动到direction_vector)
    Eigen::Vector3d direction = direction_vector_1.cross(direction_vector);
    double sin_theta = direction.z();//z坐标即为向量的长度，也即sin_theta;
    double cos_theta = direction_vector_1.dot(direction_vector);
    double theta = std::atan2(sin_theta, cos_theta);
    theta = theta < 0 ? theta + CV_2PI : theta;//转换到0-2pi

    //判断位置来进行构造
    //1号位在0/360位置，无需判断
    //2号位在72位置
    //3号位在144位置
    //4号位在216位置
    //5号位在288位置
    constexpr double deg_72_in_rad = 0.2 * CV_2PI;
    constexpr double deg_144_in_rad = 0.4 * CV_2PI;
    constexpr double deg_216_in_rad = 0.6 * CV_2PI;
    constexpr double deg_288_in_rad = 0.8 * CV_2PI;
    double half_interval = (double)J_POWER_RUNE.config_["project"]["half_interval"];//误差允许区间(由于观察角度和计算误差的原因，向量夹角是在区间里)

    //TODO：加入去重的处理

    if (deg_72_in_rad - half_interval <= theta && theta <= deg_72_in_rad + half_interval)
    {
        // 2号位
        set_location_x(2, single_rune_blade_2D);
    }
    else if (deg_144_in_rad - half_interval <= theta && theta <= deg_144_in_rad + half_interval)
    {
        // 3号位
        set_location_x(3, single_rune_blade_2D);
    }
    else if (deg_216_in_rad - half_interval <= theta && theta <= deg_216_in_rad + half_interval)
    {
        // 4号位
        set_location_x(4, single_rune_blade_2D);
    }
    else if (deg_288_in_rad - half_interval <= theta && theta <= deg_288_in_rad + half_interval)
    {
        // 5号位
        set_location_x(5, single_rune_blade_2D);
    }
    else
    {
        LOG(ERROR) << "[Projector]方向向量夹角极端";
    }
}

void Projector::set_location_x(int location_x, const SingleRuneBlade2D &single_rune_blade_2D)
{
    if (location_x<1 || location_x>5)
    {
       LOG(ERROR) << "[Projector]set_location传入了异常位置";
       throw std::runtime_error("[Projector]set_location传入了异常位置");
    }

    switch (single_rune_blade_2D.rune_state)
    {
    case RuneState::SmallInactive:
    {
        switch (location_x)
        {
        case 1:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_inactive_points_model_1_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            rune_blade_correspondence.anchor_points = calculate_anchor_points(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        case 2:
        case 3:
        case 4:
        case 5:
            //小符不接收第二片未激活叶片
            LOG(ERROR) << "[Projector]小符出现两片未激活符叶";
            break;
        }
        break;
    }
    case RuneState::BigInactive:
    {
        switch (location_x)
        {
        case 1:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_inactive_points_model_1_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            rune_blade_correspondence.anchor_points = calculate_anchor_points(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        case 2:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_inactive_points_model_2_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            rune_blade_correspondence.anchor_points = calculate_anchor_points(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            m_another_inactivate_target_location_num = location_x;
            break;
        }
        case 3:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_inactive_points_model_3_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            rune_blade_correspondence.anchor_points = calculate_anchor_points(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            m_another_inactivate_target_location_num = location_x;
            break;
        }
        case 4:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_inactive_points_model_4_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            rune_blade_correspondence.anchor_points = calculate_anchor_points(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            m_another_inactivate_target_location_num = location_x;
            break;
        }
        case 5:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_inactive_points_model_5_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            rune_blade_correspondence.anchor_points = calculate_anchor_points(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            m_another_inactivate_target_location_num = location_x;
            break;
        }
        }
        break;
    }
    case RuneState::SmallActivated:
    {
        // 为了减少计算量只放回装甲板模块
        switch (location_x)
        {
        case 1:
        {
            LOG(ERROR) << "[Projector]尝试在1号位设置已经激活的小符目标";
            throw std::runtime_error("[Projector]尝试在1号位设置已经激活的小符目标");
            break;
        }
        case 2:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_small_power_rune_active_armor_model_2_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }

        case 3:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_small_power_rune_active_armor_model_3_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        case 4:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_small_power_rune_active_armor_model_4_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        case 5:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_small_power_rune_active_armor_model_5_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.armor_module_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        }
        break;
    }
    case RuneState::BigActivated:
    {
        
        // 只有灯臂可放回
        switch (location_x)
        {
        case 1:
        {
            LOG(ERROR) << "[Projector]尝试在1号位设置已经激活的大符目标";
            throw std::runtime_error("[Projector]尝试在1号位设置已经激活的大符目标");
            break;
        }
        case 2:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_big_power_rune_active_points_model_2_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }

        case 3:
        {
            RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_big_power_rune_active_points_model_3_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        case 4:
        {
             RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_big_power_rune_active_points_model_4_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        case 5:
        {
             RuneBladeCorrespondence rune_blade_correspondence;
            rune_blade_correspondence.location_num = location_x;
            rune_blade_correspondence.rune_state = single_rune_blade_2D.rune_state;
            rune_blade_correspondence.model_rune_points.push_back(m_big_power_rune_active_points_model_5_ptr);
            rune_blade_correspondence.matched_contours.push_back(*(single_rune_blade_2D.constrained_contours.light_arm_opt));
            rune_blade_correspondence.direction_vector = calculate_direction_vector(single_rune_blade_2D);
            m_rune_correspondence.emplace_back(std::move(rune_blade_correspondence));
            break;
        }
        }
        break;
    }
    }

    return;
}

const Projector::RuneBladeCorrespondence *Projector::get_location_x_correspondence(int location_x) const
{
    for (const auto &rune_blade_correspondence : m_rune_correspondence)
    {
        if (rune_blade_correspondence.location_num == location_x)
        {
            return &rune_blade_correspondence;
        }
    }

    return nullptr;
}

std::vector<cv::Point3f> Projector::calculate_inactive_anchor_points_world(int location_x) const
{
    if (location_x < 1 || location_x > 5)
    {
        LOG(ERROR) << "[Projector]calculate_inactive_anchor_points_world传入了异常位置";
        throw std::runtime_error("[Projector]calculate_inactive_anchor_points_world传入了异常位置");
    }

    if (location_x == 1)
    {
        return *m_anchor_inactive_points_world_ptr;
    }

    const double theta_rotate = (location_x - 1) * (CV_2PI / 5.0);
    const double cos_theta = std::cos(theta_rotate);
    const double sin_theta = std::sin(theta_rotate);

    std::vector<cv::Point3f> rotated_anchor_points_world;
    rotated_anchor_points_world.reserve(m_anchor_inactive_points_world_ptr->size());
    for (const auto &point : *m_anchor_inactive_points_world_ptr)
    {
        const float x = static_cast<float>(cos_theta * point.x - sin_theta * point.y);
        const float y = static_cast<float>(sin_theta * point.x + cos_theta * point.y);
        rotated_anchor_points_world.emplace_back(x, y, point.z);
    }

    return rotated_anchor_points_world;
}

void Projector::calculate_distance_transfor()
{
    // 必须截取roi来计算距离变换函数，否则计算量会非常大
    auto for_each_contour = [this](auto &&func)
    {
        for (const auto &rune_blade_correspondence : m_rune_correspondence)
        {
            for (const auto &contour : rune_blade_correspondence.matched_contours)
            {
                func(contour);
            }
        }
    };

    bool first = true;
    double contour_short_side_sum = 0.0;
    int valid_contour_count = 0;
    for_each_contour([&](const std::vector<cv::Point> &contour)
    {
        if (contour.empty())
        {
            return;
        }

        cv::Rect rect = cv::boundingRect(contour);
        contour_short_side_sum += static_cast<double>(std::min(rect.width, rect.height));
        ++valid_contour_count;

        if (first)
        {
            m_distance_transform_rect = rect;
            first = false;
        }
        else
        {
            m_distance_transform_rect |= rect;
        }
    });

    const cv::Rect contour_union_rect = m_distance_transform_rect;

    // 缩放并与最大原图尺寸求交
    float scale = J_POWER_RUNE.config_["project"]["distance_transform_roi_scale"]; // roi缩放系数
    cv::Point2f center = (m_distance_transform_rect.tl() + m_distance_transform_rect.br()) * 0.5f;
    m_distance_transform_rect.width = static_cast<int>(m_distance_transform_rect.width * scale);
    m_distance_transform_rect.height = static_cast<int>(m_distance_transform_rect.height * scale);
    m_distance_transform_rect.x = static_cast<int>(center.x - m_distance_transform_rect.width / 2);
    m_distance_transform_rect.y = static_cast<int>(center.y - m_distance_transform_rect.height / 2);
    m_distance_transform_rect &= cv::Rect(0, 0, (int)J_POWER_RUNE.config_["project"]["max_img_width"], (int)J_POWER_RUNE.config_["project"]["max_img_hight"]);

    // 生成二值图(在roi上)
    cv::Mat binary_roi = m_binary_buffer(m_distance_transform_rect); // 此处破坏内存连续性
    binary_roi.setTo(255);
    for_each_contour([&](const std::vector<cv::Point> &contour)
    {
        for (const auto &ponit : contour)
        {
            cv::Point roi_ponit = ponit - m_distance_transform_rect.tl();
            if ((unsigned)roi_ponit.x < (unsigned)binary_roi.cols && (unsigned)roi_ponit.y < (unsigned)binary_roi.rows)
            {
                binary_roi.at<uchar>(roi_ponit) = 0;
            }
        }
    });

    // 构建“轮廓内部”掩码：用于把内部代价重映射为非线性增长
    cv::Mat inside_mask = cv::Mat::zeros(binary_roi.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> roi_contours;
    size_t contour_count = 0;
    for (const auto &rune_blade_correspondence : m_rune_correspondence)
    {
        contour_count += rune_blade_correspondence.matched_contours.size();
    }
    roi_contours.reserve(contour_count);
    for_each_contour([&](const std::vector<cv::Point> &contour)
    {
        if (contour.empty())
        {
            return;
        }
        std::vector<cv::Point> roi_contour;
        roi_contour.reserve(contour.size());
        for (const auto &point : contour)
        {
            cv::Point roi_point = point - m_distance_transform_rect.tl();
            if ((unsigned)roi_point.x < (unsigned)inside_mask.cols && (unsigned)roi_point.y < (unsigned)inside_mask.rows)
            {
                roi_contour.emplace_back(roi_point);
            }
        }
        if (roi_contour.size() >= 3)
        {
            roi_contours.emplace_back(std::move(roi_contour));
        }
    });
    if (!roi_contours.empty())
    {
        cv::drawContours(inside_mask, roi_contours, -1, cv::Scalar(255), cv::FILLED);
    }

    // 生成距离函数(在roi上)注意binary_roi内存是非连续的，但是distanceTransform重新分配了内存，所以m_distance_transform内存是连续的
    cv::distanceTransform(binary_roi, m_distance_transform, cv::DIST_L2, cv::DIST_MASK_PRECISE); // 用欧氏距离

    // 对轮廓内外都使用同一套非线性代价：0,1,2,3,5,8...
    auto nonlinear_penalty = [](float distance_value) -> float
    {
        const int layer = std::max(0, static_cast<int>(std::lround(distance_value)));
        if (layer <= 0)
        {
            return 0.0f;
        }
        if (layer == 1)
        {
            return 1.0f;
        }
        if (layer == 2)
        {
            return 2.0f;
        }
        if (layer == 3)
        {
            return 3.0f;
        }
        if (layer == 4)
        {
            return 5.0f;
        }
        if (layer == 5)
        {
            return 8.0f;
        }

        int prev = 5;
        int curr = 8;
        for (int i = 6; i <= layer; ++i)
        {
            const int next = prev + curr;
            prev = curr;
            curr = next;
        }
        return static_cast<float>(curr);
    };

    int max_nonlinear_outer_band = 0;
    while (nonlinear_penalty(static_cast<float>(max_nonlinear_outer_band + 1)) <= static_cast<float>(m_max_chamfer_residual))
    {
        ++max_nonlinear_outer_band;
    }

    const int roi_outer_padding = std::max(0, std::min({
        contour_union_rect.x - m_distance_transform_rect.x,
        contour_union_rect.y - m_distance_transform_rect.y,
        m_distance_transform_rect.br().x - contour_union_rect.br().x,
        m_distance_transform_rect.br().y - contour_union_rect.br().y}));

    const double mean_contour_short_side = valid_contour_count > 0
        ? contour_short_side_sum / static_cast<double>(valid_contour_count)
        : 0.0;
    const int scale_adaptive_outer_band = static_cast<int>(
        std::ceil(mean_contour_short_side * std::max(0.0f, scale - 1.0f) * 0.5f));
    const int adaptive_outer_band = std::max(
        0,
        std::min({max_nonlinear_outer_band, roi_outer_padding, scale_adaptive_outer_band}));

    for (int y = 0; y < m_distance_transform.rows; ++y)
    {
        float *dist_ptr = m_distance_transform.ptr<float>(y);
        const uchar *inside_ptr = inside_mask.ptr<uchar>(y);
        const uchar *edge_ptr = binary_roi.ptr<uchar>(y);
        for (int x = 0; x < m_distance_transform.cols; ++x)
        {
            // 边缘点保持0
            if (edge_ptr[x] == 0)
            {
                dist_ptr[x] = 0.0f;
                continue;
            }

            // 内部始终做非线性增大。
            if (inside_ptr[x] != 0)
            {
                dist_ptr[x] = nonlinear_penalty(dist_ptr[x]);
                continue;
            }

            // 外部只在轮廓附近的自适应区域做非线性增大，远处直接饱和。
            if (adaptive_outer_band > 0 && dist_ptr[x] <= static_cast<float>(adaptive_outer_band))
            {
                dist_ptr[x] = nonlinear_penalty(dist_ptr[x]);
            }
            else
            {
                dist_ptr[x] = static_cast<float>(m_max_chamfer_residual);
            }
        }
    }

    CV_Assert(m_distance_transform.isContinuous());

    // 可视化(需要1ms的耗时，除非需要可视化不然不要开)
    // cv::normalize(m_distance_transform,distance_transform_visualize,0, 255,cv::NORM_MINMAX);
    // distance_transform_visualize.convertTo(distance_transform_visualize, CV_8UC1);
}

bool Projector::minimize_chamfer_distance()
{
    // 动态聚合所有未激活目标的锚点，除了1号位，也允许其他位置的未激活目标提供锚点。
    std::vector<cv::Point2f> aggregated_anchor_points;
    std::vector<cv::Point3f> aggregated_anchor_points_world;
    aggregated_anchor_points.reserve(m_rune_correspondence.size() * 4);
    aggregated_anchor_points_world.reserve(m_rune_correspondence.size() * 4);

    for (const auto &correspondence : m_rune_correspondence)
    {
        const bool is_inactive_target =
            correspondence.rune_state == RuneState::SmallInactive ||
            correspondence.rune_state == RuneState::BigInactive;
        if (!is_inactive_target || correspondence.anchor_points.size() < 4)
        {
            continue;
        }

        const std::vector<cv::Point3f> anchor_points_world =
            calculate_inactive_anchor_points_world(correspondence.location_num);
        if (anchor_points_world.size() < 4)
        {
            continue;
        }

        aggregated_anchor_points.insert(
            aggregated_anchor_points.end(),
            correspondence.anchor_points.begin(),
            correspondence.anchor_points.begin() + 4);
        aggregated_anchor_points_world.insert(
            aggregated_anchor_points_world.end(),
            anchor_points_world.begin(),
            anchor_points_world.begin() + 4);
    }

    if (aggregated_anchor_points.size() < 4 || aggregated_anchor_points_world.size() < 4)
    {
        LOG(ERROR) << "[minimize_chamfer_distance_new]未激活目标锚点数量不足";
        return false;
    }

    // pnp得到先验pose
    cv::Vec3d rvec, tvec;
    const int solvepnp_flag = aggregated_anchor_points_world.size() == 4 ? cv::SOLVEPNP_IPPE : cv::SOLVEPNP_IPPE;
    const bool success = cv::solvePnP(
        aggregated_anchor_points_world,
        aggregated_anchor_points,
        CAM,
        DIS,
        rvec, tvec,
        false,
        solvepnp_flag);
    if (!success)
    {
        LOG(ERROR) << "[minimize_chamfer_distance_new]solvePnP失败";
        return false;
    }

    //赋值(先验)
    m_pose[0] = rvec[0];
    m_pose[1] = rvec[1];
    m_pose[2] = rvec[2];
    m_pose[3] = tvec[0];
    m_pose[4] = tvec[1];
    m_pose[5] = tvec[2];

    //先验待优化参数(rvec,tvec)
    Eigen::Vector<double, 6> pose;
    pose[0] = rvec[0];
    pose[1] = rvec[1];
    pose[2] = rvec[2];
    pose[3] = tvec[0];
    pose[4] = tvec[1];
    pose[5] = tvec[2];

    //断言m_distance_transform的数据是float类型，否则后续的计算会出错
    CV_Assert(m_distance_transform.type() == CV_32FC1);

    //对模型点进行下采样
    sample_model_points();

    // 调用ceres进行优化
    ceres::Problem problem;
    double fx = CAM(0, 0);
    double fy = CAM(1, 1);
    double cx = CAM(0, 2);
    double cy = CAM(1, 2);
    const float *distance_transform_ptr = m_distance_transform.ptr<float>();
    transform_tools::TF tf_car_from_camera = m_tf_tree.calculate_relative_TF(car_frame, camera_frame);
    Eigen::Matrix3d r_car_from_camera = tf_car_from_camera.get_rotation();

    double k_axis_y_constraint_weight_total = (double)J_POWER_RUNE.config_["project"]["k_axis_y_constraint_weight_total"];
    size_t total_points = m_sampled_model_points.size();
    double axis_weight_per_residual = k_axis_y_constraint_weight_total / std::sqrt(std::max<size_t>(1, total_points));

    //添加残差块
    for (const auto *model_point_ptr : m_sampled_model_points)
    {
        const Eigen::Vector3d &model_point = *model_point_ptr;
        ceres::CostFunction *cost_function =
            new ceres::AutoDiffCostFunction<ChamferResidual, 2, 6>(
                new ChamferResidual(
                    distance_transform_ptr,
                    m_distance_transform_rect,
                    model_point,
                    m_max_chamfer_residual,
                    fx, fy, cx, cy,
                    r_car_from_camera,
                    axis_weight_per_residual));
        problem.AddResidualBlock(cost_function, nullptr, pose.data());
    }

    double anchor_reproj_weight = 1.0;
    {
        const cv::FileNode weight_node = J_POWER_RUNE.config_["project"]["anchor_reproj_weight"];
        if (!weight_node.empty())
        {
            anchor_reproj_weight = std::max(0.0, static_cast<double>(weight_node));
        }
    }
    if (anchor_reproj_weight > 0.0)
    {
        const size_t anchor_count = std::min(aggregated_anchor_points.size(), aggregated_anchor_points_world.size());
        for (size_t i = 0; i < anchor_count; ++i)
        {
            const cv::Point3f &anchor_world = aggregated_anchor_points_world[i];
            const Eigen::Vector3d anchor_world_e(anchor_world.x, anchor_world.y, anchor_world.z);
            ceres::CostFunction *anchor_cost_function =
                new ceres::AutoDiffCostFunction<AnchorReprojResidual, 2, 6>(
                    new AnchorReprojResidual(
                        anchor_world_e,
                        aggregated_anchor_points[i],
                        fx, fy, cx, cy,
                        anchor_reproj_weight));
            problem.AddResidualBlock(anchor_cost_function, nullptr, pose.data());
        }
    }

    //求解 
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 100;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    //结果检查
    if (summary.IsSolutionUsable())
    {
        // 赋值
        m_pose = pose;
        return true;
    }

    LOG(ERROR) << "[minimize_chamfer_distance_new]失败";
    return false;
}

void Projector::sample_model_points()
{
    m_sampled_model_points.clear();
    if (m_rune_correspondence.empty())
    {
        return;
    }

    //采样步长
    int stride = J_POWER_RUNE.config_["project"]["sample_stride"];

    //采样
    m_sampled_model_points.reserve(256);
    for (const auto &rune_blade_correspondence : m_rune_correspondence)
    {
        for (const auto &model_points_ptr : rune_blade_correspondence.model_rune_points)
        {
            const auto &model_points = *model_points_ptr;
            if (model_points.empty())
            {
                continue;
            }

            for (size_t i = 0; i < model_points.size(); i += static_cast<size_t>(stride))
            {
                m_sampled_model_points.emplace_back(&model_points[i]);
            }

            // 保证尾点被采到，避免末段完全丢失
            const size_t last_idx = model_points.size() - 1;
            if (last_idx % static_cast<size_t>(stride) != 0)
            {
                m_sampled_model_points.emplace_back(&model_points[last_idx]);
            }
        }
    }
}

std::vector<cv::Point2f> Projector::calculate_anchor_points(const SingleRuneBlade2D &single_rune_blade_2D)
{
    //如果是未激活目标
    if (single_rune_blade_2D.rune_state == RuneState::BigInactive || single_rune_blade_2D.rune_state == RuneState::SmallInactive)
    {
        cv::Point2f pixel_points[4];
        find_anchor_points(pixel_points,single_rune_blade_2D);
        sort_anchor_points(pixel_points);

        return std::vector<cv::Point2f>(pixel_points, pixel_points + 4);
    }
    

    //TODO:如果是已激活目标
    return std::vector<cv::Point2f>();
}

void Projector::find_anchor_points(cv::Point2f pixel_points[4], const SingleRuneBlade2D &single_rune_blade_2D)
{
    // contour -> PCA 数据
    const auto &armor_contour = single_rune_blade_2D.constrained_contours.armor_module_opt.value();
    const auto &light_arm_contour = single_rune_blade_2D.constrained_contours.light_arm_opt.value();

    const int total_pts = armor_contour.size() + light_arm_contour.size();
    cv::Mat data_pts(total_pts, 2, CV_64F);

    int idx = 0;
    // 装甲板轮廓
    for (const auto &pt : armor_contour)
    {
        data_pts.at<double>(idx, 0) = static_cast<double>(pt.x);
        data_pts.at<double>(idx, 1) = static_cast<double>(pt.y);
        ++idx;
    }

    // 灯臂轮廓
    for (const auto &pt : light_arm_contour)
    {
        data_pts.at<double>(idx, 0) = static_cast<double>(pt.x);
        data_pts.at<double>(idx, 1) = static_cast<double>(pt.y);
        ++idx;
    }

    // PCA
    cv::PCA pca(data_pts, cv::Mat(), cv::PCA::DATA_AS_ROW);

    // 中心
    cv::Point2f center(
        static_cast<float>(pca.mean.at<double>(0, 0)),
        static_cast<float>(pca.mean.at<double>(0, 1)));

    // 主方向, 垂直方向
    cv::Point2f axis_x(
        static_cast<float>(pca.eigenvectors.at<double>(0, 0)),
        static_cast<float>(pca.eigenvectors.at<double>(0, 1)));
    cv::Point2f axis_y(-axis_x.y, axis_x.x);

    // 投影到PCA坐标系，找min/max
    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;

    // 装甲板轮廓
    for (const auto &p : armor_contour)
    {
        cv::Point2f pf(static_cast<float>(p.x), static_cast<float>(p.y));
        cv::Point2f v = pf - center;

        float x = v.dot(axis_x);
        float y = v.dot(axis_y);

        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    // 灯臂轮廓
    for (const auto &p : light_arm_contour)
    {
        cv::Point2f pf(static_cast<float>(p.x), static_cast<float>(p.y));
        cv::Point2f v = pf - center;

        float x = v.dot(axis_x);
        float y = v.dot(axis_y);

        min_x = std::min(min_x, x);
        max_x = std::max(max_x, x);
        min_y = std::min(min_y, y);
        max_y = std::max(max_y, y);
    }

    // 反投影得到4个角点
    pixel_points[0] = center + min_x * axis_x + min_y * axis_y;
    pixel_points[1] = center + min_x * axis_x + max_y * axis_y;
    pixel_points[2] = center + max_x * axis_x + max_y * axis_y;
    pixel_points[3] = center + max_x * axis_x + min_y * axis_y;
}

bool Projector::is_valid_for_projection(const SingleRuneBlade2D &single_rune_blade_2D)
{
    if (single_rune_blade_2D.rune_state == RuneState::BigInactive 
        || single_rune_blade_2D.rune_state == RuneState::SmallInactive
        || single_rune_blade_2D.rune_state == RuneState::SmallActivated)
    {
        //对于未激活和已激活的小符必须三者全部允许投影
        return single_rune_blade_2D.is_armor_module_usable 
        && single_rune_blade_2D.is_light_arm_usable
        && single_rune_blade_2D.is_center_R_usable;
    }

    //对于已经激活的大符不要求装甲板模块
    return single_rune_blade_2D.is_center_R_usable && single_rune_blade_2D.is_light_arm_usable;
    
}

Eigen::Vector3d Projector::calculate_direction_vector(const SingleRuneBlade2D &single_rune_blade_2D)
{
    //计算符心
    Eigen::Vector3d rune_center(0, 0, 0);
    const auto &rune_center_contour =  single_rune_blade_2D.constrained_contours.center_R_opt.value();
    cv::Moments rune_center_moments = cv::moments(rune_center_contour);
    if (std::abs(rune_center_moments.m00) > 1e-6)
    {
        rune_center.x() =  rune_center_moments.m10 / rune_center_moments.m00;
        rune_center.y() = rune_center_moments.m01 / rune_center_moments.m00;
    }
    else
    {
        //用几何中心代替质心
        for (const auto &point : rune_center_contour)
        {
            rune_center.x() += point.x;
            rune_center.y() += point.y;
        }
        rune_center /= rune_center_contour.size();
    }
    
    //计算方向向量
    Eigen::Vector3d direction_vector(0, 0, 0);
    switch (single_rune_blade_2D.rune_state)
    {
    //有装甲板模块的轮廓直接拟合矩形
    case RuneState::BigInactive:
    case RuneState::SmallInactive:
    case RuneState::SmallActivated:
    {
        const auto &armor_contour =  single_rune_blade_2D.constrained_contours.armor_module_opt.value();
        cv::RotatedRect ellipse = cv::fitEllipse(armor_contour);//在轮廓提取的时候已经保证过了至少有五个点
        direction_vector = Eigen::Vector3d(ellipse.center.x,ellipse.center.y,0) - rune_center;
        direction_vector.normalize();
        break;
    }
    // 对于只有灯臂的情况计算灯臂轮廓的质心
    case RuneState::BigActivated:
    {
        Eigen::Vector3d light_arm_center(0, 0, 0);
        const auto &light_arm_contour = single_rune_blade_2D.constrained_contours.light_arm_opt.value();
        cv::Moments light_arm_center_moments = cv::moments(light_arm_contour);
        if (std::abs(light_arm_center_moments.m00) > 1e-6)
        {
            light_arm_center.x() = light_arm_center_moments.m10 / light_arm_center_moments.m00;
            light_arm_center.y() = light_arm_center_moments.m01 / light_arm_center_moments.m00;
        }
        else
        {
            // 用几何中心代替质心
            for (const auto &point : light_arm_contour)
            {
                light_arm_center.x() += point.x;
                light_arm_center.y() += point.y;
            }
            light_arm_center /= light_arm_contour.size();
        }
        direction_vector = light_arm_center - rune_center;
        direction_vector.normalize();
        break;
    }
    }
    return direction_vector;
}

void Projector::sort_anchor_points(cv::Point2f pixel_points[4])
{
    // 原始点
    std::vector<cv::Point2f> pts(pixel_points, pixel_points + 4);

    // 从符心出发的向量。严格按这4个向量的长度判断Top/Bottom。
    std::vector<cv::Point2f> vecs(4);
    std::array<float, 4> squared_norms{};
    for (int i = 0; i < 4; ++i)
    {
        vecs[i] = pts[i] - m_rune_center;
        squared_norms[i] = vecs[i].dot(vecs[i]);
    }

    // 严格约定：长度绝对值最大的两个向量是Top，最小的两个向量是Bottom。
    std::vector<int> idx = {0, 1, 2, 3};
    std::stable_sort(idx.begin(), idx.end(),
                     [&](int a, int b)
                     {
                         return squared_norms[a] > squared_norms[b];
                     });

    // 两个长向量 上边
    cv::Point2f up0 = pts[idx[0]];
    cv::Point2f up1 = pts[idx[1]];

    //两个短向量：下边
    cv::Point2f down0 = pts[idx[2]];
    cv::Point2f down1 = pts[idx[3]];

    
    //取中点
    cv::Point2f up_center = (up0 + up1) * 0.5f;
    cv::Point2f down_center = (down0 + down1) * 0.5f;
    cv::Point2f center = (up_center + down_center) * 0.5f;
    
    //构造朝向符心的向量
    Eigen::Vector3f center2rune_center((m_rune_center - center).x, (m_rune_center - center).y, 0);
    
    //构造假设朝向右的向量
    Eigen::Vector3f down02down1((down1 - down0).x, (down1 - down0).y, 0);
    Eigen::Vector3f up02up1((up1-up0).x, (up1-up0).y, 0);
    


    //叉乘判断是否朝向右
    if ((up02up1.cross(center2rune_center)).z() > 0.0)
    {
        //说明朝向右
        pixel_points[0] =up0; //左上
        pixel_points[3] =up1; //右上
        
    }
    else
    {
        //说明朝左
        pixel_points[0] =up1; //左上
        pixel_points[3] =up0; //右上

    }
    //叉乘判断是否朝向右
    if ((down02down1.cross(center2rune_center)).z() > 0.0)
    {
        //说明朝向右
        pixel_points[1] = down0;//左下
        pixel_points[2] = down1;//右下
    }
    else
    {
        //说明朝向左
        pixel_points[1] = down1;//左下
        pixel_points[2] = down0;//右下
    }

    //0:左上
    //1:左下
    //2:右下
    //3:右上   
}

void Projector::find_armor_center_anchor_points(const std::vector<SingleRuneBlade2D> &rune_blade_2D)
{
}

const Eigen::Vector<double, 6> Projector::get_pose() const
{
    return m_pose;
}

const std::vector<const Eigen::Vector3d *> &Projector::get_sampled_model_points() const
{
    return m_sampled_model_points;
}

std::vector<int> Projector::get_inactive_target_location_nums() const
{
    std::vector<int> inactive_target_location_nums;
    inactive_target_location_nums.reserve(m_rune_correspondence.size());

    for (const auto &correspondence : m_rune_correspondence)
    {
        const bool is_inactive_target =
            correspondence.rune_state == RuneState::SmallInactive ||
            correspondence.rune_state == RuneState::BigInactive;
        if (!is_inactive_target)
        {
            continue;
        }

        if (std::find(inactive_target_location_nums.begin(), inactive_target_location_nums.end(), correspondence.location_num) ==
            inactive_target_location_nums.end())
        {
            inactive_target_location_nums.emplace_back(correspondence.location_num);
        }
    }

    return inactive_target_location_nums;
}

const int Projector::get_another_inactivate_target_location_num() const
{
    return m_another_inactivate_target_location_num;
}
