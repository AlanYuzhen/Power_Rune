#include "PowerRuneProcessor.hpp"
#include <chrono>
#include "json.hpp"
#include "common/PowerRuneDiagnostics.hpp"
#include "img_viz.hpp"
#include "common/power_rune_function.hpp"
#include <glog/logging.h>

PowerRuneProcessor::PowerRuneProcessor(RuneDecisionModule &rune_decision_module)
    : m_rune_decision_module(rune_decision_module)
{
}

void PowerRuneProcessor::process_power_rune(const power_rune::RuneInput &detect_input)
{
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"])
    {
        J_POWER_RUNE.updateJson();
    }

    //流程：
    //1. 对网络结果进行处理
    //2. 获取外轮廓
    //3. 更新符平面
    //4. 估计相位和运动方程
    //5. 更新火控模块
    
    //检查网络结果是否为空
    if (detect_input.nn_rune_infos.empty())
    {
        //没有网络结果，直接返回
        return;
    }

    //构造符的观测数据
    RuneObservation rune_observation = convert2rune_observation(detect_input);

    if (rune_observation.rune_infos.empty())
    {
        //没有观测结果，直接返回
        return;
    }
    
#pragma region 
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_DETECT"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_DETECT_SHOW_NN_IMG"])
    {
        // 网络结果可视化
        cv::Mat ori_img0 = detect_input.ori_mat.clone();
        for (const auto &rune_info : rune_observation.rune_infos)
        {
            cv::rectangle(ori_img0, rune_info.view_rect, cv::Scalar(0, 255, 0), 1);
            cv::circle(ori_img0, rune_info.top, 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(ori_img0, rune_info.left, 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(ori_img0, rune_info.bottom, 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(ori_img0, rune_info.right, 3, cv::Scalar(0, 255, 0), 2);
            cv::circle(ori_img0, rune_info.point_R, 3, cv::Scalar(0, 255, 0), 2);

            std::string cls_text;
            cv::Scalar cls_color;

            if (rune_info.class_id == 0)
            {
                cls_text = "UNHIT";
                cls_color = cv::Scalar(0, 255, 255); // 黄
            }
            else
            {
                cls_text = "HIT";
                cls_color = cv::Scalar(0, 0, 255); // 红
            }

            cv::Point text_pos(
                rune_info.view_rect.x,
                rune_info.view_rect.y - 5 // 框上方
            );

            cv::putText(
                ori_img0,
                cls_text,
                text_pos,
                cv::FONT_HERSHEY_SIMPLEX,
                0.5,
                cls_color,
                1);
        }
        ImgViz::enqueue_image_zero_copy("PowerRune/NNResult", ori_img0);
    }
#pragma endregion

    //获取符号轮廓和语义信息
    RefinedRuneObservation refined_rune_observation = m_rune_observation_refiner.refine(rune_observation);//开销在release下大该0.3ms

#pragma region 
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_DETECT"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_DETECT_SHOW_SINGLE_BLADE_IMG"])
    {
        //轮廓可视化
        cv::Mat ori_img = rune_observation.ori_img.clone();
        for (const auto &single_rune_blade_2D : refined_rune_observation.rune_blade_2D)
        {
            // ======== 装甲板模块 ========
            if (single_rune_blade_2D.constrained_contours.armor_module_opt.has_value())
            {
                cv::Scalar color = single_rune_blade_2D.is_armor_module_usable
                                       ? cv::Scalar(0, 255, 0)   // 亮绿
                                       : cv::Scalar(80, 80, 80); // 灰

                std::vector<std::vector<cv::Point>> contours;
                contours.push_back(single_rune_blade_2D.constrained_contours.armor_module_opt.value());

                cv::drawContours(ori_img, contours, -1, color, 2);

                cv::putText(
                    ori_img,
                    single_rune_blade_2D.is_armor_module_usable ? "Armor" : "Armor (X)",
                    single_rune_blade_2D.constrained_contours.armor_module_opt.value()[0],
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1);
            }

            // ======== 灯臂 ========
            if (single_rune_blade_2D.constrained_contours.light_arm_opt.has_value())
            {
                cv::Scalar color = single_rune_blade_2D.is_light_arm_usable
                                       ? cv::Scalar(0, 255, 0) // 黄
                                       : cv::Scalar(80, 80, 80); // 灰

                std::vector<std::vector<cv::Point>> contours;
                contours.push_back(single_rune_blade_2D.constrained_contours.light_arm_opt.value());

                cv::drawContours(ori_img, contours, -1, color, 2);

                cv::putText(
                    ori_img,
                    single_rune_blade_2D.is_light_arm_usable ? "LightArm" : "LightArm (X)",
                    single_rune_blade_2D.constrained_contours.light_arm_opt.value()[0],
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1);
            }

            // ======== 中心 R ========
            if (single_rune_blade_2D.constrained_contours.center_R_opt.has_value())
            {
                cv::Scalar color = single_rune_blade_2D.is_center_R_usable
                                       ? cv::Scalar(0, 255, 0) // 白
                                       : cv::Scalar(80, 80, 80);   // 灰

                std::vector<std::vector<cv::Point>> contours;
                contours.push_back(single_rune_blade_2D.constrained_contours.center_R_opt.value());

                cv::drawContours(ori_img, contours, -1, color, 2);

                cv::putText(
                    ori_img,
                    single_rune_blade_2D.is_center_R_usable ? "CenterR" : "CenterR (X)",
                    single_rune_blade_2D.constrained_contours.center_R_opt.value()[0],
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.5,
                    color,
                    1);
            }

            // ======== Rune 状态（直接贴在装甲板轮廓第一个点附近） ========
            if (single_rune_blade_2D.constrained_contours.armor_module_opt.has_value())
            {
                std::string state_text;
                switch (single_rune_blade_2D.rune_state)
                {
                case RuneState::BigInactive:
                    state_text = "Big-I";
                    break;
                case RuneState::BigActivated:
                    state_text = "Big-A";
                    break;
                case RuneState::SmallInactive:
                    state_text = "Small-I";
                    break;
                case RuneState::SmallActivated:
                    state_text = "Small-A";
                    break;
                }

                cv::putText(
                    ori_img,
                    state_text,
                    single_rune_blade_2D.constrained_contours.armor_module_opt.value()[0] + cv::Point(5, 15),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.6,
                    cv::Scalar(255, 255, 255),
                    2);
            }
        }
        ImgViz::enqueue_image_zero_copy("PowerRune/Contours", ori_img);
    }
#pragma endregion

    if(refined_rune_observation.rune_blade_2D.empty())
    {
        //没有传统识别结果,直接返回
        return;
    }

#pragma region 
    //符平面
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD_REPROJECT_IMG"])
    {
        m_power_rune_plane.set_debug_rune_targets(m_debug_rune_targets);
    }
#pragma endregion
    
    //更新符平面
    m_power_rune_plane.update_power_rune_plane(refined_rune_observation);    
    InactiveTargets inactive_targets = m_power_rune_plane.get_inactive_targets();
    inactive_targets.is_big_rune = rune_observation.is_big_rune;

    //没有结果直接返回
    if (!inactive_targets.is_vaild)
    {
        return;
    }
    
    //估计相位和运动方程
    m_phase_motion_estimator.estimate_phase_motion(inactive_targets);
    std::optional<std::vector<RuneTarget>> rune_targets_opt = m_phase_motion_estimator.try_get_rune_targets();
    
    //没有结果直接跳过
    if (!rune_targets_opt.has_value())
    {
        return;
    }
    
    //运行到此处说明有滤波后的数据
    std::vector<RuneTarget> rune_targets = std::move(rune_targets_opt.value());

#pragma region 
    if ((int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD"] &&
        (int)J_POWER_RUNE.config_["debug"]["POWER_RUNE_DEBUG_REBUILD_REPROJECT_IMG"])
    {
        m_debug_rune_targets = rune_targets;
    }
#pragma endregion

    m_rune_decision_module.update_decision_module(std::move(rune_targets));

}

RuneObservation PowerRuneProcessor::convert2rune_observation(const power_rune::RuneInput &detect_input)
{
    RuneObservation rune_observation;
    rune_observation.is_big_rune = detect_input.is_big_rune;
    rune_observation.ori_img = detect_input.ori_mat;
    rune_observation.tf_tree = detect_input.tf_tree;

    rune_observation.timestamp = PRF::timestamp_to_foxglove_time(detect_input.timestamp);
    
    rune_observation.rune_infos.reserve(detect_input.nn_rune_infos.size());
    for (const auto &NN_rune_info : detect_input.nn_rune_infos)
    {
        // 关键点集
        std::vector<cv::Point2i> points;
        points.reserve(5);
        points.push_back(NN_rune_info.top);
        points.push_back(NN_rune_info.left);
        points.push_back(NN_rune_info.point_R);
        points.push_back(NN_rune_info.right);
        points.push_back(NN_rune_info.bottom);


        // 拟合椭圆，扩展长短边
        cv::RotatedRect rotated_rect = cv::minAreaRect(points);
        rotated_rect.size = rotated_rect.size * (float)J_POWER_RUNE.config_["detect"]["extand_rotated_rect"];

        // 视图
        cv::Rect view_rect = rotated_rect.boundingRect();
        view_rect &= cv::Rect(0, 0, detect_input.ori_mat.cols, detect_input.ori_mat.rows);
        if (view_rect.empty())
        {
            LOG(ERROR)<<"[convert2rune_observation]错误的view_rect";
            continue;
        }

        // 赋值
        RuneInfo rune_info;
        rune_info.top = NN_rune_info.top;
        rune_info.left = NN_rune_info.left;
        rune_info.bottom = NN_rune_info.bottom;
        rune_info.right = NN_rune_info.right;
        rune_info.point_R = NN_rune_info.point_R;
        rune_info.class_id = NN_rune_info.class_id;
        rune_info.rotated_rect = rotated_rect;
        rune_info.view_rect = view_rect;
        rune_info.view = detect_input.ori_mat(view_rect).clone();
        rune_info.color = detect_input.cd_my_color == 0 ? rune_info.RED : rune_info.BLUE;
        rune_observation.rune_infos.emplace_back(std::move(rune_info));
    }

    return rune_observation;
}
