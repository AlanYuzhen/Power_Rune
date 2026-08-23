#pragma once
#include <array>
#include<Eigen/Geometry>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<ceres/ceres.h>
#include<ceres/rotation.h>
#include <ceres/jet.h>
#include "common/power_rune_global.hpp"

//符的弹道模型

//小符的状态方程
template <typename T>
struct SmallRuneBallisticODE
{
    T drag_coefficient;
    T gravity;
    T magnus_coefficient;// 马格努斯样力系数

    void operator()(const std::array<T, 4>& current_state,
                    std::array<T, 4>& derivative_of_state,
                    const T& current_time) const
    {
        const T& d = current_state[0];   // 水平距离
        const T& y = current_state[1];   // 垂直位置
        const T& v_d = current_state[2]; // 水平速度
        const T& v_y = current_state[3]; // 垂直速度

        //和速度
        const T v2 = v_d * v_d + v_y * v_y;
        const T v  = ceres::sqrt(v2 + T(1e-12));

        //空气阻力(与速度方向反向)
        const T acceleration_drag_d = - drag_coefficient * v * v_d;//水平方向
        const T acceleration_drag_y = - drag_coefficient * v * v_y;//y方向

        //马格努斯力(默认垂直于速度向上)
        const T acceleration_magnus_d = magnus_coefficient * v * v_y;//水平方向
        const T acceleration_magnus_y = -magnus_coefficient * v * v_d;//y方向

        derivative_of_state[0] = v_d;//水平距离的导数
        derivative_of_state[1] = v_y;//垂直距离的导数
        derivative_of_state[2] = acceleration_drag_d + acceleration_magnus_d;//水平方向速度的导数
        derivative_of_state[3] = acceleration_drag_y + gravity + acceleration_magnus_y;//垂直方向速度的导数
    }
};

//小符的弹道模型
class SmallRuneBallisticModel
{
public:
    SmallRuneBallisticModel(
        Eigen::Vector3d rune_center,        // 符心
        Eigen::Vector3d start_vector, // 0相位向量
        Eigen::Vector3d rune_plane_normal,  // 符平面靶向量
        double rune_radius,                 //半径
        double rune_w,                      // 角速度(已经含有方向)
        double armor_phase,                 // 相位
        double gune_length,                 // 枪口长度
        double V_bullet,                    // 弹丸初速
        double k_total_coefficient,         // 总空气阻力系数
        double gravity,                     // 重力
        double magnus_coefficient,         // 马格努斯加速度系数
        double algorithmic_time,            // 算法开销
        double delay_time                   // 预留给云台的响应量
    );

    template <typename T>
    bool operator()(const T *const param, T *residuals) const
    {   
        const T& flying_time = param[0];//弹丸飞行时间
        const T& yaw         = param[1];//弹丸出射的yaw角
        const T& pitch   = param[2];//弹丸出射的pitch角

        //命中时刻靶心的状态
        T t_armor = T(m_algorithmic_time) + T(m_delay_time) + T(flying_time);
        T theta = T(m_rune_w) * t_armor + T(m_armor_phase);

        //利用罗德里格斯公式进行旋转(由于起始向量和法向量垂直，所以公式进行了简化)
        Eigen::Matrix<T, 3, 1> rune_plane_normal = m_rune_plane_normal.cast<T>();
        Eigen::Matrix<T, 3, 1> start_vector = (m_rune_radius * m_start_vector).cast<T>();
        Eigen::Matrix<T, 3, 1> target_vector =  start_vector * ceres::cos(theta) + rune_plane_normal.cross(start_vector) * ceres::sin(theta);
        Eigen::Matrix<T, 3, 1> armor_center =  m_rune_center.cast<T>() + target_vector;
        T X_armor_predict = armor_center.x();
        T Y_armor_predict = armor_center.y();
        T Z_armor_predict = armor_center.z();


        //弹丸初始状态
        std::array<T, 4> state;
        state[0] =  T(m_gune_length) * ceres::cos(pitch);//水平位置
        state[1] = -T(m_gune_length) * ceres::sin(pitch);//垂直位置
        state[2] =  m_V_bullet * ceres::cos(pitch);//水平速度
        state[3] = -m_V_bullet * ceres::sin(pitch);//垂直速度

        //计算出飞行后弹丸的状态
        integrate_rk4_fixed(flying_time, state);
        const T d_final = state[0];
        const T y_final = state[1];
        const T X_bullet_predict = d_final * ceres::sin(yaw);
        const T Z_bullet_predict = d_final * ceres::cos(yaw);
        const T Y_bullet_predict = y_final;

        //计算残差  
        residuals[0] = X_bullet_predict - X_armor_predict;
        residuals[1] = Y_bullet_predict - Y_armor_predict;
        residuals[2] = Z_bullet_predict - Z_armor_predict;

        return true;
    }

private:
    template <typename T>
    void integrate_rk4_fixed(const T& total_time, std::array<T, 4>& state) const
    {
        // 使用带马格努斯效应的ODE
        SmallRuneBallisticODE<T> ode{
            T(m_k_total_coefficient), 
            T(m_gravity),
            T(m_magnus_coefficient)
        };

        constexpr int kSteps = 52;//52是迭代次数
        const T dt = total_time / T(kSteps);
        T t = T(0);

        for (int i = 0; i < kSteps; ++i)
        {
            //使用rk4步进器计算
            rk4_step(ode, state, t, dt);
            t += dt;
        }
    }

    template <typename T>
    void rk4_step(const SmallRuneBallisticODE<T> &ode, std::array<T, 4> &state, const T &t, const T &dt) const
    {
        std::array<T, 4> k1, k2, k3, k4, tmp;

        ode(state, k1, t);

        for (int i = 0; i < 4; ++i)
            tmp[i] = state[i] + k1[i] * dt * T(0.5);
        ode(tmp, k2, t + dt * T(0.5));

        for (int i = 0; i < 4; ++i)
            tmp[i] = state[i] + k2[i] * dt * T(0.5);
        ode(tmp, k3, t + dt * T(0.5));

        for (int i = 0; i < 4; ++i)
            tmp[i] = state[i] + k3[i] * dt;
        ode(tmp, k4, t + dt);

        for (int i = 0; i < 4; ++i)
        {
            state[i] += (k1[i] + T(2.0) * k2[i] + T(2.0) * k3[i] + k4[i]) * dt / T(6.0);
        }
    }

private:
    
    Eigen::Vector3d m_rune_center;//符心
    Eigen::Vector3d m_start_vector;//0相位向量
    Eigen::Vector3d m_rune_plane_normal;//符平面靶向量
    double m_rune_radius;
    double m_rune_w;//角速度(已经含有方向)
    double m_armor_phase;//相位
    double m_gune_length;//枪口长度
    double m_V_bullet;//弹丸初速
    double m_k_total_coefficient;//总空气阻力系数
    double m_gravity;//重力
    double m_magnus_coefficient;//马格努斯样力系数
    double m_algorithmic_time;//算法开销
    double m_delay_time;//预留给云台的响应量
};

//大符的弹道模型（相位由拟合运动方程预测）
class BigRuneBallisticModel
{
public:
    BigRuneBallisticModel(
        Eigen::Vector3d rune_center,        // 符心
        Eigen::Vector3d start_vector,       // 0相位向量
        Eigen::Vector3d rune_plane_normal,  // 符平面法向量（单位向量）
        double rune_radius,                 // 半径
        const RuneTarget::BigRuneMotionModelParams &motion_model, // 相位运动方程
        double capture_to_reference_time_s, // capture_timestamp - reference_timestamp（秒）
        double gune_length,                 // 枪口长度
        double V_bullet,                    // 弹丸初速
        double k_total_coefficient,         // 总空气阻力系数
        double gravity,                     // 重力
        double magnus_coefficient,          // 马格努斯加速度系数
        double algorithmic_time,            // 算法开销
        double delay_time                   // 预留给云台的响应量
    );

    template <typename T>
    bool operator()(const T *const param, T *residuals) const
    {
        const T &flying_time = param[0];
        const T &yaw = param[1];
        const T &pitch = param[2];

        // 命中时刻相对 reference_timestamp 的时间（秒）
        const T t_hit_from_ref =
            T(m_capture_to_reference_time_s) + T(m_algorithmic_time) + T(m_delay_time) + T(flying_time);

        // 命中时刻相位（连续相位）
        const T omega_t = T(m_speed_angular_frequency) * t_hit_from_ref;
        const T phase =
            T(m_phase_cos_coefficient) * ceres::cos(omega_t) +
            T(m_phase_sin_coefficient) * ceres::sin(omega_t) +
            T(m_phase_linear_velocity) * t_hit_from_ref +
            T(m_phase_constant_offset_radians);

        // 目标三维位置
        Eigen::Matrix<T, 3, 1> rune_plane_normal = m_rune_plane_normal.cast<T>();
        Eigen::Matrix<T, 3, 1> start_vector = (m_rune_radius * m_start_vector).cast<T>();
        Eigen::Matrix<T, 3, 1> target_vector =
            start_vector * ceres::cos(phase) + rune_plane_normal.cross(start_vector) * ceres::sin(phase);
        Eigen::Matrix<T, 3, 1> armor_center = m_rune_center.cast<T>() + target_vector;
        const T X_armor_predict = armor_center.x();
        const T Y_armor_predict = armor_center.y();
        const T Z_armor_predict = armor_center.z();

        // 弹丸初始状态
        std::array<T, 4> state;
        state[0] = T(m_gune_length) * ceres::cos(pitch);
        state[1] = -T(m_gune_length) * ceres::sin(pitch);
        state[2] = m_V_bullet * ceres::cos(pitch);
        state[3] = -m_V_bullet * ceres::sin(pitch);

        // 计算出飞行后弹丸的状态
        integrate_rk4_fixed(flying_time, state);
        const T d_final = state[0];
        const T y_final = state[1];
        const T X_bullet_predict = d_final * ceres::sin(yaw);
        const T Z_bullet_predict = d_final * ceres::cos(yaw);
        const T Y_bullet_predict = y_final;

        residuals[0] = X_bullet_predict - X_armor_predict;
        residuals[1] = Y_bullet_predict - Y_armor_predict;
        residuals[2] = Z_bullet_predict - Z_armor_predict;
        return true;
    }

private:
    template <typename T>
    void integrate_rk4_fixed(const T &total_time, std::array<T, 4> &state) const
    {
        SmallRuneBallisticODE<T> ode{
            T(m_k_total_coefficient),
            T(m_gravity),
            T(m_magnus_coefficient)};

        constexpr int kSteps = 52;
        const T dt = total_time / T(kSteps);
        T t = T(0);

        for (int i = 0; i < kSteps; ++i)
        {
            rk4_step(ode, state, t, dt);
            t += dt;
        }
    }

    template <typename T>
    void rk4_step(const SmallRuneBallisticODE<T> &ode, std::array<T, 4> &state, const T &t, const T &dt) const
    {
        std::array<T, 4> k1, k2, k3, k4, tmp;

        ode(state, k1, t);

        for (int i = 0; i < 4; ++i)
            tmp[i] = state[i] + k1[i] * dt * T(0.5);
        ode(tmp, k2, t + dt * T(0.5));

        for (int i = 0; i < 4; ++i)
            tmp[i] = state[i] + k2[i] * dt * T(0.5);
        ode(tmp, k3, t + dt * T(0.5));

        for (int i = 0; i < 4; ++i)
            tmp[i] = state[i] + k3[i] * dt;
        ode(tmp, k4, t + dt);

        for (int i = 0; i < 4; ++i)
        {
            state[i] += (k1[i] + T(2.0) * k2[i] + T(2.0) * k3[i] + k4[i]) * dt / T(6.0);
        }
    }

private:
    Eigen::Vector3d m_rune_center;
    Eigen::Vector3d m_start_vector;
    Eigen::Vector3d m_rune_plane_normal;
    double m_rune_radius;

    double m_phase_cos_coefficient;
    double m_phase_sin_coefficient;
    double m_phase_linear_velocity;
    double m_phase_constant_offset_radians;
    double m_speed_angular_frequency;
    double m_capture_to_reference_time_s;

    double m_gune_length;
    double m_V_bullet;
    double m_k_total_coefficient;
    double m_gravity;
    double m_magnus_coefficient;
    double m_algorithmic_time;
    double m_delay_time;
};
