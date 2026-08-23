#include"PowerRuneBallisticModel.hpp"

SmallRuneBallisticModel::SmallRuneBallisticModel(
    Eigen::Vector3d rune_center,
    Eigen::Vector3d start_vector,
    Eigen::Vector3d rune_plane_normal,
    double rune_radius,
    double rune_w,
    double armor_phase,
    double gune_length,
    double V_bullet,
    double k_total_coefficient,
    double gravity,
    double magnus_coefficient,
    double algorithmic_time,
    double delay_time
)
    : m_rune_center(rune_center),
      m_start_vector(start_vector.normalized()),
      m_rune_plane_normal(rune_plane_normal.normalized()),
      m_rune_radius(rune_radius),
      m_rune_w(rune_w),
      m_armor_phase(armor_phase),
      m_gune_length(gune_length),
      m_V_bullet(V_bullet),
      m_k_total_coefficient(k_total_coefficient),
      m_gravity(gravity),
      m_magnus_coefficient(magnus_coefficient),
      m_algorithmic_time(algorithmic_time),
      m_delay_time(delay_time)
{
}

BigRuneBallisticModel::BigRuneBallisticModel(
    Eigen::Vector3d rune_center,
    Eigen::Vector3d start_vector,
    Eigen::Vector3d rune_plane_normal,
    double rune_radius,
    const RuneTarget::BigRuneMotionModelParams &motion_model,
    double capture_to_reference_time_s,
    double gune_length,
    double V_bullet,
    double k_total_coefficient,
    double gravity,
    double magnus_coefficient,
    double algorithmic_time,
    double delay_time)
    : m_rune_center(rune_center),
      m_start_vector(start_vector.normalized()),
      m_rune_plane_normal(rune_plane_normal.normalized()),
      m_rune_radius(rune_radius),
      m_phase_cos_coefficient(motion_model.phase_cos_coefficient),
      m_phase_sin_coefficient(motion_model.phase_sin_coefficient),
      m_phase_linear_velocity(motion_model.phase_linear_velocity),
      m_phase_constant_offset_radians(motion_model.phase_constant_offset_radians),
      m_speed_angular_frequency(motion_model.speed_angular_frequency),
      m_capture_to_reference_time_s(capture_to_reference_time_s),
      m_gune_length(gune_length),
      m_V_bullet(V_bullet),
      m_k_total_coefficient(k_total_coefficient),
      m_gravity(gravity),
      m_magnus_coefficient(magnus_coefficient),
      m_algorithmic_time(algorithmic_time),
      m_delay_time(delay_time)
{
}
