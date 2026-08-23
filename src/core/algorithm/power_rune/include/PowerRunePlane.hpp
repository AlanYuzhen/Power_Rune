#pragma once
#include<Eigen/Geometry>
#include<Eigen/Dense>
#include<Eigen/Core>
#include<ceres/ceres.h>
#include<ceres/rotation.h>
#include <ceres/jet.h>
#include "common/power_rune_global.hpp"




//相似性误差残差块
class ChamferResidual
{
public:
    ChamferResidual(const float *distance_transform,
                    cv::Rect distance_transform_rect,
                    const Eigen::Vector3d &point_world,
                    double max_chamfer_residual,
                    double fx, double fy, double cx, double cy,
                    const Eigen::Matrix3d &r_car_from_camera,
                    double axis_y_constraint_weight)
        : m_distance_transform(distance_transform),           // 去畸变的距离变换函数
          m_distance_transform_rect(distance_transform_rect), // 距离变换函数的界
          m_point_world(point_world),                         // 模型点
          m_max_chamfer_residual(max_chamfer_residual),       // 单点最大可接受chamfer_residual
          m_fx(fx),                                           // fx
          m_fy(fy),                                           // fy
          m_cx(cx),                                           // cx
          m_cy(cy),                                           // cy
          m_r10(r_car_from_camera(1, 0)),
          m_r11(r_car_from_camera(1, 1)),
          m_r12(r_car_from_camera(1, 2)),
          m_axis_y_constraint_weight(axis_y_constraint_weight)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residual) const
    {
        // pose = [rvec_x, rvec_y, rvec_z, tx, ty, tz]
        const T point_world[3] = {
            T(m_point_world.x()),
            T(m_point_world.y()),
            T(m_point_world.z())
        };
        T point_camera[3];
        ceres::AngleAxisRotatePoint(pose, point_world, point_camera);
        point_camera[0] += pose[3];
        point_camera[1] += pose[4];
        point_camera[2] += pose[5];

        //点不允许在相机的后方
        if (point_camera[2] <= T(1e-6))
        {
            residual[0] = T(m_max_chamfer_residual);
            residual[1] = T(0);
            return true;
        }

        //小孔成像
        const T u = T(m_fx) * point_camera[0] / point_camera[2] + T(m_cx);
        const T v = T(m_fy) * point_camera[1] / point_camera[2] + T(m_cy);
    
        //计算该点的chamfer_residual
        T chamfer_residual = sample_chamfer_residual(u,v);

        //残差截断避免错误点对整体优化造成过大影响(可能Huber更有用)
        if (chamfer_residual > T(m_max_chamfer_residual))
        {
            chamfer_residual = T(m_max_chamfer_residual);
        }

        //返回
        residual[0] = chamfer_residual;

        // 法向量约束：(0,0,1) 转到车系后 y 分量尽量为 0
        const T model_z_axis[3] = {T(0), T(0), T(1)};
        T z_axis_in_camera[3];
        ceres::AngleAxisRotatePoint(pose, model_z_axis, z_axis_in_camera);
        const T y_in_car = T(m_r10) * z_axis_in_camera[0]
                         + T(m_r11) * z_axis_in_camera[1]
                         + T(m_r12) * z_axis_in_camera[2];
        residual[1] = T(m_axis_y_constraint_weight) * y_in_car;
        return true;

    }

private:
    const float *m_distance_transform;     // 距离变换函数
    const cv::Rect m_distance_transform_rect;//距离变换函数的rect(限制边界和记录偏置)
    const Eigen::Vector3d &m_point_world;   // 模型点坐标
    const double m_max_chamfer_residual;   //单点允许的最大残差
    // 相机内参
    const double m_fx;
    const double m_fy;
    const double m_cx;
    const double m_cy;
    const double m_r10;
    const double m_r11;
    const double m_r12;
    const double m_axis_y_constraint_weight;


    // 获得chamfer_residual(离散值)
    template <typename T>
    T get_chamfer_residual(int index_x, int index_y) const
    {
        if (index_x < 0 || index_x >= m_distance_transform_rect.width || index_y < 0 || index_y >= m_distance_transform_rect.height)
        {
            // 说明范围大于函数
            return T(m_max_chamfer_residual);
        }

        // 返回离散值
        return T(m_distance_transform[index_y * m_distance_transform_rect.width + index_x]);
    }

    template <typename T>
    inline double get_scalar(const T &x) const
    {
        return x;
    }

    template <typename T, int N>
    inline double get_scalar(const ceres::Jet<T, N> &x) const
    {
        return x.a;
    }

    // 通过二次线性差值对距离变换函数进行连续化,返回连续值
    template <typename T>
    T sample_chamfer_residual(const T &u, const T &v) const
    {

        // 左上角整数像素
        T u0 = ceres::floor(u);
        T v0 = ceres::floor(v);

        // 得到索引(要减去偏置变换到ROI)，不参与导数计算
        int index_x0 = static_cast<int>(get_scalar(u0)) - m_distance_transform_rect.x;
        int index_y0 = static_cast<int>(get_scalar(v0)) - m_distance_transform_rect.y;

        //插值权重(让导数连续化的重要手段)
        T du = u - u0;
        T dv = v - v0;

        // 双线性插值
        return (T(1) - du) * (T(1) - dv) * get_chamfer_residual<T>(index_x0, index_y0) 
            + du * (T(1) - dv) * get_chamfer_residual<T>(index_x0 + 1, index_y0) 
            + (T(1) - du) * dv * get_chamfer_residual<T>(index_x0, index_y0 + 1) 
            + du * dv * get_chamfer_residual<T>(index_x0 + 1, index_y0 + 1);
    
    }
};

// 锚点重投影误差残差块（用于约束PnP锚点）
class AnchorReprojResidual
{
public:
    AnchorReprojResidual(const Eigen::Vector3d &point_world,
                         const cv::Point2f &pixel_observe,
                         double fx, double fy, double cx, double cy,
                         double residual_weight)
        : m_point_world(point_world),
          m_pixel_u(pixel_observe.x),
          m_pixel_v(pixel_observe.y),
          m_fx(fx), m_fy(fy), m_cx(cx), m_cy(cy),
          m_residual_weight(residual_weight)
    {
    }

    template <typename T>
    bool operator()(const T *const pose, T *residual) const
    {
        const T point_world[3] = {
            T(m_point_world.x()),
            T(m_point_world.y()),
            T(m_point_world.z())
        };
        T point_camera[3];
        ceres::AngleAxisRotatePoint(pose, point_world, point_camera);
        point_camera[0] += pose[3];
        point_camera[1] += pose[4];
        point_camera[2] += pose[5];

        if (point_camera[2] <= T(1e-6))
        {
            residual[0] = T(m_residual_weight * 1000.0);
            residual[1] = T(m_residual_weight * 1000.0);
            return true;
        }
        
        //TODO：可以尝试把损失计算改为非线性的。
        const T u = T(m_fx) * point_camera[0] / point_camera[2] + T(m_cx);
        const T v = T(m_fy) * point_camera[1] / point_camera[2] + T(m_cy);
        residual[0] = T(m_residual_weight) * (u - T(m_pixel_u));
        residual[1] = T(m_residual_weight) * (v - T(m_pixel_v));
        return true;
    }

private:
    const Eigen::Vector3d m_point_world;
    const double m_pixel_u;
    const double m_pixel_v;
    const double m_fx;
    const double m_fy;
    const double m_cx;
    const double m_cy;
    const double m_residual_weight;
};

class Projector
{
public:
    Projector();
    bool caculate_pose(const std::vector<SingleRuneBlade2D> &rune_blade_2D,
                       const transform_tools::TFTree& tf_tree,
                       const cv::Mat &ori_img);

    const Eigen::Vector<double,6> get_pose() const;
    const std::vector<const Eigen::Vector3d*>& get_sampled_model_points() const;
    std::vector<int> get_inactive_target_location_nums() const;
    const int get_another_inactivate_target_location_num() const;
private:
    //单个符叶匹配对
    struct RuneBladeCorrespondence
    {
        RuneState rune_state;//符的类型
        int location_num;//位置
        std::vector<std::shared_ptr<std::vector<Eigen::Vector3d>>> model_rune_points;//模型符
        std::vector<std::vector<cv::Point>> matched_contours;//轮廓
        std::vector<cv::Point2f> anchor_points;//锚点
        Eigen::Vector3d direction_vector;//方向向量
    };

    std::vector<RuneBladeCorrespondence> m_rune_correspondence;//整个符的所有匹配对
    std::vector<const Eigen::Vector3d*> m_sampled_model_points;//下采样得到的用于优化的模型点
    int m_another_inactivate_target_location_num = -1;//对于大符来说另一个未激活目标的位置

    bool construct_rune_correspondence(const std::vector<SingleRuneBlade2D> &rune_blade_2D);//符的匹配对
    void set_location(const SingleRuneBlade2D &single_rune_blade_2D);//设置其他位置
    void set_location_x(int location_x,const SingleRuneBlade2D &single_rune_blade_2D);//对x位置进行初始化
    const RuneBladeCorrespondence* get_location_x_correspondence(int location_x) const;//获取x位置的指针

    std::vector<cv::Point2f> calculate_anchor_points(const SingleRuneBlade2D &single_rune_blade_2D);//计算锚点
    std::vector<cv::Point3f> calculate_inactive_anchor_points_world(int location_x) const;//按位置号生成未激活目标的锚点
    void find_anchor_points(cv::Point2f pixel_points[4],const SingleRuneBlade2D &single_rune_blade_2D);//外接矩形锚点
    void sort_anchor_points(cv::Point2f pixel_points[4]);//给外接矩形锚点角点排序
    void find_armor_center_anchor_points(const std::vector<SingleRuneBlade2D> &rune_blade_2D);//TODO：对于已经激活的小符,可以计算其装甲板中心作为锚点
    
    void calculate_distance_transfor();//计算距离变换函数
    bool minimize_chamfer_distance();//model-base优化
    void sample_model_points();//对模型点进行下采样

    bool is_valid_for_projection(const SingleRuneBlade2D &single_rune_blade_2D);//判断是否可以用于投影
    Eigen::Vector3d calculate_direction_vector(const SingleRuneBlade2D &single_rune_blade_2D);//从符心到靶心的方向向量

    transform_tools::TFTree m_tf_tree;//tf_tree
    double m_max_chamfer_residual;//单点允许的最大残差
    cv::Point2f m_rune_center;//符心
    cv::Mat m_binary_buffer;//预分配的内存用于二值化
    cv::Rect m_distance_transform_rect;//距离变换函数的rect(限制边界和记录偏置)
    cv::Mat m_distance_transform;//距离变换函数
    Eigen::Vector<double,6> m_pose;//姿态(轴角表示)


private:
    //模型符对应的指针

    //未激活的符叶的模型锚框点
    std::shared_ptr<std::vector<cv::Point3f>> m_anchor_inactive_points_world_ptr;

    //未激活的符
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_inactive_points_model_1_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_inactive_points_model_2_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_inactive_points_model_3_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_inactive_points_model_4_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_inactive_points_model_5_ptr;

    //已激活的小符的装甲板模块
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_armor_model_1_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_armor_model_2_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_armor_model_3_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_armor_model_4_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_armor_model_5_ptr;

    //已激活的小符的灯臂
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_light_arm_model_1_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_light_arm_model_2_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_light_arm_model_3_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_light_arm_model_4_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_small_power_rune_active_light_arm_model_5_ptr;

    //已激活的大符的灯臂
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_big_power_rune_active_points_model_1_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_big_power_rune_active_points_model_2_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_big_power_rune_active_points_model_3_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_big_power_rune_active_points_model_4_ptr;
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_big_power_rune_active_points_model_5_ptr;

};

struct PlanePoints
{
    std::vector<Eigen::Vector3d> plane_points;//车系下的点
    foxglove::Time timestamp_capture;//这些点对应的原图的拍摄时间
};

class PowerRunePlane 
{
public:
    PowerRunePlane();
    
    //更新平面
    void update_power_rune_plane(const RefinedRuneObservation &refined_rune_observation);
    void set_debug_rune_targets(std::vector<RuneTarget> rune_targets);
    
    //返回未激活目标
    const InactiveTargets get_inactive_targets() const;
private:
    std::shared_ptr<std::vector<cv::Point3f>> m_anchor_inactive_points_world_ptr;//未激活的符叶的模型锚框点
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_plane_points_ptr;//用于拟合平面的点
    std::shared_ptr<std::vector<Eigen::Vector3d>> m_power_rune_box_exoskeleton_points_ptr;//可视化用


    
    
    //点云滑窗口(车系)
    std::deque<PlanePoints> m_plane_points_window;
    void update_plane_points_window(PlanePoints &&plane_points);

    //符平面参数(车系)
    Eigen::Hyperplane<double,3> m_power_rune_plane;
    bool estimate_power_rune_plane();
    
    //未激活目标(车系)
    InactiveTargets m_inactive_targets;
    void optimize_inactive_targets();

    Projector m_projector;//新的投影类

    std::vector<RuneTarget> m_debug_rune_targets;
    

};
