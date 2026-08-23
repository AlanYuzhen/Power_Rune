#pragma once
#include "common/power_rune_global.hpp"
#include <optional>

// 描述符
class ContourDescriptor
{
public:
    virtual ~ContourDescriptor() = default;
    virtual bool is_usable() const = 0;//是否能用于投影优化

protected:

    std::vector<cv::Point> m_contour;//轮廓
    cv::Rect m_ori_img_rect;//原图对应的矩形
    
    //构造函数
    explicit ContourDescriptor(std::vector<cv::Point> contour, cv::Rect ori_img_rect);
    
    //判断轮廓是否靠近边缘
    bool is_contour_near_image_border();
};

//椭圆形轮廓描述符(装甲板模块)
class EllipseDescriptor : public ContourDescriptor
{
public:
    explicit EllipseDescriptor(std::vector<cv::Point> contour, cv::Rect ori_img_rect);
    bool is_usable() const override;
    double get_solidity() const;
    bool is_near_image_border() const;

private:
    bool m_is_near_image_border;//是否靠近图片边缘
    double m_solidity;//实心度，用来衡量凹陷程度(输出的时候几乎没有出现过0.97以下)

};

//矩形描述符(未激活符叶的灯臂)
class RectangularDescriptor : public ContourDescriptor
{
public:
    explicit RectangularDescriptor(std::vector<cv::Point> contour, cv::Rect ori_img_rect);
    bool is_usable() const override;
    double get_solidity() const;
    float get_aspect_ratio() const;

private:
    double m_solidity;//实心度，用来衡量凹陷程度(输出的时候几乎没有出现过0.97以下)
    float m_aspect_ratio;//长宽比
    
};

//山形描述符(已激活符叶的灯臂)
class PeakProfileDescriptor : public ContourDescriptor
{
public:
    explicit PeakProfileDescriptor(std::vector<cv::Point> contour, cv::Rect ori_img_rect, bool is_big_rune);
    bool is_usable() const override;
    const std::vector<cv::Point> get_m_approx_contour() const;
    const int get_approx_contour_size() const;
    bool is_big_rune() const;
    bool is_approx_contour_size_valid() const;
    bool is_near_image_border() const;

private:
    std::vector<cv::Point> m_approx_contour;//近似多边形的拐点
    int m_approx_contour_size;//拐点的数目
    bool m_is_big_rune;//是否为大符的未激活灯臂
    bool m_is_approx_contour_size_valid;//拐点数目是否满足条件
    bool m_is_near_image_border;//是否靠近图片边缘（因为大符的灯臂是没有对应的装甲板模块的，所以需要作这个检验）

};

// 这个类负责获取对网络识别的结果进行修正。
class RuneObservationRefiner
{
public:
    
    //二值化，特征选取，特征匹配，重新获取特征点。
    RefinedRuneObservation refine(const RuneObservation &rune_observation);

private:

    //提取轮廓
    std::vector<std::vector<cv::Point>> extract_contours(const RuneInfo &rune_info, RuneInfo::Color color);
    
    //轮廓约束和分类
    ConstrainedContours constrain_contours(const RuneInfo &rune_info, const std::vector<std::vector<cv::Point>> &contours);

    //构造单片符叶
    SingleRuneBlade2D construct_blade(bool is_big_rune, const RuneInfo &rune_info, ConstrainedContours &&constrained_contours, const cv::Rect &ori_img_rect);

    //工具函数,判断直线是否经过轮廓
    bool is_line_pass_through_contour(const cv::Point2f &A,const cv::Point2f &B,const std::vector<cv::Point> &contour,int samples);

    //对矩形灯臂的轮廓进行处理，使其更加接近矩形
    void rebuild_rectangle(std::vector<cv::Point> &contour,cv::Rect ori_img_rect);
};
