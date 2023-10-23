#ifndef UTILS_H
#define UTILS_H

#include <iostream>
#include <vector>
#include <Eigen/Dense>

#include "Param.h"

class Utils {
    public:
        // 四元数转换成欧拉角，返回偏航角，从-pi到pi
        static Eigen::Vector3d quat_to_euler(Eigen::Quaterniond quat);

        // 生成向量对应的反对称矩阵
        static Eigen::Matrix3d skew(Eigen::Vector3d vec);

        // 计算矩阵的伪逆
        static Eigen::Matrix3d pseudo_inverse(const Eigen::Matrix3d &mat);

        // 计算二面角
        static double cal_dihedral_angle(Eigen::Vector3d surf_coef_1, Eigen::Vector3d surf_coef_2);
};

class BezierUtils {
    public:
        BezierUtils () {
            curve_constructed = false;
            bezier_degree = 4;
        }
        
        // 根据时间，起始点，落足点和地面俯仰角生成贝塞尔曲线
        Eigen::Vector3d get_foot_pos_curve(float t,
                                        Eigen::Vector3d foot_pos_start,
                                        Eigen::Vector3d foot_pos_final,
                                        double terrain_pitch_angle);

        bool reset_foot_pos_curve() {
            curve_constructed = false;
        }
    private:
        // 贝塞尔曲线函数
        double bezier_curve(double t, const std::vector<double> &P);

        // 曲线是否构建
        bool curve_constructed;

        // 贝塞尔曲线维数
        float bezier_degree;
};

#endif //UTILS_H
