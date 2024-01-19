#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <eigen3/Eigen/Dense>

class Kinematics {
public:
    Kinematics() = default;
    ~Kinematics() = default;

    const int RHO_OPT_SIZE = 3; // rho_opt代表接触的偏置cx, cy, cz
    const int RHO_FIX_SIZE = 5; // rho_fix代表机体偏置x和y, thigh关节偏置, 大腿长度, 小腿长度

    // 正运动学 3x1
    Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // 雅可比 3x3
    Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // 正运动学矩阵相对于rho_opt的偏导 3x3
    Eigen::Matrix3d dfk_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // 雅可比矩阵相对于q的偏导 9x3
    Eigen::Matrix<double, 9, 3> dJ_dq(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // 雅可比矩阵相对于rho_opt的偏导 9x3
    Eigen::Matrix<double, 9, 3> dJ_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

private:
    // Matlab生成函数
    void autoFunc_fk_derive(const double in1[3], const double in2[3], const double in3[5], double p_bf[3]);
    void autoFunc_d_fk_dq(const double in1[3], const double in2[3], const double in3[5], double jacobian[9]);
    void autoFunc_d_fk_dc(const double in1[3], const double in2[3], const double in3[5], double d_fk_dc[9]);
    void autoFunc_dJ_dq(const double in1[3], const double in2[3], const double in3[5], double dJ_dq[27]);
    void autoFunc_dJ_dpho(const double in1[3], const double [3], const double [5], double dJ_dpho[27]);
};

#endif //VILEOM_A1KINEMATICS_H
