#ifndef KINEMATICS_H
#define KINEMATICS_H

#include <eigen3/Eigen/Dense>

class Kinematics {
public:
    Kinematics() = default;
    ~Kinematics() = default;

    // rho_opt代表接触的偏置cx, cy, cz
    // rho_fix代表机体偏置x和y, thigh关节偏置, 大腿长度, 小腿长度
    const int RHO_OPT_SIZE = 3;
    const int RHO_FIX_SIZE = 5;

    // forward kinematics 3x1
    Eigen::Vector3d fk(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // jacobian   3x3
    Eigen::Matrix3d jac(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // the partial derivative of fk wrt rho opt   3x3
    Eigen::Matrix3d dfk_drho(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // the partial derivative of jacobian wrt q    9x3
    Eigen::Matrix<double, 9, 3> dJ_dq(Eigen::Vector3d q, Eigen::VectorXd rho_opt, Eigen::VectorXd rho_fix);

    // the partial derivative of jacobian wrt rho opt   9x3
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
