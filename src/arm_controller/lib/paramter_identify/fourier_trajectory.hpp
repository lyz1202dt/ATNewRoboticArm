#pragma once

#include <Eigen/Core>

class FourierTrajectory {
public:
    FourierTrajectory(int dof, int harmonics, double period);

    int dof() const;
    int harmonics() const;

    double period() const;
    double omega() const;

    void set_init_pos(const Eigen::VectorXd& q0);

    const Eigen::VectorXd& init_pos() const;

    /**
     * 行数为关节数，列数为傅里叶系数[a1,b1,a2,b2,a3,b3,...]
     */
    void set_coefficients(const Eigen::MatrixXd& coefficients);
    void set_coefficients(const Eigen::VectorXd& coefficients);
    void set_coefficients(const double *x,int n);

    const Eigen::VectorXd& coefficients() const;

    Eigen::VectorXd position(double t) const;

    Eigen::VectorXd velocity(double t) const;

    Eigen::VectorXd acceleration(double t) const;

private:
    int dof_;
    int harmonics_;

    double period_;
    double omega_;

    Eigen::VectorXd q0_;

    Eigen::VectorXd coefficients_;

    Eigen::VectorXd coefficientA(int k) const;
    Eigen::VectorXd coefficientB(int k) const;
};