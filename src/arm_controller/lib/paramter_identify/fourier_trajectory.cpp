#include "fourier_trajectory.hpp"

#include <cmath>
#include <stdexcept>

FourierTrajectory::FourierTrajectory(int dof, int harmonics, double period)
    : dof_(dof)
    , harmonics_(harmonics)
    , period_(period)
    , omega_(2.0 * M_PI / period)
    , q0_(Eigen::VectorXd::Zero(dof))
    , coefficients_(Eigen::VectorXd::Zero(dof * 2 * harmonics)) {
}

int FourierTrajectory::dof() const {
    return dof_;
}

int FourierTrajectory::harmonics() const {
    return harmonics_;
}

double FourierTrajectory::period() const {
    return period_;
}

double FourierTrajectory::omega() const {
    return omega_;
}

void FourierTrajectory::set_init_pos(const Eigen::VectorXd& q0) {
    if (q0.size() != dof_) {
        throw std::invalid_argument("Invalid q0 dimension");
    }

    q0_ = q0;
}

const Eigen::VectorXd& FourierTrajectory::init_pos() const {
    return q0_;
}

void FourierTrajectory::set_coefficients(const Eigen::MatrixXd& coefficients) {
    if (coefficients.rows() != dof_ || coefficients.cols() != 2 * harmonics_) {

        throw std::invalid_argument("Invalid Fourier coefficient dimension");
    }

    coefficients_ = Eigen::Map<const Eigen::VectorXd>(coefficients.data(), coefficients.size());
}

void FourierTrajectory::set_coefficients(const Eigen::VectorXd& coefficients) {
    coefficients_ = coefficients;
}

void FourierTrajectory::set_coefficients(const double* x, int n) {
    for (int i = 0; i < n; i++)
        coefficients_(i) = x[i];
}

const Eigen::VectorXd& FourierTrajectory::coefficients() const {
    return coefficients_;
}


Eigen::VectorXd FourierTrajectory::coefficientA(int k) const {
    const int offset = (k - 1) * 2 * dof_;

    return coefficients_.segment(offset, dof_);
}

Eigen::VectorXd FourierTrajectory::coefficientB(int k) const {
    const int offset = (k - 1) * 2 * dof_ + dof_;

    return coefficients_.segment(offset, dof_);
}


Eigen::VectorXd FourierTrajectory::position(double t) const {
    Eigen::VectorXd q = q0_;

    for (int k = 1; k <= harmonics_; ++k) {

        const double phase = static_cast<double>(k) * omega_ * t;

        q += coefficientA(k) * std::sin(phase) + coefficientB(k) * std::cos(phase);
    }

    return q;
}

bool FourierTrajectory::position(double t, Eigen::VectorXd& q) const {
    if (q.size() != dof_) {
        return false;
    }

    q = q0_;
    for (int k = 1; k <= harmonics_; ++k) {
        const double phase = static_cast<double>(k) * omega_ * t;
        q.noalias() += coefficients_.segment((k - 1) * 2 * dof_, dof_) * std::sin(phase);
        q.noalias() += coefficients_.segment((k - 1) * 2 * dof_ + dof_, dof_) * std::cos(phase);
    }

    return true;
}


Eigen::VectorXd FourierTrajectory::velocity(double t) const {
    Eigen::VectorXd qdot = Eigen::VectorXd::Zero(dof_);

    for (int k = 1; k <= harmonics_; ++k) {

        const double kw = static_cast<double>(k) * omega_;

        const double phase = kw * t;

        qdot += coefficientA(k) * kw * std::cos(phase) - coefficientB(k) * kw * std::sin(phase);
    }

    return qdot;
}


Eigen::VectorXd FourierTrajectory::acceleration(double t) const {
    Eigen::VectorXd qddot = Eigen::VectorXd::Zero(dof_);

    for (int k = 1; k <= harmonics_; ++k) {

        const double kw = static_cast<double>(k) * omega_;

        const double phase = kw * t;

        qddot -= coefficientA(k) * kw * kw * std::sin(phase) + coefficientB(k) * kw * kw * std::cos(phase);
    }

    return qddot;
}
