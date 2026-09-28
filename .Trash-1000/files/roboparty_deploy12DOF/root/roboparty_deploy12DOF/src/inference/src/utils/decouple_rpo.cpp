// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi

#include "decouple_rpo.hpp"

#include <array>
#include <cmath>
#include <iostream>
#include <limits>
#include <stdexcept>

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kDeg2Rad = kPi / 180.0;

constexpr double kBarLength = 56.0;
constexpr std::array<double, 2> kRodLength = {253.0, 165.0};
constexpr double kSpacing = 31.75;
constexpr double kA1Z = 252.8;
constexpr double kA2Z = 164.7;
constexpr double kCX = -52.64;
constexpr double kCZ = 19.107;
constexpr std::array<double, 2> kRightLegMotorToInternalSign = {1.0, -1.0};

std::array<double, 2> motor_to_internal_sign(bool is_left)
{
    const double mirror = is_left ? -1.0 : 1.0;
    return {mirror * kRightLegMotorToInternalSign[0],
            mirror * kRightLegMotorToInternalSign[1]};
}

std::array<double, 2> motor_to_internal_sign_from_geometry(const std::array<Eigen::Vector3d, 2> &r_C)
{
    const bool inferred_left = r_C[0][1] < r_C[1][1];
    return motor_to_internal_sign(inferred_left);
}

double clamp_value(double value, double low, double high)
{
    if (value < low) return low;
    if (value > high) return high;
    return value;
}

double normalize_angle(double angle)
{
    return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Matrix3d rotation_y(double angle)
{
    Eigen::Matrix3d R_y = Eigen::Matrix3d::Zero();
    R_y << std::cos(angle), 0.0, std::sin(angle),
           0.0,             1.0, 0.0,
          -std::sin(angle), 0.0, std::cos(angle);
    return R_y;
}

Eigen::Vector3d zero_bar_vector()
{
    const double zero_global_angle = 164.0 * kDeg2Rad;
    return Eigen::Vector3d(kBarLength * std::cos(zero_global_angle),
                           0.0,
                           kBarLength * std::sin(zero_global_angle));
}

double solve_internal_beta(const Eigen::Vector3d &r_A,
                           const Eigen::Vector3d &r_C,
                           const Eigen::Vector3d &r_bar_zero,
                           double rod_length,
                           double preferred_beta)
{
    const Eigen::Vector3d p = r_C - r_A;
    const double px = p[0];
    const double pz = p[2];
    const double ux = r_bar_zero[0];
    const double uz = r_bar_zero[2];
    const double rhs = (p.squaredNorm() + r_bar_zero.squaredNorm() - rod_length * rod_length) / 2.0;
    const double Kc = px * ux + pz * uz;
    const double Ks = px * uz - pz * ux;
    const double amplitude = std::hypot(Kc, Ks);

    if (amplitude < 1e-12) {
        std::cerr << "Warning: Degenerate geometry in solve_internal_beta(). Use preferred beta." << std::endl;
        return preferred_beta;
    }

    double ratio = rhs / amplitude;
    if (ratio < -1.0 || ratio > 1.0) {
        std::cerr << "Warning: Unreachable geometry in inverse kinematics. Clamping acos argument from "
                  << ratio << " to [-1, 1]." << std::endl;
    }
    ratio = clamp_value(ratio, -1.0, 1.0);

    const double phi = std::atan2(Ks, Kc);
    const double gamma = std::acos(ratio);
    const double candidate_1 = normalize_angle(phi + gamma);
    const double candidate_2 = normalize_angle(phi - gamma);
    const double error_1 = std::abs(normalize_angle(candidate_1 - preferred_beta));
    const double error_2 = std::abs(normalize_angle(candidate_2 - preferred_beta));
    return (error_1 <= error_2) ? candidate_1 : candidate_2;
}

bool has_invalid_number(const Eigen::Matrix2d &matrix)
{
    for (int r = 0; r < matrix.rows(); ++r) {
        for (int c = 0; c < matrix.cols(); ++c) {
            if (!std::isfinite(matrix(r, c))) return true;
        }
    }
    return false;
}
} // namespace

std::vector<LinkParamsRPO> DecoupleRPO::get_links(bool is_left)
{
    (void)is_left;
    return {};
}

DecoupleRPO::DecoupleRPO()
    : links_left_(get_links(true)), links_right_(get_links(false))
{
}

IKResultRPO DecoupleRPO::inverse_kinematics(double q_roll, double q_pitch, bool is_left)
{
    IKResultRPO result;
    result.THETA = Eigen::Vector2d::Zero();

    const double l_spacing = is_left ? kSpacing : -kSpacing;
    const std::array<double, 2> motor_sign = motor_to_internal_sign(is_left);
    const Eigen::Vector3d r_bar_zero = zero_bar_vector();

    std::array<Eigen::Vector3d, 2> r_A_0 = {
        Eigen::Vector3d(10.0, -l_spacing, kA1Z),
        Eigen::Vector3d(10.0,  l_spacing, kA2Z)
    };
    std::array<Eigen::Vector3d, 2> r_C_0 = {
        Eigen::Vector3d(kCX, -l_spacing, kCZ),
        Eigen::Vector3d(kCX,  l_spacing, kCZ)
    };

    const std::array<double, 2> beta_zero_bias = {
        solve_internal_beta(r_A_0[0], r_C_0[0], r_bar_zero, kRodLength[0], 0.0),
        solve_internal_beta(r_A_0[1], r_C_0[1], r_bar_zero, kRodLength[1], 0.0)
    };

    Eigen::Matrix3d R_y = Eigen::Matrix3d::Zero();
    R_y << std::cos(q_pitch), 0.0, std::sin(q_pitch),
           0.0,               1.0, 0.0,
          -std::sin(q_pitch), 0.0, std::cos(q_pitch);

    Eigen::Matrix3d R_x = Eigen::Matrix3d::Zero();
    R_x << 1.0, 0.0,              0.0,
           0.0, std::cos(q_roll), -std::sin(q_roll),
           0.0, std::sin(q_roll),  std::cos(q_roll);

    const Eigen::Matrix3d x_rot = R_y * R_x;

    for (int i = 0; i < 2; ++i) {
        const Eigen::Vector3d r_A_i = r_A_0[i];
        const Eigen::Vector3d r_C_i = x_rot * r_C_0[i];
        const double beta_i = solve_internal_beta(r_A_i,
                                                  r_C_i,
                                                  r_bar_zero,
                                                  kRodLength[i],
                                                  beta_zero_bias[i]);
        const Eigen::Vector3d r_B_i = r_A_i + rotation_y(beta_i) * r_bar_zero;
        result.r_bar[i] = r_B_i - r_A_i;
        result.r_rod[i] = r_C_i - r_B_i;
        result.r_C[i] = r_C_i;
        result.THETA[i] = normalize_angle(beta_i - beta_zero_bias[i]) / motor_sign[i];
    }

    return result;
}

JacobianResult DecoupleRPO::jacobian(const IKResultRPO &ik_result, double q_pitch)
{
    const auto &r_C = ik_result.r_C;
    const auto &r_bar = ik_result.r_bar;
    const auto &r_rod = ik_result.r_rod;
    static const Eigen::Vector3d s_unit(0.0, 1.0, 0.0);
    const std::array<double, 2> motor_sign = motor_to_internal_sign_from_geometry(r_C);

    Eigen::Matrix<double, 2, 6> J_x;
    J_x << r_rod[0].transpose(), (r_C[0].cross(r_rod[0])).transpose(),
           r_rod[1].transpose(), (r_C[1].cross(r_rod[1])).transpose();

    Eigen::Matrix2d J_theta;
    J_theta << motor_sign[0] * s_unit.dot(r_bar[0].cross(r_rod[0])), 0.0,
               0.0, motor_sign[1] * s_unit.dot(r_bar[1].cross(r_rod[1]));

    Eigen::Matrix<double, 6, 2> J_q;
    J_q << 0.0, 0.0,
           0.0, 0.0,
           0.0, 0.0,
           0.0, std::cos(q_pitch),
           1.0, 0.0,
           0.0, -std::sin(q_pitch);

    const Eigen::Matrix2d J_temp = J_x * J_q;
    Eigen::PartialPivLU<Eigen::Matrix2d> lu_decomp(J_temp);
    Eigen::PartialPivLU<Eigen::Matrix2d> lu_theta(J_theta);

    JacobianResult result;
    result.J_motor2Joint = lu_decomp.solve(J_theta);
    result.J_Joint2motor = lu_theta.solve(J_temp);
    return result;
}

std::pair<Eigen::Vector2d, JacobianResult> DecoupleRPO::get_decouple(double roll, double pitch, bool is_left)
{
    IKResultRPO kinematics = inverse_kinematics(roll, pitch, is_left);
    JacobianResult Jac = jacobian(kinematics, pitch);
    return {kinematics.THETA, Jac};
}

ForwardMappingResult DecoupleRPO::forward_kinematics(const Eigen::Vector2d &thetaRef, bool is_left)
{
    ForwardMappingResult mapping_result;
    int count = 0;
    Eigen::Vector2d f_error(10.0, 10.0);
    Eigen::Vector2d x_k = last_solution_.count(is_left) ? last_solution_[is_left] : Eigen::Vector2d::Zero();
    JacobianResult Jac;
    static constexpr int MAX_ITERATIONS = 100;
    static constexpr double TOLERANCE = 1e-3;
    static constexpr double ALPHA = 0.5;

    while (f_error.norm() > TOLERANCE && count < MAX_ITERATIONS) {
        IKResultRPO kinematics = inverse_kinematics(x_k[1], x_k[0], is_left);
        Jac = jacobian(kinematics, x_k[0]);

        if (has_invalid_number(Jac.J_motor2Joint)) {
            std::cerr << "DecoupleRPO::forward_kinematics() Jac is invalid." << std::endl;
            std::cerr << "  pitch=" << x_k[0] << ", roll=" << x_k[1] << std::endl;
            mapping_result.count = -1;
            mapping_result.ankle_joint_ori = Eigen::Vector2d::Zero();
            mapping_result.Jac = Jac;
            return mapping_result;
        }

        f_error = thetaRef - kinematics.THETA;
        x_k = x_k + ALPHA * Jac.J_motor2Joint * f_error;
        ++count;
    }

    if (f_error.norm() < TOLERANCE) {
        last_solution_[is_left] = x_k;
    } else {
        std::cerr << "Warning: DecoupleRPO::forward_kinematics() did not converge. count="
                  << count << ", error_norm=" << f_error.norm() << std::endl;
        mapping_result.count = -1;
        mapping_result.ankle_joint_ori = Eigen::Vector2d::Zero();
        mapping_result.Jac = Jac;
        return mapping_result;
    }

    mapping_result.count = count;
    mapping_result.ankle_joint_ori = x_k;
    mapping_result.Jac = Jac;
    return mapping_result;
}

void DecoupleRPO::get_decoupleQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel, Eigen::VectorXd &tau, bool is_left)
{
    const double pitch = q[0];
    const double roll = q[1];
    auto motor = get_decouple(roll, pitch, is_left);
    q.segment<2>(0) = motor.first;
    vel.segment<2>(0) = motor.second.J_Joint2motor * vel.segment<2>(0);
    tau.segment<2>(0) = motor.second.J_motor2Joint.transpose() * tau.segment<2>(0);
}

void DecoupleRPO::get_forwardQVT(Eigen::VectorXd &q, Eigen::VectorXd &vel, Eigen::VectorXd &tau, bool is_left)
{
    ForwardMappingResult joint = forward_kinematics(q.segment<2>(0), is_left);
    q.segment<2>(0) = joint.ankle_joint_ori;

    if (joint.count < 0) {
        throw std::runtime_error("DecoupleRPO::get_forwardQVT() failed: forward kinematics did not converge");
    }

    vel.segment<2>(0) = joint.Jac.J_motor2Joint * vel.segment<2>(0);
    tau.segment<2>(0) = joint.Jac.J_Joint2motor.transpose() * tau.segment<2>(0);
}
