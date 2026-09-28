// SPDX-License-Identifier: GPL-3.0
// Copyright (C) 2025-2026 Luo1imasi

#include "robot_interface.hpp"
#include <cmath>

namespace {
bool is_reasonable_reset_value(float value, float limit) {
    return std::isfinite(value) && std::abs(value) <= limit;
}

float wrap_to_pi_local(float x) {
    if (!std::isfinite(x)) return 0.0f;
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    float wrapped = std::remainder(x, two_pi);
    if (wrapped > static_cast<float>(M_PI)) wrapped -= two_pi;
    if (wrapped < -static_cast<float>(M_PI)) wrapped += two_pi;
    return wrapped;
}

float wrap_to_nearest_equivalent(float current, float target) {
    return current + wrap_to_pi_local(target - current);
}

float wrap_around_target(float current, float target) {
    return target + wrap_to_pi_local(current - target);
}

float nearest_equivalent_target(float current, float target) {
    return current + wrap_to_pi_local(target - current);
}

}

RobotInterface::RobotInterface(const std::string& config_file) {
    YAML::Node config = YAML::LoadFile(config_file);

    imu_cfg_ = std::make_shared<IMUCfg>();
    if (config["imu"]) {
        YAML::Node imu_node = config["imu"];
        if (imu_node["imu_id"]) imu_cfg_->imu_id_ = imu_node["imu_id"].as<int>();
        if (imu_node["baudrate"]) imu_cfg_->baudrate_ = imu_node["baudrate"].as<int>();
        if (imu_node["imu_type"]) imu_cfg_->imu_type_ = imu_node["imu_type"].as<std::string>();
        if (imu_node["imu_interface_type"]) imu_cfg_->imu_interface_type_ = imu_node["imu_interface_type"].as<std::string>();
        if (imu_node["imu_interface"]) imu_cfg_->imu_interface_ = imu_node["imu_interface"].as<std::string>();
        if (imu_node["debug_fake_imu"]) debug_fake_imu_ = imu_node["debug_fake_imu"].as<bool>();
        if (!debug_fake_imu_) {
            setup_imu();
        }
    }

    motors_cfg_ = std::make_shared<MotorsCfg>();
    if (config["motors"]) {
        YAML::Node motors_node = config["motors"];
        if (motors_node["motor_zero_offset"]) motors_cfg_->motor_zero_offset_ = motors_node["motor_zero_offset"].as<std::vector<double>>();
        if (motors_node["master_id_offset"]) motors_cfg_->master_id_offset_ = motors_node["master_id_offset"].as<int>();
        if (motors_node["motor_type"]) motors_cfg_->motor_type_ = motors_node["motor_type"].as<std::vector<std::string>>();
        if (motors_node["motor_interface_type"]) motors_cfg_->motor_interface_type_ = motors_node["motor_interface_type"].as<std::vector<std::string>>();
        if (motors_node["motor_interface"]) motors_cfg_->motor_interface_ = motors_node["motor_interface"].as<std::vector<std::string>>();
        if (motors_node["motor_id"]) motors_cfg_->motor_id_ = motors_node["motor_id"].as<std::vector<long int>>();
        if (motors_node["motor_model"]) motors_cfg_->motor_model_ = motors_node["motor_model"].as<std::vector<long int>>();
        if (motors_node["motor_num"]) motors_cfg_->motor_num_ = motors_node["motor_num"].as<std::vector<long int>>();
        setup_motors();
    } else {
        throw std::runtime_error("Motors configuration not found in " + config_file);
    }

    robot_cfg_ = std::make_shared<RobotCfg>();
    if (config["robot"]) {
        YAML::Node robot_node = config["robot"];
        if (robot_node["kp"]) robot_cfg_->kp_ = robot_node["kp"].as<std::vector<double>>();
        if (robot_node["kd"]) robot_cfg_->kd_ = robot_node["kd"].as<std::vector<double>>();
        if (robot_node["close_chain_motor_idx"]) robot_cfg_->close_chain_motor_idx_ = robot_node["close_chain_motor_idx"].as<std::vector<long int>>();
        if (robot_node["motor_sign"]) robot_cfg_->motor_sign_ = robot_node["motor_sign"].as<std::vector<long int>>();
        if (robot_node["urdf2motor"]) robot_cfg_->urdf2motor_ = robot_node["urdf2motor"].as<std::vector<long int>>();
        motor2urdf_ = std::vector<int>(motors_cfg_->motor_id_.size(), -1);
        for (size_t i = 0; i < robot_cfg_->urdf2motor_.size(); ++i) {
            motor2urdf_[robot_cfg_->urdf2motor_[i] - 1] = i;
        }
        if (robot_node["extrinsic_R"]) {
            robot_cfg_->extrinsic_R_ = robot_node["extrinsic_R"].as<std::vector<double>>();
            if (robot_cfg_->extrinsic_R_.size() == 9) {
                // Row-major: [r00, r01, r02, r10, r11, r12, r20, r21, r22]
                extrinsic_R_mat_ << robot_cfg_->extrinsic_R_[0], robot_cfg_->extrinsic_R_[1], robot_cfg_->extrinsic_R_[2],
                                    robot_cfg_->extrinsic_R_[3], robot_cfg_->extrinsic_R_[4], robot_cfg_->extrinsic_R_[5],
                                    robot_cfg_->extrinsic_R_[6], robot_cfg_->extrinsic_R_[7], robot_cfg_->extrinsic_R_[8];
                Eigen::Quaternionf q_R(extrinsic_R_mat_);  // quaternion of R (Body->IMU)
                extrinsic_q_inv_ = q_R.inverse();           // we need R_inv for quaternion transform
            }
        }
        for (auto idx : robot_cfg_->close_chain_motor_idx_) {
            auto it = std::find(robot_cfg_->urdf2motor_.begin(), robot_cfg_->urdf2motor_.end(), idx);
            if (it != robot_cfg_->urdf2motor_.end()) {
                close_chain_joint_idx_.push_back(std::distance(robot_cfg_->urdf2motor_.begin(), it));
            }
        }
        cached_ankle_action_.resize(close_chain_joint_idx_.size(), 0.0f);
        last_ankle_joint_target_.resize(close_chain_joint_idx_.size(), 0.0f);
        if (robot_node["type"]) {
            ankle_decouple_ = Decouple::create(robot_node["type"].as<std::string>());
        } else {
            ankle_decouple_ = nullptr;
        }
    } else {
        throw std::runtime_error("Robot configuration not found in " + config_file);
    }

    thread_pool_ = std::make_unique<ThreadPool>(motors_cfg_->motor_interface_.size());

    joint_q_ = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    joint_vel_ = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    joint_tau_ = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    motor_pos_target_ = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    motor_vel_target_ = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    motor_kp_target_  = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    motor_kd_target_  = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
    motor_tau_target_ = std::vector<float>(motors_cfg_->motor_id_.size(), 0.0);
}

void RobotInterface::setup_motors(){
    size_t count = 0;
    motors_.resize(motors_cfg_->motor_id_.size());
    motor_is_encos_.assign(motors_cfg_->motor_id_.size(), false);
    for (size_t i = 0; i < motors_cfg_->motor_interface_.size(); ++i){
        for (size_t j = 0; j < motors_cfg_->motor_num_[i]; ++j){
            motors_[count] = MotorDriver::create_motor(motors_cfg_->motor_id_[count], motors_cfg_->motor_interface_type_[i], motors_cfg_->motor_interface_[i], motors_cfg_->motor_type_[i], motors_cfg_->motor_model_[count], motors_cfg_->master_id_offset_, motors_cfg_->motor_zero_offset_[count]);
            motor_is_encos_[count] = motors_cfg_->motor_type_[i] == "ENCOS";
            count += 1;
        }
    }
}

void RobotInterface::setup_imu(){
    imu_ = IMUDriver::create_imu(imu_cfg_->imu_id_, imu_cfg_->imu_interface_type_, imu_cfg_->imu_interface_, imu_cfg_->imu_type_, imu_cfg_->baudrate_);
}

void RobotInterface::forward_close_chain() {
    Eigen::VectorXd q(2), vel(2), tau(2);
    for (size_t pair = 0; pair < 2; ++pair) {
        const bool left = (pair == 0);
        int idx1 = close_chain_joint_idx_[pair * 2];
        int idx2 = close_chain_joint_idx_[pair * 2 + 1];
        q << joint_q_[idx1], joint_q_[idx2];
        vel << joint_vel_[idx1], joint_vel_[idx2];
        tau << joint_tau_[idx1], joint_tau_[idx2];
        ankle_decouple_->get_forwardQVT(q, vel, tau, left);
        joint_q_[idx1]   = q[0];
        joint_q_[idx2]   = q[1];
        joint_vel_[idx1] = vel[0];
        joint_vel_[idx2] = vel[1];
        joint_tau_[idx1] = tau[0];
        joint_tau_[idx2] = tau[1];
    }
}

void RobotInterface::apply_action(std::vector<float> p,
                                  std::vector<float> v,
                                  std::vector<float> kp,
                                  std::vector<float> kd,
                                  std::vector<float> tau) {
    stop_hold_position();
    if(!is_init_.load()){
        return;
    }

    {
        std::unique_lock<std::mutex> lock(joint_mutex_);
        exec_motors_parallel([this](std::shared_ptr<MotorDriver>& motor, int idx) {
            const float raw_pos = motor->get_motor_pos() * robot_cfg_->motor_sign_[idx];
            joint_q_[motor2urdf_[idx]] = motor_is_encos_[idx] ? wrap_to_pi_local(raw_pos) : raw_pos;
            joint_vel_[motor2urdf_[idx]] = motor->get_motor_spd() * robot_cfg_->motor_sign_[idx];
            joint_tau_[motor2urdf_[idx]] = motor->get_motor_current() * robot_cfg_->motor_sign_[idx];
            if (motor->get_response_count() > offline_threshold_) {
                throw std::runtime_error("Motor id " + std::to_string(motors_cfg_->motor_id_[idx]) + " offline");
            }
        });

        if (!close_chain_joint_idx_.empty() && ankle_decouple_){
            auto kp_cc = [&](size_t i) -> double {
                return kp.empty() ? robot_cfg_->kp_[close_chain_joint_idx_[i]]
                                  : static_cast<double>(kp[close_chain_joint_idx_[i]]);
            };
            auto kd_cc = [&](size_t i) -> double {
                return kd.empty() ? robot_cfg_->kd_[close_chain_joint_idx_[i]]
                                  : static_cast<double>(kd[close_chain_joint_idx_[i]]);
            };

            bool refresh = false;
            for (size_t i = 0; i < close_chain_joint_idx_.size(); ++i) {
                if (p[close_chain_joint_idx_[i]] != last_ankle_joint_target_[i]) {
                    refresh = true;
                }
                last_ankle_joint_target_[i] = p[close_chain_joint_idx_[i]];
            }
            forward_close_chain();

            Eigen::VectorXd q(2), vel(2), tau_cc(2);
            for (size_t pair = 0; pair < 2; ++pair) {
                const bool left = (pair == 0);
                const size_t off = pair * 2;
                int idx1 = close_chain_joint_idx_[off];
                int idx2 = close_chain_joint_idx_[off + 1];
                q << joint_q_[idx1], joint_q_[idx2];
                vel << joint_vel_[idx1], joint_vel_[idx2];
                tau_cc << joint_tau_[idx1], joint_tau_[idx2];
                if (refresh) {
                    tau_cc << kp_cc(off)     * (p[idx1] - q[0]) + kd_cc(off)     * (0.0 - vel[0]),
                             kp_cc(off + 1) * (p[idx2] - q[1]) + kd_cc(off + 1) * (0.0 - vel[1]);
                    ankle_decouple_->get_decoupleQVT(q, vel, tau_cc, left);
                    p[idx1] = static_cast<float>(q[0] + (tau_cc[0] + kd_cc(off)     * vel[0]) / kp_cc(off));
                    p[idx2] = static_cast<float>(q[1] + (tau_cc[1] + kd_cc(off + 1) * vel[1]) / kp_cc(off + 1));
                    cached_ankle_action_[off]     = p[idx1];
                    cached_ankle_action_[off + 1] = p[idx2];
                } else {
                    p[idx1] = cached_ankle_action_[off];
                    p[idx2] = cached_ankle_action_[off + 1];
                }
            }
        }
    }

    {
        std::unique_lock<std::mutex> lock(motors_mutex_);
        for (size_t i = 0; i < motor_pos_target_.size(); i++){
            const size_t ji = motor2urdf_[i];
            motor_pos_target_[i] = p[ji];
            motor_vel_target_[i] = v.empty()  ? 0.0f : v[ji];
            motor_kp_target_[i]  = kp.empty() ? static_cast<float>(robot_cfg_->kp_[i]) : kp[ji];
            motor_kd_target_[i]  = kd.empty() ? static_cast<float>(robot_cfg_->kd_[i]) : kd[ji];
            motor_tau_target_[i] = tau.empty() ? 0.0f : tau[ji];
        }
    }

    motors_mit_cmd();
}

void RobotInterface::reset_joints(std::vector<double> joint_default_angle) {
    stop_hold_position();
    if (!is_init_.load()) {
        throw std::runtime_error("Motors are not initialized");
    }
    if (joint_default_angle.empty()) {
        throw std::runtime_error("joint_default_angle is empty");
    }
    std::vector<double> joint_default_angle_ori = joint_default_angle;
    if (!close_chain_joint_idx_.empty() && ankle_decouple_){
        Eigen::VectorXd q(2), vel(2), tau(2);
        vel.setZero();
        tau.setZero();
        for (size_t pair = 0; pair < 2; ++pair) {
            const bool left = (pair == 0);
            int idx1 = close_chain_joint_idx_[pair * 2];
            int idx2 = close_chain_joint_idx_[pair * 2 + 1];
            q << joint_default_angle[idx1], joint_default_angle[idx2];
            ankle_decouple_->get_decoupleQVT(q, vel, tau, left);
            joint_default_angle[idx1] = q[0];
            joint_default_angle[idx2] = q[1];
        }
    }

    for (int refresh_try = 0; refresh_try < 3; ++refresh_try) {
        refresh_joints();
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    std::vector<float> q_start = get_joint_q();
    if (q_start.size() != joint_default_angle.size()) {
        throw std::runtime_error("joint_default_angle size does not match joint size");
    }
    std::vector<float> q_start_ori = q_start;
    for (size_t i = 0; i < close_chain_joint_idx_.size(); ++i) {
        const int ji = close_chain_joint_idx_[i];
        const bool bad_index = ji < 0 || static_cast<size_t>(ji) >= q_start_ori.size();
        const float value = bad_index ? 9999.0f : q_start_ori[ji];
        if (bad_index || !is_reasonable_reset_value(value, 1.5f)) {
            std::cerr << "[reset_joints] close-chain reset rejected: invalid start joint_idx=" << ji
                      << " q_start=" << value << std::endl;
            throw std::runtime_error("close-chain start feedback invalid; reset rejected");
        }
    }
    std::vector<float> q_reset_start = q_start;
    std::vector<float> q_reset_target(joint_default_angle.size(), 0.0f);
    std::vector<bool> soft_reset_kp(motor_pos_target_.size(), false);
    std::vector<bool> close_chain_joint_mask(joint_default_angle.size(), false);
    std::vector<bool> close_chain_motor_mask(motor_pos_target_.size(), false);
    for (size_t i = 0; i < close_chain_joint_idx_.size(); ++i) {
        const int ji = close_chain_joint_idx_[i];
        if (ji >= 0 && static_cast<size_t>(ji) < close_chain_joint_mask.size()) {
            close_chain_joint_mask[ji] = true;
        }
    }
    for (auto motor_id : robot_cfg_->close_chain_motor_idx_) {
        if (motor_id > 0 && static_cast<size_t>(motor_id - 1) < close_chain_motor_mask.size()) {
            close_chain_motor_mask[static_cast<size_t>(motor_id - 1)] = true;
        }
    }
    for (size_t ji = 0; ji < q_reset_target.size(); ++ji) {
        q_reset_target[ji] = static_cast<float>(joint_default_angle[ji]);
    }
    size_t motor_idx = 0;
    for (size_t bus = 0; bus < motors_cfg_->motor_num_.size(); ++bus) {
        const bool is_encos_bus = bus < motors_cfg_->motor_type_.size() && motors_cfg_->motor_type_[bus] == "ENCOS";
        for (long int j = 0; j < motors_cfg_->motor_num_[bus] && motor_idx < motor_pos_target_.size(); ++j, ++motor_idx) {
            if (!is_encos_bus) continue;
            if (close_chain_motor_mask[motor_idx]) {
                soft_reset_kp[motor_idx] = true;
                continue;
            }
            const size_t ji = motor2urdf_[motor_idx];
            const float target = static_cast<float>(joint_default_angle[ji]);
            const float raw_start = motors_[motor_idx]->get_motor_pos() * static_cast<float>(robot_cfg_->motor_sign_[motor_idx]);
            const float raw_wrapped = wrap_to_pi_local(raw_start);
            if (!std::isfinite(raw_start)) {
                std::cerr << "[reset_joints] ENCOS reset rejected motor_idx=" << motor_idx
                          << " joint_idx=" << ji
                          << " raw_start=" << raw_start
                          << " target=" << target
                          << std::endl;
                throw std::runtime_error("ENCOS raw position invalid; reset rejected");
            }
            q_reset_start[ji] = raw_wrapped;
            q_reset_target[ji] = nearest_equivalent_target(raw_wrapped, target);
            soft_reset_kp[motor_idx] = true;
            std::cerr << "[reset_joints] ENCOS motor_idx=" << motor_idx
                      << " joint_idx=" << ji
                      << " raw_start=" << raw_start
                      << " raw_wrapped=" << raw_wrapped
                      << " q_start=" << q_start[ji]
                      << " target=" << target
                      << " q_reset_start=" << q_reset_start[ji]
                      << " q_reset_target=" << q_reset_target[ji]
                      << std::endl;
        }
    }

    const double dt = 0.10;      // Keep ENCOS reset conservative on gs_usb/CANable adapters.
    const double max_reset_speed = 0.025;  // rad/s, about 1.4 deg/s.
    const double min_duration_sec = 5.0;
    const double max_duration_sec = 120.0;
    const float reset_kp_start_scale = 0.05f;
    const float reset_kp_end_scale = 0.35f;

    double max_duration = min_duration_sec;
    std::vector<double> reset_duration(motor_pos_target_.size(), min_duration_sec);
    std::vector<double> reset_delta(motor_pos_target_.size(), 0.0);
    for (size_t i = 0; i < motor_pos_target_.size(); ++i) {
        const size_t ji = motor2urdf_[i];
        if (close_chain_motor_mask[i]) {
            reset_delta[i] = std::abs(static_cast<double>(joint_default_angle_ori[ji] - q_start_ori[ji]));
        } else {
            reset_delta[i] = std::abs(static_cast<double>(q_reset_target[ji] - q_reset_start[ji]));
        }
        reset_duration[i] = std::clamp(reset_delta[i] / max_reset_speed, min_duration_sec, max_duration_sec);
        max_duration = std::max(max_duration, reset_duration[i]);
        if (soft_reset_kp[i]) {
            std::cerr << "[reset_joints] motor_idx=" << i
                      << " delta=" << reset_delta[i]
                      << " duration=" << reset_duration[i]
                      << std::endl;
        }
    }
    std::cerr << "[reset_joints] max_duration=" << max_duration
              << " max_speed=" << max_reset_speed
              << std::endl;

    if (false) {
    std::cerr << "[reset_joints][diagnostic] BEGIN per-motor reset plan" << std::endl;
    for (size_t i = 0; i < motor_pos_target_.size(); ++i) {
        const size_t ji = motor2urdf_[i];
        const bool is_close_chain_motor = close_chain_motor_mask[i];
        const float driver_pos = motors_[i]->get_motor_pos();
        const float signed_pos = driver_pos * static_cast<float>(robot_cfg_->motor_sign_[i]);
        const float signed_wrapped = wrap_to_pi_local(signed_pos);
        std::string iface = "unknown";
        int motor_id = -1;
        size_t accum = 0;
        for (size_t bus = 0; bus < motors_cfg_->motor_num_.size(); ++bus) {
            const size_t count = static_cast<size_t>(motors_cfg_->motor_num_[bus]);
            if (i >= accum && i < accum + count) {
                if (bus < motors_cfg_->motor_interface_.size()) iface = motors_cfg_->motor_interface_[bus];
                if (i < motors_cfg_->motor_id_.size()) motor_id = motors_cfg_->motor_id_[i];
                break;
            }
            accum += count;
        }
        std::cerr << "[reset_joints][diagnostic] motor_idx=" << i
                  << " motor_id=" << motor_id
                  << " iface=" << iface
                  << " joint_idx=" << ji
                  << " urdf2motor=" << ((ji < robot_cfg_->urdf2motor_.size()) ? robot_cfg_->urdf2motor_[ji] : -1)
                  << " motor2urdf=" << ji
                  << " close_chain=" << (is_close_chain_motor ? 1 : 0)
                  << " sign=" << robot_cfg_->motor_sign_[i]
                  << " zero_offset=" << ((i < motors_cfg_->motor_zero_offset_.size()) ? motors_cfg_->motor_zero_offset_[i] : 0.0)
                  << " kp=" << robot_cfg_->kp_[i]
                  << " kd=" << robot_cfg_->kd_[i]
                  << " driver_pos=" << driver_pos
                  << " signed_pos=" << signed_pos
                  << " signed_wrapped=" << signed_wrapped
                  << " q_start=" << q_start[ji]
                  << " q_start_ori=" << q_start_ori[ji]
                  << " joint_default_motor_space=" << joint_default_angle[ji]
                  << " joint_default_ori=" << joint_default_angle_ori[ji]
                  << " q_reset_start=" << q_reset_start[ji]
                  << " q_reset_target=" << q_reset_target[ji]
                  << " delta=" << reset_delta[i]
                  << " duration=" << reset_duration[i]
                  << std::endl;
    }
    if (!close_chain_joint_idx_.empty() && ankle_decouple_) {
        for (size_t pair = 0; pair < 2 && pair * 2 + 1 < close_chain_joint_idx_.size(); ++pair) {
            const bool left = (pair == 0);
            const int idx1 = close_chain_joint_idx_[pair * 2];
            const int idx2 = close_chain_joint_idx_[pair * 2 + 1];
            Eigen::VectorXd q0(2), qT(2), vel(2), tau(2);
            vel.setZero();
            tau.setZero();
            q0 << q_start_ori[idx1], q_start_ori[idx2];
            qT << joint_default_angle_ori[idx1], joint_default_angle_ori[idx2];
            Eigen::VectorXd q0_motor = q0;
            Eigen::VectorXd qT_motor = qT;
            ankle_decouple_->get_decoupleQVT(q0_motor, vel, tau, left);
            ankle_decouple_->get_decoupleQVT(qT_motor, vel, tau, left);
            std::cerr << "[reset_joints][diagnostic] close_chain_pair=" << pair
                      << " left=" << (left ? 1 : 0)
                      << " joint_idx=(" << idx1 << "," << idx2 << ")"
                      << " joint_start=(" << q0[0] << "," << q0[1] << ")"
                      << " joint_target=(" << qT[0] << "," << qT[1] << ")"
                      << " motor_start_ik=(" << q0_motor[0] << "," << q0_motor[1] << ")"
                      << " motor_target_ik=(" << qT_motor[0] << "," << qT_motor[1] << ")"
                      << std::endl;
        }
    }
    std::cerr << "[reset_joints][diagnostic] END per-motor reset plan" << std::endl;
    }

    const int steps = static_cast<int>(max_duration / dt);

    for (int k = 1; k <= steps; ++k) {
        const double elapsed = static_cast<double>(k) * dt;
        std::vector<float> joint_cmd = q_reset_start;
        for (size_t ji = 0; ji < joint_cmd.size(); ++ji) {
            if (close_chain_joint_mask[ji]) continue;
            const size_t mi = robot_cfg_->urdf2motor_[ji] - 1;
            const double a = std::clamp(elapsed / reset_duration[mi], 0.0, 1.0);
            const double smooth = a * a * a * (10.0 + a * (-15.0 + 6.0 * a));
            joint_cmd[ji] = static_cast<float>(q_reset_start[ji] + smooth * (q_reset_target[ji] - q_reset_start[ji]));
        }
        for (size_t pair = 0; pair < 2 && pair * 2 + 1 < close_chain_joint_idx_.size(); ++pair) {
            const bool left = (pair == 0);
            const int idx1 = close_chain_joint_idx_[pair * 2];
            const int idx2 = close_chain_joint_idx_[pair * 2 + 1];
            const size_t mi1 = robot_cfg_->urdf2motor_[idx1] - 1;
            const size_t mi2 = robot_cfg_->urdf2motor_[idx2] - 1;
            const double pair_duration = std::max(reset_duration[mi1], reset_duration[mi2]);
            const double a = std::clamp(elapsed / pair_duration, 0.0, 1.0);
            const double smooth = a * a * a * (10.0 + a * (-15.0 + 6.0 * a));
            Eigen::VectorXd q(2), vel(2), tau(2);
            q << q_start_ori[idx1] + smooth * (joint_default_angle_ori[idx1] - q_start_ori[idx1]),
                 q_start_ori[idx2] + smooth * (joint_default_angle_ori[idx2] - q_start_ori[idx2]);
            vel.setZero();
            tau.setZero();
            ankle_decouple_->get_decoupleQVT(q, vel, tau, left);
            if (!is_reasonable_reset_value(static_cast<float>(q[0]), 2.0f) ||
                !is_reasonable_reset_value(static_cast<float>(q[1]), 2.0f)) {
                std::cerr << "[reset_joints] close-chain reset rejected: IK target invalid pair=" << pair
                          << " motor_theta=(" << q[0] << ", " << q[1] << ")" << std::endl;
                throw std::runtime_error("close-chain IK target invalid; reset rejected");
            }
            joint_cmd[idx1] = static_cast<float>(q[0]);
            joint_cmd[idx2] = static_cast<float>(q[1]);
        }
        {
            std::unique_lock<std::mutex> lock(motors_mutex_);
            for (size_t i = 0; i < motor_pos_target_.size(); ++i) {
                const size_t ji = motor2urdf_[i];
                motor_pos_target_[i] = joint_cmd[ji];
                motor_vel_target_[i] = 0.0f;
                const float kp = static_cast<float>(robot_cfg_->kp_[i]);
                const double kp_a = std::clamp(elapsed / reset_duration[i], 0.0, 1.0);
                const double kp_smooth = kp_a * kp_a * kp_a * (10.0 + kp_a * (-15.0 + 6.0 * kp_a));
                if (soft_reset_kp[i] && kp_a < 1.0) {
                    const float kp_scale = reset_kp_start_scale +
                        static_cast<float>(kp_smooth) * (reset_kp_end_scale - reset_kp_start_scale);
                    motor_kp_target_[i] = kp * kp_scale;
                } else {
                    motor_kp_target_[i] = kp;
                }
                motor_kd_target_[i] = static_cast<float>(robot_cfg_->kd_[i]);
                motor_tau_target_[i] = 0.0f;
            }
        }
        motors_mit_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    {
        std::unique_lock<std::mutex> lock(motors_mutex_);
        std::vector<float> final_cmd(q_reset_target.begin(), q_reset_target.end());
        for (size_t pair = 0; pair < 2 && pair * 2 + 1 < close_chain_joint_idx_.size(); ++pair) {
            const bool left = (pair == 0);
            const int idx1 = close_chain_joint_idx_[pair * 2];
            const int idx2 = close_chain_joint_idx_[pair * 2 + 1];
            Eigen::VectorXd q(2), vel(2), tau(2);
            q << joint_default_angle_ori[idx1], joint_default_angle_ori[idx2];
            vel.setZero();
            tau.setZero();
            ankle_decouple_->get_decoupleQVT(q, vel, tau, left);
            if (!is_reasonable_reset_value(static_cast<float>(q[0]), 2.0f) ||
                !is_reasonable_reset_value(static_cast<float>(q[1]), 2.0f)) {
                std::cerr << "[reset_joints] close-chain final target rejected: pair=" << pair
                          << " motor_theta=(" << q[0] << ", " << q[1] << ")" << std::endl;
                throw std::runtime_error("close-chain final IK target invalid; reset rejected");
            }
            final_cmd[idx1] = static_cast<float>(q[0]);
            final_cmd[idx2] = static_cast<float>(q[1]);
        }
        for (size_t i = 0; i < motor_pos_target_.size(); ++i) {
            const size_t ji = motor2urdf_[i];
            motor_pos_target_[i] = final_cmd[ji];
            motor_vel_target_[i] = 0.0f;
            motor_kp_target_[i] = static_cast<float>(robot_cfg_->kp_[i]);
            motor_kd_target_[i] = static_cast<float>(robot_cfg_->kd_[i]);
            motor_tau_target_[i] = 0.0f;
        }
    }
    for (int k = 0; k < 5; ++k) {
        motors_mit_cmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
}

void RobotInterface::move_joints_smooth(
    const std::vector<double>& q_target,
    double duration_sec) {
    stop_hold_position();
    if (!is_init_.load()) {
        throw std::runtime_error("Motors are not initialized");
    }
    if (q_target.empty()) {
        throw std::runtime_error("q_target is empty");
    }

    refresh_joints();
    std::vector<float> q_start = get_joint_q();
    if (q_start.size() != q_target.size()) {
        throw std::runtime_error("q_target size does not match joint size");
    }

    const double dt = 0.004;
    int steps = static_cast<int>(duration_sec / dt);
    if (steps < 1) steps = 1;

    std::vector<float> q_cmd(q_start.size(), 0.0f);
    std::vector<float> v_zero(q_start.size(), 0.0f);

    for (int k = 1; k <= steps; ++k) {
        const double a = static_cast<double>(k) / static_cast<double>(steps);
        const double s = 3.0 * a * a - 2.0 * a * a * a;
        for (size_t i = 0; i < q_start.size(); ++i) {
            q_cmd[i] = static_cast<float>(
                q_start[i] + s * (q_target[i] - q_start[i]));
            v_zero[i] = 0.0f;
        }
        apply_action(q_cmd, v_zero);
        std::this_thread::sleep_for(
            std::chrono::microseconds(static_cast<int>(dt * 1000000.0)));
    }

    // Hold final target 1s for convergence
    for (size_t i = 0; i < q_start.size(); ++i) {
        q_cmd[i] = static_cast<float>(q_target[i]);
        v_zero[i] = 0.0f;
    }
    const double hold_sec = 1.0;
    const int hold_steps = static_cast<int>(hold_sec / dt);
    for (int k = 0; k < hold_steps; ++k) {
        apply_action(q_cmd, v_zero);
        std::this_thread::sleep_for(
            std::chrono::microseconds(static_cast<int>(dt * 1000000.0)));
    }

    // ★★★ 修复 2026-06-24 ★★★
    // 问题：DM电机在 MIT 模式下 CAN 反馈帧的位置字段不随物理转动更新
    // （SPD/POS 模式正常，仅 MIT 模式位置冻结於 ~0.097 rad），
    // get_motor_pos() 读到的始终是插值开始前的缓存值。
    // 
    // 作用：保持阶段结束后，通过 0xCC 查询命令主动向电机索要真实位置，
    // 确保 joint_q_ 反映的是电机实际角度而非过期的缓存值。
    // 
    // 副作用：增加约 1 秒延迟（refresh_joints 内部有 1000ms 收数等待）。
    refresh_joints();
}


void RobotInterface::refresh_joints() {
    {
        std::unique_lock<std::mutex> lock(joint_mutex_);
        exec_motors_parallel([this](std::shared_ptr<MotorDriver>& motor, int idx) {
            motor->refresh_motor_status();
        });

        std::this_thread::sleep_for(std::chrono::milliseconds(1000));

        exec_motors_parallel([this](std::shared_ptr<MotorDriver>& motor, int idx) {
            const float raw_pos = motor->get_motor_pos() * robot_cfg_->motor_sign_[idx];
            joint_q_[motor2urdf_[idx]] = motor_is_encos_[idx] ? wrap_to_pi_local(raw_pos) : raw_pos;
            joint_vel_[motor2urdf_[idx]] = motor->get_motor_spd() * robot_cfg_->motor_sign_[idx];
            joint_tau_[motor2urdf_[idx]] = motor->get_motor_current() * robot_cfg_->motor_sign_[idx];
        });

        if (!close_chain_joint_idx_.empty() && ankle_decouple_) {
            forward_close_chain();
        }
    }
}

void RobotInterface::set_zeros() {
    exec_motors_parallel([](std::shared_ptr<MotorDriver>& motor, int idx) {
        motor->set_motor_zero();
    });
}

void RobotInterface::clear_errors() {
    exec_motors_parallel([](std::shared_ptr<MotorDriver>& motor, int idx) {
        motor->clear_motor_error();
    });
}

void RobotInterface::init_motors() {
    exec_motors_parallel([](std::shared_ptr<MotorDriver>& motor, int idx) {
        motor->init_motor();
    });
    is_init_.store(true);
}

void RobotInterface::deinit_motors() {
    stop_hold_position();
    exec_motors_parallel([](std::shared_ptr<MotorDriver>& motor, int idx) {
        motor->deinit_motor();
    });
    is_init_.store(false);
}

void RobotInterface::motors_mit_cmd() {
    std::unique_lock<std::mutex> lock(motors_mutex_);
    std::vector<std::function<void()>> tasks;
    size_t count = 0;
    for (size_t bus = 0; bus < motors_cfg_->motor_interface_.size(); ++bus) {
        const size_t num_motors = motors_cfg_->motor_num_[bus];
        const size_t start_count = count;
        if (motors_cfg_->motor_interface_type_[bus] == "canfd") {
            tasks.push_back([this, start_count, num_motors]() {
                float pos[8] = {}, vel[8] = {}, kp[8] = {}, kd[8] = {}, tau[8] = {};
                for (size_t j = 0; j < num_motors; ++j) {
                    const size_t idx = start_count + j;
                    const long int motor_id = motors_cfg_->motor_id_[idx];
                    const size_t slot = (motor_id > 0 && motor_id <= 8) ? static_cast<size_t>(motor_id - 1) : j;
                    if (slot >= 8) continue;
                    pos[slot] = motor_pos_target_[idx] * robot_cfg_->motor_sign_[idx];
                    vel[slot] = motor_vel_target_[idx];
                    kp[slot]  = motor_kp_target_[idx];
                    kd[slot]  = motor_kd_target_[idx];
                    tau[slot] = motor_tau_target_[idx];
                }
                motors_[start_count]->motor_mit_cmd(pos, vel, kp, kd, tau);
            });
        } else {
            tasks.push_back([this, start_count, num_motors]() {
                for (size_t j = 0; j < num_motors; ++j) {
                    const size_t idx = start_count + j;
                    motors_[idx]->motor_mit_cmd(motor_pos_target_[idx] * robot_cfg_->motor_sign_[idx],
                                                 motor_vel_target_[idx],
                                                 motor_kp_target_[idx],
                                                 motor_kd_target_[idx],
                                                 motor_tau_target_[idx]);
                }
            });
        }
        count += num_motors;
    }
    thread_pool_->run_parallel(tasks);
}

void RobotInterface::start_hold_position() {
}

void RobotInterface::stop_hold_position() {
    hold_position_active_.store(false);
    if (hold_position_thread_.joinable()) {
        hold_position_thread_.join();
    }
}

void RobotInterface::exec_motors_parallel(const std::function<void(std::shared_ptr<MotorDriver>&, int)>& cmd_func) {
    std::unique_lock<std::mutex> lock(motors_mutex_);
    std::vector<std::function<void()>> tasks;
    size_t count = 0;
    
    for (size_t i = 0; i < motors_cfg_->motor_interface_.size(); ++i) {
        size_t num_motors = motors_cfg_->motor_num_[i];
        size_t start_idx = count;
        tasks.push_back([this, start_idx, num_motors, cmd_func]() {
            for (size_t j = 0; j < num_motors; ++j) {
                size_t idx = start_idx + j;
                cmd_func(motors_[idx], idx); 
            }
        });
        count += num_motors;
    }
    thread_pool_->run_parallel(tasks);
}
