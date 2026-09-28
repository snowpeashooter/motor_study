// SPDX-License-Identifier: GPL-3.0
// ENCOS classic CAN driver for Roboparty DOF6 bring-up.
#pragma once

#include <atomic>
#include <chrono>
#include <string>

#include "motor_driver.hpp"
#include "protocol/can_iso.hpp"
#include "utils.hpp"

enum ENCOS_Motor_Model {
    ENCOS_GENERIC = 0,
    ENCOS_Num_Of_Model
};

typedef struct {
    float PosMax;
    float SpdMax;
    float TauMax;
    float OKpMax;
    float OKdMax;
} ENCOS_Limit_Param;

class EncosMotorDriver : public MotorDriver {
   public:
    EncosMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                     ENCOS_Motor_Model motor_model, double motor_zero_offset = 0.0);
    ~EncosMotorDriver();

    void lock_motor() override;
    void unlock_motor() override;
    uint8_t init_motor() override;
    void deinit_motor() override;
    bool set_motor_zero() override;
    bool write_motor_flash() override { return true; }
    void get_motor_param(uint8_t param_cmd) override;

    void motor_pos_cmd(float pos, float spd, bool ignore_limit = false) override;
    void motor_spd_cmd(float spd) override;
    void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override;
    void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override;
    void set_motor_control_mode(uint8_t motor_control_mode) override { motor_control_mode_ = motor_control_mode; }
    int get_response_count() const override { return response_count_; }
    void set_motor_id(uint8_t old_id, uint8_t new_id) override {}
    void reset_motor_id() override {}
    void refresh_motor_status() override;
    void clear_motor_error() override {}

   private:
    std::atomic<int> response_count_{0};
    ENCOS_Motor_Model motor_model_;
    ENCOS_Limit_Param limit_param_;
    std::shared_ptr<MotorsCAN> can_;
    std::chrono::steady_clock::time_point last_mit_tx_time_{};
    std::chrono::steady_clock::time_point last_mit_log_time_{};
    int mit_min_interval_ms_{4};
    bool has_valid_position_{false};

    void send_frame(uint16_t can_id, const std::initializer_list<uint8_t>& data);
    void query_float_parameter(uint8_t query_code);
    void send_position_command(float pos_rad, float speed_rad_s, float current_limit_a, uint8_t ack_type = 2);
    void send_damping_brake(uint8_t ack_type = 1);
    void can_rx_cbk(const can_frame& rx_frame);
};
