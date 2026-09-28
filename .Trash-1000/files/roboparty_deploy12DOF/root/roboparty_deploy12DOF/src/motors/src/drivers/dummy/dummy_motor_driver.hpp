#pragma once
#include "motor_driver.hpp"

class DummyMotorDriver : public MotorDriver {
 public:
  DummyMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& interface,
                   int motor_model, double motor_zero_offset = 0.0) {
    (void)interface_type; (void)motor_model;
    motor_id_ = motor_id;
    can_interface_ = interface;
    motor_zero_offset_ = motor_zero_offset;
    motor_control_mode_ = NONE;
    error_id_ = 0;
  }
  void lock_motor() override {}
  void unlock_motor() override { motor_control_mode_ = NONE; }
  uint8_t init_motor() override { motor_control_mode_ = MIT; return 0; }
  void deinit_motor() override { motor_control_mode_ = NONE; }
  bool set_motor_zero() override { motor_pos_ = 0.f; return true; }
  bool write_motor_flash() override { return true; }
  void get_motor_param(uint8_t param_cmd) override { (void)param_cmd; }
  void motor_pos_cmd(float pos, float spd, bool ignore_limit = false) override { (void)spd; (void)ignore_limit; motor_pos_ = pos; }
  void motor_spd_cmd(float spd) override { motor_spd_ = spd; }
  void motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) override {
    (void)f_kp; (void)f_kd; (void)f_t; motor_pos_ = f_p; motor_spd_ = f_v;
  }
  void motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) override {
    if (!f_p || !f_v) return;
    motor_mit_cmd(f_p[motor_id_ - 1], f_v[motor_id_ - 1], f_kp ? f_kp[motor_id_ - 1] : 0.f,
                  f_kd ? f_kd[motor_id_ - 1] : 0.f, f_t ? f_t[motor_id_ - 1] : 0.f);
  }
  void set_motor_control_mode(uint8_t motor_control_mode) override { motor_control_mode_ = motor_control_mode; }
  int get_response_count() const override { return 0; }
  void refresh_motor_status() override {}
  void clear_motor_error() override { error_id_ = 0; }
  void set_motor_id(uint8_t old_id, uint8_t new_id) override { (void)old_id; motor_id_ = new_id; }
  void reset_motor_id() override {}
};
