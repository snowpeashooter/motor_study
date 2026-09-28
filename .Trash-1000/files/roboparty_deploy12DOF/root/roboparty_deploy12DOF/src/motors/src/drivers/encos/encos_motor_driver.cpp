// SPDX-License-Identifier: GPL-3.0
#include "encos_motor_driver.hpp"

#include <cstring>
#include <cmath>
#include <array>
#include <cstdlib>
#include <limits>

ENCOS_Limit_Param encos_limit_param[ENCOS_Num_Of_Model] = {
    // EC-A4315-P2-36: reducer ratio 8.28, peak torque 10.22 Nm, peak speed 117 RPM.
    // Position range is protocol mapping range, not mechanical joint limit.
    // Kp/Kd ranges follow ENCOS servo/MIT command scaling used by the vendor tool package.
    {12.5f, 12.25f, 10.22f, 500.0f, 5.0f},
};

static uint32_t float_to_u32_be(float value) {
    uint32_t raw = 0;
    static_assert(sizeof(float) == sizeof(uint32_t));
    std::memcpy(&raw, &value, sizeof(float));
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
    raw = __builtin_bswap32(raw);
#endif
    return raw;
}


static uint16_t float_to_uint(float x, float x_min, float x_max, uint16_t bits) {
    const float span = x_max - x_min;
    const float offset = x_min;
    const uint32_t max_int = (1u << bits) - 1u;
    const float clamped = limit(x, x_min, x_max);
    return static_cast<uint16_t>((clamped - offset) * static_cast<float>(max_int) / span + 0.5f);
}

static float wrap_delta_to_pi(float delta) {
    if (!std::isfinite(delta)) return 0.0f;
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    return std::remainder(delta, two_pi);
}

static float nearest_equivalent_to_current(float current, float target) {
    return current + wrap_delta_to_pi(target - current);
}

static float nearest_equivalent_within_limit(float current, float target, float limit) {
    if (!std::isfinite(current) || !std::isfinite(target) || limit <= 0.0f) return 0.0f;
    const float two_pi = 2.0f * static_cast<float>(M_PI);
    float best = std::remainder(target, two_pi);
    float best_dist = std::numeric_limits<float>::infinity();
    for (int k = -8; k <= 8; ++k) {
        const float candidate = best + static_cast<float>(k) * two_pi;
        if (candidate < -limit || candidate > limit) continue;
        const float dist = std::abs(candidate - current);
        if (dist < best_dist) {
            best = candidate;
            best_dist = dist;
        }
    }
    return best;
}

static bool is_reasonable_encos_position(float pos_rad, bool has_last, float last_pos_rad) {
    if (!std::isfinite(pos_rad)) return false;
    if (std::abs(pos_rad) > 8.0f * static_cast<float>(M_PI)) return false;
    (void)has_last;
    (void)last_pos_rad;
    return true;
}

static float u32_be_to_float(const uint8_t* data) {
    uint32_t raw = (static_cast<uint32_t>(data[0]) << 24) |
                   (static_cast<uint32_t>(data[1]) << 16) |
                   (static_cast<uint32_t>(data[2]) << 8) |
                   static_cast<uint32_t>(data[3]);
#if __BYTE_ORDER__ == __ORDER_LITTLE_ENDIAN__
raw = __builtin_bswap32(raw);
#endif
    float value = 0.0f;
    std::memcpy(&value, &raw, sizeof(float));
    return value;
}

EncosMotorDriver::EncosMotorDriver(uint16_t motor_id, const std::string& interface_type, const std::string& can_interface,
                                   ENCOS_Motor_Model motor_model, double motor_zero_offset)
    : MotorDriver(), motor_model_(motor_model) {
    if (interface_type != "can") {
        throw std::runtime_error("ENCOS driver only supports classic CAN interface");
    }
    motor_id_ = motor_id;
    limit_param_ = encos_limit_param[motor_model_];
    can_interface_ = can_interface;
    motor_zero_offset_ = motor_zero_offset;
    comm_type_ = CommType::CAN;
    can_ = MotorsCAN::get(can_interface);
    if (const char* env = std::getenv("ENCOS_MIT_MIN_INTERVAL_MS")) {
        char* end = nullptr;
        const long value = std::strtol(env, &end, 10);
        if (end != env && value >= 0 && value <= 1000) {
            mit_min_interval_ms_ = static_cast<int>(value);
        }
    }
    CanCbkFunc can_callback = std::bind(&EncosMotorDriver::can_rx_cbk, this, std::placeholders::_1);
    can_->add_can_callback(can_callback, motor_id_);
    can_->add_can_callback(can_callback, 0x7FF);
}

EncosMotorDriver::~EncosMotorDriver() {
    if (can_) {
        can_->remove_can_callback(motor_id_);
    }
}

void EncosMotorDriver::send_frame(uint16_t can_id, const std::initializer_list<uint8_t>& data) {
    can_frame tx_frame{};
    tx_frame.can_id = can_id & CAN_SFF_MASK;
    tx_frame.can_dlc = static_cast<__u8>(std::min<size_t>(8, data.size()));
    size_t i = 0;
    for (uint8_t byte : data) {
        if (i >= 8) break;
        tx_frame.data[i++] = byte;
    }
    can_->transmit(tx_frame);
    response_count_++;
}

void EncosMotorDriver::lock_motor() {
    set_motor_control_mode(POS);
    refresh_motor_status();
}

void EncosMotorDriver::unlock_motor() {
    send_damping_brake(1);
    motor_control_mode_ = NONE;
}

uint8_t EncosMotorDriver::init_motor() {
    set_motor_control_mode(MIT);
    // Manual 9.1.5: 0x80 0x01 releases the electromagnetic brake.
    send_frame(motor_id_, {0x80, 0x01});
    Timer::sleep_for(setup_sleep_time);
    refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);
    // Do not command motion here; RobotInterface::init_motors() sets a current-position MIT hold target after feedback refresh.
    motor_mit_cmd(motor_pos_, 0.0f, 0.0f, 0.0f, 0.0f);
    Timer::sleep_for(setup_sleep_time);
    return error_id_;
}

void EncosMotorDriver::deinit_motor() {
    unlock_motor();
    Timer::sleep_for(normal_sleep_time);
}

bool EncosMotorDriver::set_motor_zero() {
    const uint8_t mid_h = (motor_id_ >> 8) & 0xFF;
    const uint8_t mid_l = motor_id_ & 0xFF;
    send_frame(0x7FF, {mid_h, mid_l, 0x00, 0x03});
    Timer::sleep_for(setup_sleep_time);
    refresh_motor_status();
    Timer::sleep_for(setup_sleep_time);
    logger_->info("ENCOS motor_id: {} position after zero: {} rad", motor_id_, get_motor_pos());
    return std::abs(get_motor_pos()) <= 0.5f * static_cast<float>(M_PI) / 180.0f;
}

void EncosMotorDriver::get_motor_param(uint8_t param_cmd) {
    query_float_parameter(param_cmd);
}

void EncosMotorDriver::query_float_parameter(uint8_t query_code) {
    send_frame(motor_id_, {0xE1, query_code});
}

void EncosMotorDriver::refresh_motor_status() {
    query_float_parameter(1);
    query_float_parameter(2);
}

void EncosMotorDriver::send_damping_brake(uint8_t ack_type) {
    (void)ack_type;
    // Manual 9.1.4: 0x69 00 00 = variable damping brake, return type 1.
    send_frame(motor_id_, {0x69, 0x00, 0x00});
}

void EncosMotorDriver::send_position_command(float pos_rad, float speed_rad_s, float current_limit_a, uint8_t ack_type) {
    const float pos_deg = (pos_rad - static_cast<float>(motor_zero_offset_)) * 180.0f / static_cast<float>(M_PI);
    const float speed_rpm = std::abs(speed_rad_s) * 60.0f / (2.0f * static_cast<float>(M_PI));
    const uint32_t pos_bits = float_to_u32_be(pos_deg);
    const uint16_t speed_raw = static_cast<uint16_t>(limit(speed_rpm * 10.0f, 0.0f, 32767.0f));
    const uint16_t cur_raw = static_cast<uint16_t>(limit(current_limit_a * 10.0f, 0.0f, 4095.0f));
    uint64_t packed = (static_cast<uint64_t>(0x01) << 61) |
                      (static_cast<uint64_t>(pos_bits) << 29) |
                      (static_cast<uint64_t>(speed_raw) << 14) |
                      (static_cast<uint64_t>(cur_raw) << 2) |
                      (ack_type & 0x03);
    can_frame tx_frame{};
    tx_frame.can_id = motor_id_ & CAN_SFF_MASK;
    tx_frame.can_dlc = 8;
    for (int i = 7; i >= 0; --i) {
        tx_frame.data[i] = packed & 0xFF;
        packed >>= 8;
    }
    can_->transmit(tx_frame);
    response_count_++;
}


static std::array<uint8_t, 8> make_mit_frame(float pos_rad, float vel_rad_s, float kp, float kd, float torque_nm,
                                             const ENCOS_Limit_Param& limit_param) {
    const uint64_t mode = 0x00;
    const uint64_t kp_raw = float_to_uint(kp, 0.0f, limit_param.OKpMax, 12);
    const uint64_t kd_raw = float_to_uint(kd, 0.0f, limit_param.OKdMax, 9);
    const uint64_t pos_raw = float_to_uint(pos_rad, -limit_param.PosMax, limit_param.PosMax, 16);
    const uint64_t vel_raw = float_to_uint(vel_rad_s, -limit_param.SpdMax, limit_param.SpdMax, 12);
    const uint64_t tor_raw = float_to_uint(torque_nm, -limit_param.TauMax, limit_param.TauMax, 12);
    uint64_t packed = (mode << 61) | (kp_raw << 49) | (kd_raw << 40) | (pos_raw << 24) | (vel_raw << 12) | tor_raw;
    std::array<uint8_t, 8> out{};
    for (int i = 7; i >= 0; --i) {
        out[i] = packed & 0xFF;
        packed >>= 8;
    }
    return out;
}

void EncosMotorDriver::motor_pos_cmd(float pos, float spd, bool ignore_limit) {
    set_motor_control_mode(POS);
    const float limited_pos = ignore_limit ? pos : limit(pos, -limit_param_.PosMax, limit_param_.PosMax);
    const float speed = spd > 1e-4f ? spd : 0.5f;
    send_position_command(limited_pos, speed, 0.5f, 2);
}

void EncosMotorDriver::motor_spd_cmd(float spd) {
    send_damping_brake(1);
}

void EncosMotorDriver::motor_mit_cmd(float f_p, float f_v, float f_kp, float f_kd, float f_t) {
    set_motor_control_mode(MIT);
    const auto now = std::chrono::steady_clock::now();
    if (mit_min_interval_ms_ > 0 &&
        last_mit_tx_time_.time_since_epoch().count() != 0 &&
        std::chrono::duration_cast<std::chrono::milliseconds>(now - last_mit_tx_time_).count() < mit_min_interval_ms_) {
        return;
    }
    last_mit_tx_time_ = now;
    const float pos = limit(f_p - static_cast<float>(motor_zero_offset_), -limit_param_.PosMax, limit_param_.PosMax);
    const float vel = limit(f_v, -limit_param_.SpdMax, limit_param_.SpdMax);
    const float kp = limit(f_kp, 0.0f, limit_param_.OKpMax);
    const float kd = limit(f_kd, 0.0f, limit_param_.OKdMax);
    const float tor = limit(f_t, -limit_param_.TauMax, limit_param_.TauMax);
    const auto data = make_mit_frame(pos, vel, kp, kd, tor, limit_param_);
    if (motor_id_ == 2) {
        const auto log_now = std::chrono::steady_clock::now();
        if (last_mit_log_time_.time_since_epoch().count() == 0 ||
            std::chrono::duration_cast<std::chrono::milliseconds>(log_now - last_mit_log_time_).count() >= 2000) {
            last_mit_log_time_ = log_now;
            logger_->debug("ENCOS MIT target id={} pos_rad={:.4f} pos_deg={:.2f} vel={:.4f} kp={:.2f} kd={:.2f} tau={:.4f} frame={:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X} {:02X}",
                           motor_id_, pos, pos * 180.0f / static_cast<float>(M_PI), vel, kp, kd, tor,
                           data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7]);
        }
    }
    can_frame tx_frame{};
    tx_frame.can_id = motor_id_ & CAN_SFF_MASK;
    tx_frame.can_dlc = 8;
    std::copy(data.begin(), data.end(), tx_frame.data);
    can_->transmit(tx_frame);
    // ENCOS MIT rate is configurable through ENCOS_MIT_MIN_INTERVAL_MS.
    // Default 4ms follows the Roboparty 250Hz control loop; set a larger value
    // when using weak USB-CAN adapters or during early bring-up.
}

void EncosMotorDriver::motor_mit_cmd(float* f_p, float* f_v, float* f_kp, float* f_kd, float* f_t) {
    if (!f_p || !f_v || !f_kp || !f_kd || !f_t) return;
    const uint8_t slot = 0;
    motor_mit_cmd(f_p[slot], f_v[slot], f_kp[slot], f_kd[slot], f_t[slot]);
}

void EncosMotorDriver::can_rx_cbk(const can_frame& rx_frame) {
    if (rx_frame.can_id == 0x7FF) {
        if (rx_frame.can_dlc >= 4) {
            const uint16_t ack_id = (static_cast<uint16_t>(rx_frame.data[0]) << 8) | rx_frame.data[1];
            if (ack_id == motor_id_ && rx_frame.data[2] == 0x01) {
                response_count_ = 0;
                error_id_ = (rx_frame.data[3] == 0x00) ? 1 : 0;
            }
        }
        return;
    }
    if ((rx_frame.can_id & CAN_SFF_MASK) != motor_id_ || rx_frame.can_dlc < 2) return;
    response_count_ = 0;
    const uint8_t msg_type = (rx_frame.data[0] >> 5) & 0x07;
    const uint8_t err = rx_frame.data[0] & 0x1F;
    error_id_ = err;
    if (msg_type == 5 && rx_frame.can_dlc >= 6) {
        const uint8_t query_code = rx_frame.data[1];
        const float value = u32_be_to_float(&rx_frame.data[2]);
        if (query_code == 1) {
            const float pos_rad = value * static_cast<float>(M_PI) / 180.0f + static_cast<float>(motor_zero_offset_);
            if (is_reasonable_encos_position(pos_rad, has_valid_position_, motor_pos_)) {
                motor_pos_ = has_valid_position_ ? nearest_equivalent_to_current(motor_pos_, pos_rad) : pos_rad;
                has_valid_position_ = true;
            } else {
                // Drop noisy/invalid E101 replies silently; valid cached position is kept.
            }
        } else if (query_code == 2) {
            if (std::isfinite(value)) {
                motor_spd_ = value * 2.0f * static_cast<float>(M_PI) / 60.0f;
            }
        } else if (query_code == 3) {
            if (std::isfinite(value)) {
                motor_current_ = value;
            }
        }
    }
}
