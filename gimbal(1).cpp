#include "gimbal.hpp"

#include <array>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <limits>

#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

namespace io {
namespace {
constexpr uint8_t RCIA_HEADER = 0x39;
constexpr uint8_t RCIA_TAIL = 0xFF;
constexpr std::size_t RCIA_RX_DATA_SIZE = 27; // 电控 -> 视觉
constexpr std::size_t RCIA_TX_DATA_SIZE = 17; // 视觉 -> 电控
constexpr double RAD2DEG = 180.0 / M_PI;
constexpr double DEG2RAD = M_PI / 180.0;

GimbalMode to_mode(uint8_t mode) {
  switch (mode) {
  case 0:
    return GimbalMode::IDLE;
  case 1:
    return GimbalMode::AUTO_AIM;
  case 2:
    return GimbalMode::SMALL_BUFF;
  case 3:
    return GimbalMode::BIG_BUFF;
  default:
    return GimbalMode::IDLE;
  }
}

int32_t decode_i32_be(const uint8_t *data) {
  const uint32_t value = (static_cast<uint32_t>(data[0]) << 24) |
                         (static_cast<uint32_t>(data[1]) << 16) |
                         (static_cast<uint32_t>(data[2]) << 8) |
                         static_cast<uint32_t>(data[3]);
  return static_cast<int32_t>(value);
}

uint32_t decode_u32_be(const uint8_t *data) {
  return (static_cast<uint32_t>(data[0]) << 24) |
         (static_cast<uint32_t>(data[1]) << 16) |
         (static_cast<uint32_t>(data[2]) << 8) | static_cast<uint32_t>(data[3]);
}

float decode_f32_be(const uint8_t *data) {
  const auto value = decode_u32_be(data);
  float result;
  std::memcpy(&result, &value, sizeof(result));
  return result;
}

void encode_i32_be(uint8_t *data, int32_t value) {
  const uint32_t u = static_cast<uint32_t>(value);
  data[0] = static_cast<uint8_t>((u >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((u >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(u & 0xFF);
}

void encode_u32_be(uint8_t *data, uint32_t value) {
  data[0] = static_cast<uint8_t>((value >> 24) & 0xFF);
  data[1] = static_cast<uint8_t>((value >> 16) & 0xFF);
  data[2] = static_cast<uint8_t>((value >> 8) & 0xFF);
  data[3] = static_cast<uint8_t>(value & 0xFF);
}

int32_t clamp_to_i32(double value) {
  if (value > static_cast<double>(std::numeric_limits<int32_t>::max())) {
    return std::numeric_limits<int32_t>::max();
  }
  if (value < static_cast<double>(std::numeric_limits<int32_t>::min())) {
    return std::numeric_limits<int32_t>::min();
  }
  return static_cast<int32_t>(std::lround(value));
}

} // namespace

Gimbal::Gimbal(const std::string &config_path) {
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");
  const auto com_baudrate =
      yaml["com_baudrate"] ? yaml["com_baudrate"].as<int>() : 115200;
  const auto protocol_name = yaml["serial_protocol"]
                                 ? yaml["serial_protocol"].as<std::string>()
                                 : "sp";
  protocol_ = (protocol_name == "rcia" || protocol_name == "rcia_infantry")
                  ? SerialProtocol::RCIA
                  : SerialProtocol::SP;
  angle_scale_ = yaml["serial_angle_scale"]
                     ? yaml["serial_angle_scale"].as<double>()
                     : 100.0;
  yaw_sign_ =
      yaml["serial_yaw_sign"] ? yaml["serial_yaw_sign"].as<double>() : 1.0;
  pitch_sign_ =
      yaml["serial_pitch_sign"] ? yaml["serial_pitch_sign"].as<double>() : 1.0;
  send_delta_ = yaml["serial_send_delta"] ? yaml["serial_send_delta"].as<bool>()
                                          : (protocol_ == SerialProtocol::RCIA);
  if (yaml["fixed_bullet_speed"]) {
    fixed_bullet_speed_ = yaml["fixed_bullet_speed"].as<float>();
    tools::logger()->info(
        "[Gimbal] Using fixed_bullet_speed from yaml: {:.2f} m/s",
        *fixed_bullet_speed_);
  }
  if (angle_scale_ <= 1e-6) {
    tools::logger()->warn(
        "[Gimbal] Invalid serial_angle_scale={:.6f}, fallback to 100.0",
        angle_scale_);
    angle_scale_ = 100.0;
  }

  // RCIA 协议需要 R_gimbal2imubody 来正确提取云台系 yaw/pitch
  if (protocol_ == SerialProtocol::RCIA) {
    if (yaml["R_gimbal2imubody"]) {
      auto r_data = yaml["R_gimbal2imubody"].as<std::vector<double>>();
      if (r_data.size() == 9) {
        R_gimbal2imubody_ =
            Eigen::Matrix<double, 3, 3, Eigen::RowMajor>(r_data.data());
        tools::logger()->info(
            "[Gimbal] Loaded R_gimbal2imubody for RCIA state correction.");
      }
    }
  }

  tools::logger()->info("[Gimbal] Serial protocol={}, angle_scale={}, "
                        "yaw_sign={}, pitch_sign={}, send_delta={}",
                        protocol_ == SerialProtocol::RCIA ? "rcia" : "sp",
                        angle_scale_, yaw_sign_, pitch_sign_, send_delta_);

  try {
    serial_.setPort(com_port);
    serial_.setBaudrate(static_cast<uint32_t>(com_baudrate));
    auto timeout = serial::Timeout::simpleTimeout(5);
    serial_.setTimeout(timeout);
    serial_.open();
  } catch (const std::exception &e) {
    tools::logger()->error("[Gimbal] Failed to open serial: {}", e.what());
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  queue_.pop();
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal() {
  quit_ = true;
  if (thread_.joinable())
    thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const {
  switch (mode) {
  case GimbalMode::IDLE:
    return "IDLE";
  case GimbalMode::AUTO_AIM:
    return "AUTO_AIM";
  case GimbalMode::SMALL_BUFF:
    return "SMALL_BUFF";
  case GimbalMode::BIG_BUFF:
    return "BIG_BUFF";
  default:
    return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t) {
  while (true) {
    auto [q_a, t_a] = queue_.pop();
    auto [q_b, t_b] = queue_.front();
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a)
      return q_c;
    if (!(t_a < t && t <= t_b))
      continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal) {
  const bool control = (VisionToGimbal.mode != 0);
  const bool fire = (VisionToGimbal.mode == 2);
  send(control, fire, VisionToGimbal.yaw, VisionToGimbal.yaw_vel,
       VisionToGimbal.yaw_acc, VisionToGimbal.pitch, VisionToGimbal.pitch_vel,
       VisionToGimbal.pitch_acc);
}

void Gimbal::send(bool control, bool fire, float yaw, float yaw_vel,
                  float yaw_acc, float pitch, float pitch_vel,
                  float pitch_acc) {
  try {
    std::lock_guard<std::mutex> serial_lock(serial_mutex_);
    if (protocol_ == SerialProtocol::SP) {
      tx_data_.mode = control ? (fire ? 2 : 1) : 0;
      tx_data_.yaw = yaw;
      tx_data_.yaw_vel = yaw_vel;
      tx_data_.yaw_acc = yaw_acc;
      tx_data_.pitch = pitch;
      tx_data_.pitch_vel = pitch_vel;
      tx_data_.pitch_acc = pitch_acc;
      tx_data_.crc16 =
          tools::get_crc16(reinterpret_cast<uint8_t *>(&tx_data_),
                           sizeof(tx_data_) - sizeof(tx_data_.crc16));

      serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
      return;
    }

    // RCIA 协议：0x39 0x39 + 17字节数据区（大端）
    std::array<uint8_t, 2 + RCIA_TX_DATA_SIZE> frame{};
    frame[0] = RCIA_HEADER;
    frame[1] = RCIA_HEADER;

    double cmd_yaw = static_cast<double>(yaw);
    double cmd_pitch = static_cast<double>(pitch);
    if (send_delta_) {
      GimbalState gs;
      {
        std::lock_guard<std::mutex> state_lock(mutex_);
        gs = state_;
      }
      cmd_yaw = tools::limit_rad(cmd_yaw - static_cast<double>(gs.yaw));
      cmd_pitch = cmd_pitch - static_cast<double>(gs.pitch);
    }

    const auto yaw_i =
        clamp_to_i32(yaw_sign_ * cmd_yaw * RAD2DEG * angle_scale_);
    const auto pitch_i =
        clamp_to_i32(pitch_sign_ * cmd_pitch * RAD2DEG * angle_scale_);
    const auto timestamp = static_cast<uint32_t>(
        std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());

    encode_i32_be(frame.data() + 2 + 0, pitch_i);
    encode_i32_be(frame.data() + 2 + 4, yaw_i);
    frame[2 + 8] = static_cast<uint8_t>(control ? 1 : 0); // vision_ready
    frame[2 + 9] = static_cast<uint8_t>(fire ? 1 : 0);    // fire
    frame[2 + 10] = RCIA_TAIL;
    encode_u32_be(frame.data() + 2 + 11, timestamp);
    frame[2 + 15] = 0; // aim_x
    frame[2 + 16] = 0; // aim_y

    serial_.write(frame.data(), frame.size());
  } catch (const std::exception &e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

Gimbal::ReadStatus Gimbal::read_exact(uint8_t *buffer, size_t size) {
  size_t offset = 0;
  while (offset < size && !quit_) {
    try {
      std::lock_guard<std::mutex> serial_lock(serial_mutex_);
      auto n = serial_.read(buffer + offset, size - offset);
      if (n == 0)
        return ReadStatus::TIMEOUT;
      offset += n;
    } catch (const std::exception &e) {
      (void)e;
      return ReadStatus::ERROR;
    }
  }
  return offset == size ? ReadStatus::OK : ReadStatus::TIMEOUT;
}

void Gimbal::read_thread() {
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 20) {
      error_count = 0;
      tools::logger()->warn(
          "[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (protocol_ == SerialProtocol::SP) {
      uint8_t last = 0, current = 0;
      bool found_header = false;
      while (!quit_) {
        auto status = read_exact(&current, 1);
        if (status == ReadStatus::TIMEOUT) {
          break;
        }
        if (status == ReadStatus::ERROR) {
          error_count++;
          break;
        }
        if (last == 'S' && current == 'P') {
          found_header = true;
          break;
        }
        last = current;
      }
      if (!found_header)
        continue;

      rx_data_.head[0] = 'S';
      rx_data_.head[1] = 'P';
      auto t = std::chrono::steady_clock::now();

      auto status = read_exact(reinterpret_cast<uint8_t *>(&rx_data_) +
                                   sizeof(rx_data_.head),
                               sizeof(rx_data_) - sizeof(rx_data_.head));
      if (status == ReadStatus::TIMEOUT) {
        continue;
      }
      if (status == ReadStatus::ERROR) {
        error_count++;
        continue;
      }

      if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_),
                              sizeof(rx_data_))) {
        tools::logger()->debug("[Gimbal] CRC16 check failed.");
        continue;
      }

      error_count = 0;
      Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2],
                           rx_data_.q[3]);
      queue_.push({q, t});

      std::lock_guard<std::mutex> lock(mutex_);
      state_.yaw = rx_data_.yaw;
      state_.yaw_vel = rx_data_.yaw_vel;
      state_.pitch = rx_data_.pitch;
      state_.pitch_vel = rx_data_.pitch_vel;
      const float reported_bullet_speed = rx_data_.bullet_speed;
      state_.bullet_speed =
          fixed_bullet_speed_.value_or(reported_bullet_speed);
      state_.bullet_count = rx_data_.bullet_count;
      mode_ = to_mode(rx_data_.mode);
      continue;
    }

    // RCIA 协议：0x39 0x39 + 27字节数据区（大端）
    uint8_t last = 0, current = 0;
    bool found_header = false;
    while (!quit_) {
      auto status = read_exact(&current, 1);
      if (status == ReadStatus::TIMEOUT) {
        break;
      }
      if (status == ReadStatus::ERROR) {
        error_count++;
        break;
      }
      if (last == RCIA_HEADER && current == RCIA_HEADER) {
        found_header = true;
        break;
      }
      last = current;
    }
    if (!found_header)
      continue;

    std::array<uint8_t, RCIA_RX_DATA_SIZE> data{};
    auto t = std::chrono::steady_clock::now();
    auto status = read_exact(data.data(), data.size());
    if (status == ReadStatus::TIMEOUT) {
      continue;
    }
    if (status == ReadStatus::ERROR) {
      error_count++;
      continue;
    }

    error_count = 0;

    // 新协议：4×float 四元数 + bullet_rate + enemy_color +
    // vision_mode + tail + timestamp
    const auto qw = decode_f32_be(data.data() + 0);
    const auto qx = decode_f32_be(data.data() + 4);
    const auto qy = decode_f32_be(data.data() + 8);
    const auto qz = decode_f32_be(data.data() + 12);

    const auto bullet_speed = decode_f32_be(data.data() + 16);
    const auto enemy_color = data[20]; // 0x42红 0x52蓝
    const auto vision_mode = data[21];
    const auto tail = data[22];
    if (tail != RCIA_TAIL) {
      tools::logger()->debug("[Gimbal] RCIA tail check failed: 0x{:02X}",
                             tail);
      continue;
    }
    const auto timestamp = decode_u32_be(data.data() + 23);

    Eigen::Quaterniond q(qw, qx, qy, qz);
    queue_.push({q.normalized(), t});

    // 将 IMU body→abs 四元数变换到 gimbal→world 坐标系，再提取 yaw/pitch
    Eigen::Matrix3d R_imubody2imuabs = q.toRotationMatrix();
    Eigen::Matrix3d R_gimbal2world =
        R_gimbal2imubody_.transpose() * R_imubody2imuabs * R_gimbal2imubody_;
    const double yaw = std::atan2(R_gimbal2world(1, 0), R_gimbal2world(0, 0));
    const double pitch =
        std::asin(-std::clamp(R_gimbal2world(2, 0), -1.0, 1.0));

    std::lock_guard<std::mutex> lock(mutex_);
    state_.yaw = static_cast<float>(yaw);
    state_.yaw_vel = 0.0F;
    state_.pitch = static_cast<float>(pitch);
    state_.pitch_vel = 0.0F;
    state_.bullet_speed = fixed_bullet_speed_.value_or(bullet_speed);
    state_.bullet_count = static_cast<uint16_t>(timestamp & 0xFFFF);
    state_.enemy_color = enemy_color;
    mode_ = to_mode(vision_mode);
  }

  tools::logger()->info("[Gimbal] read_thread stopped.");
}

void Gimbal::reconnect() {
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...",
                          i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open(); // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception &e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

} // namespace io
