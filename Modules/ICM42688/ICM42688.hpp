#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: TDK ICM42688 six-axis IMU I2C driver with topic output, RamFS commands, and gyro offset database.
constructor_args:
  - datarate: ICM42688::DataRate::DATA_RATE_1KHZ
  - accl_range: ICM42688::AcclRange::RANGE_16G
  - gyro_range: ICM42688::GyroRange::DPS_2000
  - rotation:
      w: 1.0
      x: 0.0
      y: 0.0
      z: 0.0
  - enable_clk_in: false
  - gyro_topic_name: "icm42688_gyro"
  - accl_topic_name: "icm42688_accl"
  - i2c_address: 0x68
template_args: []
required_hardware: i2c_icm42688 icm42688_int ramfs database
depends: []
=== END MANIFEST === */
// clang-format on

#include <algorithm>
#include <array>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <utility>

#include "app_framework.hpp"
#include "database.hpp"
#include "gpio.hpp"
#include "i2c.hpp"
#include "message.hpp"
#include "ramfs.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "timer.hpp"
#include "transform.hpp"

class ICM42688 : public LibXR::Application
{
 public:
  static constexpr float M_DEG2RAD_MULT = 0.01745329251F;
  static constexpr uint8_t ICM42688_REG_TEMP_DATA1 = 0x1D;
  static constexpr uint8_t ICM42688_READ_LEN = 14;
  static constexpr size_t ICM42688_TASK_STACK_DEPTH = 512;
  static constexpr uint16_t ICM42688_I2C_ADDRESS_DEFAULT = 0x68;

  enum class DataRate : uint8_t
  {
    DATA_RATE_UNKNOW = 0,
    DATA_RATE_32KHZ = 1,
    DATA_RATE_16KHZ = 2,
    DATA_RATE_8KHZ = 3,
    DATA_RATE_4KHZ = 4,
    DATA_RATE_2KHZ = 5,
    DATA_RATE_1KHZ = 6,
    DATA_RATE_200HZ = 7,
    DATA_RATE_100HZ = 8,
    DATA_RATE_50HZ = 9,
    DATA_RATE_25HZ = 10,
    DATA_RATE_12_5HZ = 11,
    DATA_RATE_500HZ = 15,
  };

  enum class GyroRange : uint8_t
  {
    DPS_2000 = 0,
    DPS_1000 = 1,
    DPS_500 = 2,
    DPS_250 = 3,
    DPS_125 = 4,
    DPS_62_5 = 5,
    DPS_31_25 = 6,
    DPS_15_625 = 7,
  };

  enum class AcclRange : uint8_t
  {
    RANGE_16G = 0,
    RANGE_8G = 1,
    RANGE_4G = 2,
    RANGE_2G = 3,
  };

  ICM42688(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
           DataRate data_rate, AcclRange accl_range, GyroRange gyro_range,
           LibXR::Quaternion<float>&& rotation, bool enable_clk_in,
           const char* gyro_topic_name, const char* accl_topic_name,
           uint16_t i2c_address)
      : data_rate_(data_rate),
        accl_range_(accl_range),
        gyro_range_(gyro_range),
        i2c_address_(i2c_address),
        enable_clk_in_(enable_clk_in),
        topic_gyro_(LibXR::Topic::CreateTopic<decltype(gyro_data_)>(gyro_topic_name)),
        topic_accl_(LibXR::Topic::CreateTopic<decltype(accl_data_)>(accl_topic_name)),
        int_(hw.template FindOrExit<LibXR::GPIO>({"icm42688_int"})),
        i2c_(hw.template FindOrExit<LibXR::I2C>({"i2c_icm42688"})),
        rotation_(std::move(rotation)),
        cmd_file_(LibXR::RamFS::CreateFile("icm42688", CommandFunc, this)),
        gyro_data_key_(*hw.template FindOrExit<LibXR::Database>({"database"}),
                       "icm42688_gyro_data",
                       Eigen::Matrix<float, 3, 1>::Zero())
  {
    app.Register(*this);
    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    ASSERT(int_->SetConfig({LibXR::GPIO::Direction::FALL_INTERRUPT,
                            LibXR::GPIO::Pull::UP}) == LibXR::ErrorCode::OK);
    int_->DisableInterrupt();

    const auto int_callback = LibXR::GPIO::Callback::Create(
        [](bool in_isr, ICM42688* self)
        {
          const uint32_t now = static_cast<uint32_t>(
              static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()));
          const uint32_t previous =
              self->last_int_us_.exchange(now, std::memory_order_relaxed);
          self->dt_us_.store(now - previous, std::memory_order_relaxed);
          self->new_data_.PostFromCallback(in_isr);
        },
        this);
    int_->RegisterCallback(int_callback);

    last_init_attempt_ms_ = LibXR::Timebase::GetMilliseconds();
    if (Init())
    {
      XR_LOG_PASS("ICM42688: Init success, WHO_AM_I=0x%02X.", who_am_i_);
    }
    else
    {
      XR_LOG_ERROR("ICM42688: Init failed; worker will retry.");
    }

#ifdef LIBXR_NOT_SUPPORT_MUTI_THREAD
    worker_timer_ = LibXR::Timer::CreateTask<ICM42688*>(ThreadPump, this, 1);
    LibXR::Timer::Add(worker_timer_);
    LibXR::Timer::Start(worker_timer_);
#else
    thread_.Create(this, ThreadFunc, "icm42688_thread", ICM42688_TASK_STACK_DEPTH,
                   LibXR::Thread::Priority::REALTIME);
#endif
  }

  void Off() { (void)WriteSingle(0x4E, 0x00); }
  void On() { (void)WriteSingle(0x4E, 0x0F); }

  bool Init()
  {
    initialized_ = false;
    last_error_ = LibXR::ErrorCode::OK;
    last_failed_register_ = 0;
    int_->DisableInterrupt();
    DrainPendingSamples();

    if (!WriteOk(0x76, 0x00) || !WriteOk(0x11, 0x01))
    {
      return false;
    }
    LibXR::Thread::Sleep(5);

    uint8_t value = 0;
    if (!ReadSingle(0x2D, value) || !WriteOk(0x76, 0x00) ||
        !ReadSingle(0x75, who_am_i_) || who_am_i_ != 0x47)
    {
      if (last_error_ == LibXR::ErrorCode::OK && who_am_i_ != 0x47)
      {
        last_error_ = LibXR::ErrorCode::FAILED;
        last_failed_register_ = 0x75;
      }
      return false;
    }

    if (!WriteOk(0x4E, 0x00))
    {
      return false;
    }

    if (!WriteOk(0x76, 0x01) || !WriteOk(0x0B, 0xA0) ||
        !WriteOk(0x0C, 0x05) || !WriteOk(0x0D, 0x19) ||
        !WriteOk(0x0E, 0xA0))
    {
      return false;
    }

    if (!WriteOk(0x76, 0x02) || !WriteOk(0x03, 0x05) ||
        !WriteOk(0x04, 0x19) || !WriteOk(0x05, 0xA0))
    {
      return false;
    }

    if (!WriteOk(0x76, 0x00) || !WriteOk(0x14, 0x12) ||
        !WriteOk(0x51, 0xCA) || !WriteOk(0x52, 0x22) ||
        !WriteOk(0x53, 0x0D) || !WriteOk(0x63, 0x00) ||
        !WriteOk(0x64, 0x00) || !WriteOk(0x65, 0x08) ||
        !WriteOk(0x66, 0x00) || !WriteOk(0x68, 0x00) ||
        !WriteOk(0x69, 0x00))
    {
      return false;
    }

    if (!ReadSingle(0x4D, value))
    {
      return false;
    }
    value = static_cast<uint8_t>((value & ~0xC0U) | 0x40U);
    if (!WriteOk(0x4D, value) || !WriteOk(0x76, 0x00) ||
        !WriteOk(0x4E, 0x0F) ||
        !WriteOk(0x4F, (static_cast<uint8_t>(gyro_range_) << 5U) |
                           static_cast<uint8_t>(data_rate_)) ||
        !WriteOk(0x50, (static_cast<uint8_t>(accl_range_) << 5U) |
                           static_cast<uint8_t>(data_rate_)) ||
        !WriteOk(0x76, 0x00) || !WriteOk(0x77, 0x95) ||
        !WriteOk(0x76, 0x01))
    {
      return false;
    }

    if (enable_clk_in_ && !WriteOk(0x7B, 0x04))
    {
      return false;
    }
    if (!WriteOk(0x76, 0x00))
    {
      return false;
    }

    LibXR::Thread::Sleep(50);
    last_int_us_.store(
        static_cast<uint32_t>(static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds())),
        std::memory_order_relaxed);
    dt_us_.store(0, std::memory_order_relaxed);
    initialized_ = true;
    int_->EnableInterrupt();
    return true;
  }

  bool Process() { return RunWorkerOnce(0); }

  bool IsOnline() const { return initialized_; }
  uint8_t GetWhoAmI() const { return who_am_i_; }
  uint8_t GetLastFailedRegister() const { return last_failed_register_; }
  LibXR::ErrorCode GetLastError() const { return last_error_; }
  uint32_t GetSampleCount() const { return sample_count_; }
  float GetTemperature() const { return temperature_; }
  const Eigen::Matrix<float, 3, 1>& GetGyro() const { return gyro_data_; }
  const Eigen::Matrix<float, 3, 1>& GetAccl() const { return accl_data_; }
  float GetSampleIntervalSeconds() const
  {
    const float actual_dt =
        static_cast<float>(dt_us_.load(std::memory_order_relaxed)) / 1000000.0F;
    return actual_dt > 0.0F ? actual_dt : GetIdealIntervalSeconds();
  }

  void OnMonitor() override
  {
    const uint32_t now = LibXR::Timebase::GetMilliseconds();
    if (now - last_monitor_ms_ < MONITOR_INTERVAL_MS)
    {
      return;
    }
    last_monitor_ms_ = now;

    if (!initialized_)
    {
      XR_LOG_WARN("ICM42688: Offline, WHO_AM_I=0x%02X.", who_am_i_);
      return;
    }

    if (sample_count_ == last_monitor_sample_count_)
    {
      XR_LOG_WARN("ICM42688: No INT1 samples received.");
      return;
    }
    last_monitor_sample_count_ = sample_count_;

    if (!gyro_data_.allFinite() || !accl_data_.allFinite())
    {
      XR_LOG_WARN("ICM42688: Non-finite sensor data.");
    }

    const float ideal_dt = GetIdealIntervalSeconds();
    const float actual_dt =
        static_cast<float>(dt_us_.load(std::memory_order_relaxed)) / 1000000.0F;
    if (ideal_dt > 0.0F && std::fabs(actual_dt - ideal_dt) > 0.00015F)
    {
      XR_LOG_WARN("ICM42688 Frequency Error: %6f", static_cast<double>(actual_dt));
    }
  }

 private:
  static constexpr uint32_t INIT_RETRY_INTERVAL_MS = 500;
  static constexpr uint32_t MONITOR_INTERVAL_MS = 1000;
  static constexpr uint32_t COMMAND_CALI_SETTLE_MS = 3000;
  static constexpr uint32_t COMMAND_CALI_COLLECT_SECONDS = 60;
  static constexpr uint32_t COMMAND_CALI_VERIFY_SECONDS = 60;

  static void ThreadFunc(ICM42688* self)
  {
    while (true)
    {
      (void)self->RunWorkerOnce(50);
    }
  }

  static void ThreadPump(ICM42688* self)
  {
    if (self == nullptr)
    {
      return;
    }
    (void)self->RunWorkerOnce(0);
  }

  static int CommandFunc(ICM42688* self, int argc, char** argv)
  {
    if (self == nullptr)
    {
      return -1;
    }
    return self->HandleCommand(argc, argv);
  }

  int HandleCommand(int argc, char** argv)
  {
    if (argc == 1)
    {
      PrintUsage();
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "list_offset") == 0)
    {
      const auto& offset = gyro_data_key_.data_;
      LibXR::STDIO::Printf<"Current gyro offset(rad/s): x=%f y=%f z=%f\r\n">(
          static_cast<double>(offset.x()), static_cast<double>(offset.y()),
          static_cast<double>(offset.z()));
      return 0;
    }

    if (argc == 2 && std::strcmp(argv[1], "cali") == 0)
    {
      return RunCalibrationCommand();
    }

    if (argc == 4 && std::strcmp(argv[1], "show") == 0)
    {
      int time_ms = std::atoi(argv[2]);
      int interval_ms = std::atoi(argv[3]);
      interval_ms = std::clamp(interval_ms, 2, 1000);

      while (time_ms > 0)
      {
        LibXR::STDIO::Printf<
            "Accel: x=%+5f y=%+5f z=%+5f | Gyro: x=%+5f y=%+5f z=%+5f | "
            "Temp: %+5f\r\n">(
            static_cast<double>(accl_data_.x()), static_cast<double>(accl_data_.y()),
            static_cast<double>(accl_data_.z()), static_cast<double>(gyro_data_.x()),
            static_cast<double>(gyro_data_.y()), static_cast<double>(gyro_data_.z()),
            static_cast<double>(temperature_));
        LibXR::Thread::Sleep(static_cast<uint32_t>(interval_ms));
        time_ms -= interval_ms;
      }
      return 0;
    }

    PrintUsage();
    return -1;
  }

  void PrintUsage() const
  {
    LibXR::STDIO::Printf<"Usage:\r\n">();
    LibXR::STDIO::Printf<
        "  icm42688 show <time_ms> <interval_ms> - Print sensor data.\r\n">();
    LibXR::STDIO::Printf<
        "  icm42688 list_offset                 - Show gyro offset.\r\n">();
    LibXR::STDIO::Printf<
        "  icm42688 cali                        - Calibrate gyro offset.\r\n">();
  }

  int RunCalibrationCommand()
  {
    gyro_data_key_.data_.setZero();
    gyro_cali_.setZero();
    cali_counter_ = 0;
    in_cali_ = true;

    LibXR::STDIO::Printf<
        "Starting gyro calibration. Keep the device steady.\r\n">();
    LibXR::Thread::Sleep(COMMAND_CALI_SETTLE_MS);

    for (uint32_t i = 0; i < COMMAND_CALI_COLLECT_SECONDS; i++)
    {
      LibXR::STDIO::Printf<"Progress: %lu / %lu\r">(
          static_cast<unsigned long>(i),
          static_cast<unsigned long>(COMMAND_CALI_COLLECT_SECONDS));
      LibXR::Thread::Sleep(1000);
    }
    in_cali_ = false;

    if (cali_counter_ == 0)
    {
      LibXR::STDIO::Printf<"\r\nCalibration failed: no samples.\r\n">();
      return -1;
    }

    gyro_data_key_.data_.x() =
        static_cast<float>(static_cast<double>(gyro_cali_.x()) /
                           static_cast<double>(cali_counter_)) *
        GetGyroLSB() * M_DEG2RAD_MULT;
    gyro_data_key_.data_.y() =
        static_cast<float>(static_cast<double>(gyro_cali_.y()) /
                           static_cast<double>(cali_counter_)) *
        GetGyroLSB() * M_DEG2RAD_MULT;
    gyro_data_key_.data_.z() =
        static_cast<float>(static_cast<double>(gyro_cali_.z()) /
                           static_cast<double>(cali_counter_)) *
        GetGyroLSB() * M_DEG2RAD_MULT;

    LibXR::STDIO::Printf<
        "\r\nCalibration result(rad/s): x=%f y=%f z=%f\r\n">(
        static_cast<double>(gyro_data_key_.data_.x()),
        static_cast<double>(gyro_data_key_.data_.y()),
        static_cast<double>(gyro_data_key_.data_.z()));

    LibXR::STDIO::Printf<"Analyzing calibration quality...\r\n">();
    gyro_cali_.setZero();
    cali_counter_ = 0;
    in_cali_ = true;
    for (uint32_t i = 0; i < COMMAND_CALI_VERIFY_SECONDS; i++)
    {
      LibXR::STDIO::Printf<"Progress: %lu / %lu\r">(
          static_cast<unsigned long>(i),
          static_cast<unsigned long>(COMMAND_CALI_VERIFY_SECONDS));
      LibXR::Thread::Sleep(1000);
    }
    in_cali_ = false;

    if (cali_counter_ > 0)
    {
      const float err_x =
          static_cast<float>(static_cast<double>(gyro_cali_.x()) /
                             static_cast<double>(cali_counter_)) *
              GetGyroLSB() * M_DEG2RAD_MULT -
          gyro_data_key_.data_.x();
      const float err_y =
          static_cast<float>(static_cast<double>(gyro_cali_.y()) /
                             static_cast<double>(cali_counter_)) *
              GetGyroLSB() * M_DEG2RAD_MULT -
          gyro_data_key_.data_.y();
      const float err_z =
          static_cast<float>(static_cast<double>(gyro_cali_.z()) /
                             static_cast<double>(cali_counter_)) *
              GetGyroLSB() * M_DEG2RAD_MULT -
          gyro_data_key_.data_.z();
      LibXR::STDIO::Printf<
          "\r\nCalibration error(rad/s): x=%f y=%f z=%f\r\n">(
          static_cast<double>(err_x), static_cast<double>(err_y),
          static_cast<double>(err_z));
    }

    (void)gyro_data_key_.Set(gyro_data_key_.data_);
    LibXR::STDIO::Printf<"Calibration data saved.\r\n">();
    return 0;
  }

  bool RunWorkerOnce(uint32_t wait_ms)
  {
    if (!initialized_)
    {
      RetryInit();
      return false;
    }

    if (new_data_.Wait(wait_ms) != LibXR::ErrorCode::OK)
    {
      return false;
    }

    return ReadAndPublish();
  }

  void RetryInit()
  {
    const uint32_t now = LibXR::Timebase::GetMilliseconds();
    if (now - last_init_attempt_ms_ < INIT_RETRY_INTERVAL_MS)
    {
      return;
    }

    last_init_attempt_ms_ = now;
    if (Init())
    {
      XR_LOG_PASS("ICM42688: Reconnected, WHO_AM_I=0x%02X.", who_am_i_);
    }
  }

  bool ReadAndPublish()
  {
    const LibXR::ErrorCode read_result = Read(ICM42688_REG_TEMP_DATA1, ICM42688_READ_LEN);
    if (read_result != LibXR::ErrorCode::OK)
    {
      last_error_ = read_result;
      last_failed_register_ = ICM42688_REG_TEMP_DATA1;
      initialized_ = false;
      int_->DisableInterrupt();
      XR_LOG_ERROR("ICM42688: Data read failed; reconnecting.");
      return false;
    }

    Parse();
    topic_gyro_.Publish(gyro_data_);
    topic_accl_.Publish(accl_data_);
    sample_count_++;
    return true;
  }

  void DrainPendingSamples()
  {
    while (new_data_.Wait(0) == LibXR::ErrorCode::OK)
    {
    }
  }

  LibXR::ErrorCode WriteSingle(uint8_t reg, uint8_t data)
  {
    return i2c_->MemWrite(i2c_address_, reg, {&data, 1}, op_i2c_write_);
  }

  bool WriteOk(uint8_t reg, uint8_t data)
  {
    const LibXR::ErrorCode result = WriteSingle(reg, data);
    if (result != LibXR::ErrorCode::OK)
    {
      last_error_ = result;
      last_failed_register_ = reg;
      return false;
    }
    return true;
  }

  bool ReadSingle(uint8_t reg, uint8_t& data)
  {
    const LibXR::ErrorCode result =
        i2c_->MemRead(i2c_address_, reg, {&data, 1}, op_i2c_read_);
    if (result != LibXR::ErrorCode::OK)
    {
      last_error_ = result;
      last_failed_register_ = reg;
      return false;
    }
    return true;
  }

  LibXR::ErrorCode Read(uint8_t reg, uint8_t len)
  {
    return i2c_->MemRead(i2c_address_, reg, {buffer_, len}, op_i2c_read_);
  }

  void Parse()
  {
    const int16_t temperature_raw =
        static_cast<int16_t>((static_cast<uint16_t>(buffer_[0]) << 8U) | buffer_[1]);
    temperature_ = static_cast<float>(temperature_raw) / 132.48F + 25.0F;

    std::array<std::int16_t, 3> accl_raw_counts{};
    std::array<std::int16_t, 3> gyro_raw_counts{};
    std::array<float, 3> accl_raw{};
    std::array<float, 3> gyro_raw{};
    for (size_t index = 0; index < 3; index++)
    {
      accl_raw_counts[index] =
          static_cast<int16_t>((static_cast<uint16_t>(buffer_[index * 2U + 2U]) << 8U) |
                               buffer_[index * 2U + 3U]);
      gyro_raw_counts[index] =
          static_cast<int16_t>((static_cast<uint16_t>(buffer_[index * 2U + 8U]) << 8U) |
                               buffer_[index * 2U + 9U]);
      accl_raw[index] = static_cast<float>(accl_raw_counts[index]) * GetAcclLSB();
      gyro_raw[index] =
          static_cast<float>(gyro_raw_counts[index]) * GetGyroLSB() * M_DEG2RAD_MULT;
    }

    if (in_cali_)
    {
      gyro_cali_.x() += gyro_raw_counts[0];
      gyro_cali_.y() += gyro_raw_counts[1];
      gyro_cali_.z() += gyro_raw_counts[2];
      cali_counter_++;
    }

    accl_data_ =
        rotation_ * Eigen::Matrix<float, 3, 1>(accl_raw[0], accl_raw[1], accl_raw[2]);
    const Eigen::Matrix<float, 3, 1> gyro_corrected =
        Eigen::Matrix<float, 3, 1>(gyro_raw[0], gyro_raw[1], gyro_raw[2]) -
        gyro_data_key_.data_;
    gyro_data_ = rotation_ * gyro_corrected;
  }

  float GetAcclLSB() const
  {
    switch (accl_range_)
    {
      case AcclRange::RANGE_16G:
        return 1.0F / 2048.0F;
      case AcclRange::RANGE_8G:
        return 1.0F / 4096.0F;
      case AcclRange::RANGE_4G:
        return 1.0F / 8192.0F;
      case AcclRange::RANGE_2G:
        return 1.0F / 16384.0F;
      default:
        return 0.0F;
    }
  }

  float GetGyroLSB() const
  {
    switch (gyro_range_)
    {
      case GyroRange::DPS_2000:
        return 1.0F / 16.384F;
      case GyroRange::DPS_1000:
        return 1.0F / 32.768F;
      case GyroRange::DPS_500:
        return 1.0F / 65.536F;
      case GyroRange::DPS_250:
        return 1.0F / 131.072F;
      case GyroRange::DPS_125:
        return 1.0F / 262.144F;
      case GyroRange::DPS_62_5:
        return 1.0F / 524.288F;
      case GyroRange::DPS_31_25:
        return 1.0F / 1048.576F;
      case GyroRange::DPS_15_625:
        return 1.0F / 2097.152F;
      default:
        return 0.0F;
    }
  }

  float GetIdealIntervalSeconds() const
  {
    switch (data_rate_)
    {
      case DataRate::DATA_RATE_32KHZ:
        return 0.00003125F;
      case DataRate::DATA_RATE_16KHZ:
        return 0.0000625F;
      case DataRate::DATA_RATE_8KHZ:
        return 0.000125F;
      case DataRate::DATA_RATE_4KHZ:
        return 0.00025F;
      case DataRate::DATA_RATE_2KHZ:
        return 0.0005F;
      case DataRate::DATA_RATE_1KHZ:
        return 0.001F;
      case DataRate::DATA_RATE_500HZ:
        return 0.002F;
      case DataRate::DATA_RATE_200HZ:
        return 0.005F;
      case DataRate::DATA_RATE_100HZ:
        return 0.01F;
      case DataRate::DATA_RATE_50HZ:
        return 0.02F;
      case DataRate::DATA_RATE_25HZ:
        return 0.04F;
      case DataRate::DATA_RATE_12_5HZ:
        return 0.08F;
      default:
        return 0.0F;
    }
  }

  DataRate data_rate_;
  AcclRange accl_range_;
  GyroRange gyro_range_;
  uint16_t i2c_address_;
  float temperature_ = 0.0F;
  bool enable_clk_in_ = false;
  bool initialized_ = false;
  bool in_cali_ = false;
  uint8_t who_am_i_ = 0;
  uint8_t last_failed_register_ = 0;
  LibXR::ErrorCode last_error_ = LibXR::ErrorCode::OK;
  uint32_t last_init_attempt_ms_ = 0;
  uint32_t sample_count_ = 0;
  uint32_t last_monitor_sample_count_ = 0;
  uint32_t last_monitor_ms_ = 0;
  uint32_t cali_counter_ = 0;

  std::atomic<uint32_t> last_int_us_{0};
  std::atomic<uint32_t> dt_us_{0};

  uint8_t buffer_[ICM42688_READ_LEN]{};
  Eigen::Matrix<float, 3, 1> gyro_data_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<float, 3, 1> accl_data_ = Eigen::Matrix<float, 3, 1>::Zero();
  Eigen::Matrix<std::int64_t, 3, 1> gyro_cali_ =
      Eigen::Matrix<std::int64_t, 3, 1>::Zero();

  LibXR::Topic topic_gyro_;
  LibXR::Topic topic_accl_;
  LibXR::GPIO* int_;
  LibXR::I2C* i2c_;
  LibXR::Quaternion<float> rotation_;
  LibXR::Semaphore new_data_;
  LibXR::ReadOperation op_i2c_read_;
  LibXR::WriteOperation op_i2c_write_;
  LibXR::RamFS::File cmd_file_;
  LibXR::Database::Key<Eigen::Matrix<float, 3, 1>> gyro_data_key_;
  LibXR::Thread thread_;
  LibXR::Timer::TimerHandle worker_timer_ = nullptr;
};
