#ifndef ENCODER_SPEED_METER_HPP
#define ENCODER_SPEED_METER_HPP
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <gpiod.hpp>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

/**
 * @brief 编码器测速器
 *
 * 通过 GPIO 中断监听编码器脉冲，计算电机转速 (RPS / RPM)
 */
class EncoderSpeedMeter {
public:
  /**
   * @param gpio_offset   GPIO 引脚编号
   * @param chip_path     GPIO 芯片路径 (如 /dev/gpiochip0)
   * @param sample_period 采样周期 (秒)
   * @param encoder_ppr   编码器每圈脉冲数
   * @param gear_ratio    减速比
   * @param queue_size    采样队列大小
   */
  EncoderSpeedMeter(unsigned int gpio_offset, unsigned int chip_index = 0,
                    double sample_period = 0.033, double encoder_ppr = 11.0,
                    double encoder_edges = 1.0, double gear_ratio = 56.0,
                    double alpha = 0.1, size_t queue_size = 10);

  ~EncoderSpeedMeter();

  /// 启动测速
  void start();

  /// 停止测速
  void stop();

  /// 获取当前转速 (转/秒)
  double get_rps() const;

  /// 获取当前转速 (转/分钟)
  double get_rpm() const;

private:
  /* ---------- GPIO ---------- */
  unsigned int gpio_offset_;
  unsigned int chip_index_;
  std::unique_ptr<gpiod::chip> chip_;
  std::unique_ptr<gpiod::line_request> line_request_;

  /* ---------- 参数 ---------- */
  double sample_period_;
  double encoder_ppr_;
  double encoder_edges_;
  double gear_ratio_;
  double alpha_;
  size_t queue_size_;

  /* ---------- 状态 ---------- */
  mutable std::mutex lock_;
  uint64_t pulse_count_{0};
  bool has_rising_{false};
  std::atomic<double> current_rps_{0.0};

  /* ---------- 线程通信 ---------- */
  std::queue<std::pair<double, uint64_t>> queue_;
  std::atomic<bool> stop_flag_{true};
  std::condition_variable cv_;

  /* ---------- 线程 ---------- */
  std::thread interrupt_thread_;
  std::thread sampler_thread_;
  std::thread processor_thread_;

  /* ---------- 私有方法 ---------- */
  void interrupt_loop();
  void handle_event(const gpiod::edge_event &event);
  void sampler_loop();
  void processor_loop();
};

#endif // ENCODER_SPEED_METER_HPP
