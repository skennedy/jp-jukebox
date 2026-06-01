#pragma once

#include "esphome/core/component.h"
#include "esphome/components/microphone/microphone.h"
#include "esphome/core/log.h"

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

#include <lwip/sockets.h>
#include <lwip/inet.h>

#include <string>
#include <vector>

namespace esphome {
namespace vban_audio {

static const char *const TAG = "vban_audio";

#define VBAN_SAMPLES_PER_PACKET 128
#define VBAN_RING_PACKETS       32
#define VBAN_TASK_STACK         4096
#define VBAN_TASK_PRIORITY      4

// VBAN format_format values
#define VBAN_FORMAT_INT16  0x01
#define VBAN_FORMAT_INT32  0x03

struct __attribute__((packed)) VBANHeader {
  char     vban[4];
  uint8_t  format_sr;
  uint8_t  format_nbs;
  uint8_t  format_nbc;
  uint8_t  format_format;
  char     streamname[16];
  uint32_t frame_counter;
};

// Samples stored as int32_t regardless of input/output format.
// For 16-bit input: values are sign-extended into int32_t.
// For 32-bit input (PCM1808): 24-bit audio in upper 24 bits, passed through directly.
struct AudioPacket {
  int32_t samples[VBAN_SAMPLES_PER_PACKET];
};

class VBANAudio : public Component {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;

  void set_microphone(microphone::Microphone *mic);
  void set_target_ip(const std::string &ip);
  void set_target_port(uint16_t port);
  void set_sample_rate(uint32_t rate);
  void set_stream_name(const std::string &name);
  void set_gain(float gain);
  void set_bits_per_sample(uint8_t bps) { bits_per_sample_ = bps; }
  void set_channels(uint8_t channels) { channels_ = channels; }

 protected:
  static void tx_task_(void *arg);
  void tx_loop_();

  void microphone_bytes_callback_(const std::vector<uint8_t> &data);
  void push_samples_(const int32_t *samples, size_t count);

  uint8_t vban_sr_index_(uint32_t rate);

  microphone::Microphone *microphone_{nullptr};

  QueueHandle_t sample_queue_{nullptr};
  TaskHandle_t tx_task_handle_{nullptr};
  bool task_started_{false};

  int sock_{-1};
  sockaddr_in dest_addr_{};

  std::string target_ip_str_;
  uint16_t port_{6980};
  uint32_t sample_rate_{48000};
  std::string stream_name_{"ESP32"};
  float gain_{1.0f};
  uint8_t bits_per_sample_{16};
  uint8_t channels_{1};
};

}  // namespace vban_audio
}  // namespace esphome
