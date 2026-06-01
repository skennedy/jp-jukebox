#include "vban_audio.h"
#include <cstring>

namespace esphome {
namespace vban_audio {

void VBANAudio::setup() {
  ESP_LOGI(TAG, "Setting up VBAN Audio");

  sample_queue_ = xQueueCreate(VBAN_RING_PACKETS, sizeof(AudioPacket));
  if (!sample_queue_) {
    ESP_LOGE(TAG, "Failed to create audio queue");
    return;
  }

  if (microphone_) {
    microphone_->add_data_callback([this](const std::vector<uint8_t> &data) {
      this->microphone_bytes_callback_(data);
    });
  }
}

void VBANAudio::loop() {
  if (task_started_)
    return;
  task_started_ = true;
  xTaskCreatePinnedToCore(tx_task_, "vban_tx", VBAN_TASK_STACK, this, VBAN_TASK_PRIORITY, &tx_task_handle_, 0);
  ESP_LOGI(TAG, "VBAN TX task started");
}

void VBANAudio::dump_config() {
  ESP_LOGCONFIG(TAG, "VBAN Audio:");
  ESP_LOGCONFIG(TAG, "  Stream name: %s", stream_name_.c_str());
  ESP_LOGCONFIG(TAG, "  Target IP: %s", target_ip_str_.c_str());
  ESP_LOGCONFIG(TAG, "  Port: %u", port_);
  ESP_LOGCONFIG(TAG, "  Sample rate: %u", sample_rate_);
  ESP_LOGCONFIG(TAG, "  Input bits per sample: %u", bits_per_sample_);
  ESP_LOGCONFIG(TAG, "  VBAN format: INT%u", bits_per_sample_);
  ESP_LOGCONFIG(TAG, "  Channels: %u", channels_);
  ESP_LOGCONFIG(TAG, "  Gain: %.2f", gain_);
}

void VBANAudio::set_microphone(microphone::Microphone *mic) { microphone_ = mic; }
void VBANAudio::set_target_ip(const std::string &ip) { target_ip_str_ = ip; }
void VBANAudio::set_target_port(uint16_t port) { port_ = port; }
void VBANAudio::set_sample_rate(uint32_t rate) { sample_rate_ = rate; }
void VBANAudio::set_stream_name(const std::string &name) { stream_name_ = name.substr(0, 16); }

void VBANAudio::set_gain(float gain) {
  if (gain < 0.0f) gain = 0.0f;
  if (gain > 10.0f) gain = 10.0f;
  gain_ = gain;
}

uint8_t VBANAudio::vban_sr_index_(uint32_t rate) {
  switch (rate) {
    case 6000:   return 0;
    case 12000:  return 1;
    case 24000:  return 2;
    case 48000:  return 3;
    case 96000:  return 4;
    case 192000: return 5;
    case 8000:   return 7;
    case 16000:  return 8;
    case 32000:  return 9;
    case 64000:  return 10;
    default:
      ESP_LOGW(TAG, "Unsupported sample rate %u, defaulting to 48000", rate);
      return 3;
  }
}

void VBANAudio::microphone_bytes_callback_(const std::vector<uint8_t> &data) {
  if (data.empty())
    return;

  const size_t bytes_per_sample = bits_per_sample_ / 8;
  const size_t total_samples = data.size() / bytes_per_sample;

  if (bits_per_sample_ == 32) {
    // 32-bit frames from ESP32 I2S. For PCM1808, 24-bit audio sits in the
    // upper 24 bits of each int32_t — pass through without shifting so the
    // full resolution is preserved in the VBAN INT32 stream.
    push_samples_(reinterpret_cast<const int32_t *>(data.data()), total_samples);
  } else if (bits_per_sample_ == 16) {
    // Sign-extend 16-bit samples into int32_t for uniform internal storage.
    const int16_t *raw = reinterpret_cast<const int16_t *>(data.data());
    std::vector<int32_t> converted(total_samples);
    for (size_t i = 0; i < total_samples; i++)
      converted[i] = static_cast<int32_t>(raw[i]);
    push_samples_(converted.data(), total_samples);
  } else {
    ESP_LOGW(TAG, "Unsupported bits_per_sample %u, skipping", bits_per_sample_);
  }
}

void VBANAudio::push_samples_(const int32_t *samples, size_t count) {
  AudioPacket pkt;

  while (count >= VBAN_SAMPLES_PER_PACKET) {
    for (size_t i = 0; i < VBAN_SAMPLES_PER_PACKET; i++) {
      // Apply gain in floating point; clamp to int32 range.
      float s = static_cast<float>(samples[i]) * gain_;
      if (s > 2147483647.0f) s = 2147483647.0f;
      if (s < -2147483648.0f) s = -2147483648.0f;
      pkt.samples[i] = static_cast<int32_t>(s);
    }

    if (xQueueSend(sample_queue_, &pkt, 0) != pdTRUE)
      return;

    samples += VBAN_SAMPLES_PER_PACKET;
    count -= VBAN_SAMPLES_PER_PACKET;
  }
}

void VBANAudio::tx_task_(void *arg) {
  static_cast<VBANAudio *>(arg)->tx_loop_();
}

void VBANAudio::tx_loop_() {
  AudioPacket pkt;
  // Size for worst case: header + 128 samples × 4 bytes (INT32)
  uint8_t packet[sizeof(VBANHeader) + VBAN_SAMPLES_PER_PACKET * sizeof(int32_t)];
  uint32_t frame_counter = 0;

  sock_ = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
  if (sock_ < 0) {
    ESP_LOGE(TAG, "Failed to create UDP socket");
    vTaskDelete(nullptr);
    return;
  }

  memset(&dest_addr_, 0, sizeof(dest_addr_));
  dest_addr_.sin_family = AF_INET;
  dest_addr_.sin_addr.s_addr = inet_addr(target_ip_str_.c_str());
  dest_addr_.sin_port = htons(port_);

  ESP_LOGI(TAG, "VBAN dest %s:%u stream=%s fmt=INT%u ch=%u sr=%u",
           target_ip_str_.c_str(), port_, stream_name_.c_str(),
           bits_per_sample_, channels_, sample_rate_);

  const uint8_t vban_format = (bits_per_sample_ == 32) ? VBAN_FORMAT_INT32 : VBAN_FORMAT_INT16;
  const size_t bytes_per_sample = bits_per_sample_ / 8;
  const size_t payload_size = VBAN_SAMPLES_PER_PACKET * bytes_per_sample;
  const size_t packet_size = sizeof(VBANHeader) + payload_size;

  while (true) {
    if (xQueueReceive(sample_queue_, &pkt, portMAX_DELAY) != pdTRUE)
      continue;

    VBANHeader *hdr = reinterpret_cast<VBANHeader *>(packet);
    memcpy(hdr->vban, "VBAN", 4);
    hdr->format_sr     = vban_sr_index_(sample_rate_);
    hdr->format_nbs    = (VBAN_SAMPLES_PER_PACKET / channels_) - 1;
    hdr->format_nbc    = channels_ - 1;
    hdr->format_format = vban_format;
    memset(hdr->streamname, 0, sizeof(hdr->streamname));
    memcpy(hdr->streamname, stream_name_.c_str(), stream_name_.size());
    hdr->frame_counter = frame_counter++;

    uint8_t *payload = packet + sizeof(VBANHeader);
    if (bits_per_sample_ == 16) {
      // Downconvert from internal int32_t storage to int16_t for the wire.
      int16_t *out = reinterpret_cast<int16_t *>(payload);
      for (size_t i = 0; i < VBAN_SAMPLES_PER_PACKET; i++)
        out[i] = static_cast<int16_t>(pkt.samples[i]);
    } else {
      memcpy(payload, pkt.samples, payload_size);
    }

    sendto(sock_, packet, packet_size, 0,
           reinterpret_cast<sockaddr *>(&dest_addr_), sizeof(dest_addr_));
  }
}

}  // namespace vban_audio
}  // namespace esphome
