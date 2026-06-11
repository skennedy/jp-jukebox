#pragma once

#include "esphome/core/component.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"
#include "esphome/components/esp32_ble/esp32_ble.h"
#include <vector>
#include <string>
#include <esp_gattc_api.h>

namespace esphome {
namespace minirig_ble {

enum MinirigState {
    STATE_IDLE,
    STATE_DISCOVERED,
    STATE_CONNECTING,
    STATE_CONNECTED,
    STATE_POLLING,
    STATE_DISCONNECTING
};

struct MinirigDevice {
    std::string name;
    esp_bd_addr_t mac;
    sensor::Sensor *battery_sensor{nullptr};
    int v_min;
    int v_max;
    
    // State Tracking
    MinirigState state{STATE_IDLE};
    uint16_t conn_id{0};
    uint16_t tx_handle{0};
    bool handles_found{false};
};

class MinirigBLEComponent : public PollingComponent,
                            public esp32_ble_tracker::ESPBTDeviceListener,
                            public esp32_ble::GATTCEventHandler {
public:
    void setup() override;
    void dump_config() override;
    void update() override;
    
    // ESPHome BLE Tracker Hook
    bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override;

    void add_minirig(const std::string &mac_str, const std::string &name,
                     sensor::Sensor *battery_sensor, int v_min, int v_max);

    // GATT Router Event Hook called by ESPHome's central event loop
    void gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param);

protected:
    void parse_mac_(const std::string &mac_str, esp_bd_addr_t &out);
    void connect_(MinirigDevice &dev);
    void send_command_(MinirigDevice &dev, const std::string &cmd);
    void handle_notify_(MinirigDevice &dev, const uint8_t *data, uint16_t len);
    int parse_voltage_(const std::string &response);
    int voltage_to_pct_(int mv, int v_min, int v_max);

    MinirigDevice *find_by_conn_id(uint16_t conn_id);
    MinirigDevice *find_by_mac(const esp_bd_addr_t &mac);

    std::vector<MinirigDevice> minirigs_;
    esp_gatt_if_t gattc_if_{0xFF}; // ESP_GATT_IF_NONE
    static const char *const TAG;
};

} // namespace minirig_ble
} // namespace esphome