#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/esp32_ble_tracker/esp32_ble_tracker.h"

#include <string>
#include <vector>
#include <functional>

#include "esp_gattc_api.h"
#include "esp_gap_ble_api.h"
#include "esp_bt_defs.h"

namespace esphome {
namespace minirig_ble {

static const char *const TAG = "minirig_ble";

// ISSC Transparent UART service on Minirig 4
static const esp_bt_uuid_t SERVICE_UUID = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {
        // 49535343-fe7d-4ae5-8fa9-9fafd205e455 (little-endian)
        0x55, 0xe4, 0x05, 0xd2, 0xaf, 0x9f, 0xa9, 0x8f,
        0xe5, 0x4a, 0x7d, 0xfe, 0x43, 0x53, 0x53, 0x49
    }}
};

// TX characteristic: 49535343-8841-43f4-a8d4-ecbe34729bb3 — write commands here
static const esp_bt_uuid_t TX_UUID = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {
        0xb3, 0x9b, 0x72, 0x34, 0xbe, 0xec, 0xd4, 0xa8,
        0xf4, 0x43, 0x41, 0x88, 0x43, 0x53, 0x53, 0x49
    }}
};

// RX characteristic: 49535343-1e4d-4bd9-ba61-23c647249616 — notify responses here
static const esp_bt_uuid_t RX_UUID = {
    .len = ESP_UUID_LEN_128,
    .uuid = {.uuid128 = {
        0x16, 0x96, 0x24, 0x47, 0xc6, 0x23, 0x61, 0xba,
        0xd9, 0x4b, 0x4d, 0x1e, 0x43, 0x53, 0x53, 0x49
    }}
};

struct MinirigDevice {
    std::string name;
    esp_bd_addr_t mac;
    sensor::Sensor *battery_sensor{nullptr};
    int v_min{10560};
    int v_max{12582};

    // Runtime state
    bool connected{false};
    uint16_t conn_id{0};
    uint16_t tx_handle{0};
    uint16_t rx_handle{0};
    bool handles_found{false};
    bool awaiting_response{false};
    uint32_t last_poll_ms{0};
};

class MinirigBLEComponent : public PollingComponent,
                             public esp32_ble_tracker::ESPBTDeviceListener {
 public:
    void setup() override;
    void update() override;
    void dump_config() override;

    // Called from __init__.py generated code
    void add_minirig(const std::string &mac_str, const std::string &name,
                     sensor::Sensor *battery_sensor, int v_min, int v_max);

    // ESPBTDeviceListener
    bool parse_device(const esp32_ble_tracker::ESPBTDevice &device) override;

    // GATT callbacks (called from global C callback via component registry)
    void gattc_event_handler(esp_gattc_cb_event_t event,
                             esp_gatt_if_t gattc_if,
                             esp_ble_gattc_cb_param_t *param);

 protected:
    std::vector<MinirigDevice> minirigs_;
    esp_gatt_if_t gattc_if_{ESP_GATT_IF_NONE};

    MinirigDevice *find_by_conn_id(uint16_t conn_id);
    MinirigDevice *find_by_mac(const esp_bd_addr_t &mac);

    void connect_(MinirigDevice &dev);
    void discover_services_(MinirigDevice &dev);
    void send_command_(MinirigDevice &dev, const std::string &cmd);
    void handle_notify_(MinirigDevice &dev, const uint8_t *data, uint16_t len);
    int parse_voltage_(const std::string &response);
    int voltage_to_pct_(int mv, int v_min, int v_max);

    static void parse_mac_(const std::string &mac_str, esp_bd_addr_t &out);
};

}  // namespace minirig_ble
}  // namespace esphome
