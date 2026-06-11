#include "minirig_ble.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

#include <sstream>
#include <iomanip>
#include <algorithm>
#include <cstring>

namespace esphome {
namespace minirig_ble {

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

static MinirigBLEComponent *s_instance = nullptr;

static void gattc_cb(esp_gattc_cb_event_t event,
                     esp_gatt_if_t gattc_if,
                     esp_ble_gattc_cb_param_t *param) {
    if (s_instance)
        s_instance->gattc_event_handler(event, gattc_if, param);
}

void MinirigBLEComponent::parse_mac_(const std::string &mac_str, esp_bd_addr_t &out) {
    // Accepts "aa:bb:cc:dd:ee:ff" format
    unsigned int b[6] = {};
    sscanf(mac_str.c_str(), "%x:%x:%x:%x:%x:%x",
           &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
    for (int i = 0; i < 6; i++)
        out[i] = static_cast<uint8_t>(b[i]);
}

// ---------------------------------------------------------------------------
// Component lifecycle
// ---------------------------------------------------------------------------

void MinirigBLEComponent::setup() {
    s_instance = this;
    esp_ble_gattc_register_callback(gattc_cb);
    esp_ble_gattc_app_register(0);
    ESP_LOGI(TAG, "minirig_ble setup — %d device(s) configured", (int)minirigs_.size());
}

void MinirigBLEComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Minirig BLE:");
    for (auto &dev : minirigs_) {
        ESP_LOGCONFIG(TAG, "  %s  " MACSTR "  v_min=%d v_max=%d",
                      dev.name.c_str(), MAC2STR(dev.mac), dev.v_min, dev.v_max);
    }
}

void MinirigBLEComponent::add_minirig(const std::string &mac_str,
                                       const std::string &name,
                                       sensor::Sensor *battery_sensor,
                                       int v_min, int v_max) {
    MinirigDevice dev;
    dev.name = name;
    dev.battery_sensor = battery_sensor;
    dev.v_min = v_min;
    dev.v_max = v_max;
    parse_mac_(mac_str, dev.mac);
    minirigs_.push_back(dev);
}

// ---------------------------------------------------------------------------
// Polling — called every update_interval (default 5 min)
// ---------------------------------------------------------------------------

void MinirigBLEComponent::update() {
    for (auto &dev : minirigs_) {
        if (dev.connected && dev.handles_found) {
            ESP_LOGD(TAG, "[%s] polling CVB", dev.name.c_str());
            send_command_(dev, "CVB");
        } else if (!dev.connected) {
            ESP_LOGD(TAG, "[%s] not connected, attempting connect", dev.name.c_str());
            connect_(dev);
        }
    }
}

// ---------------------------------------------------------------------------
// BLE scanner — spot our devices when they advertise
// ---------------------------------------------------------------------------

bool MinirigBLEComponent::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
    for (auto &dev : minirigs_) {
        if (memcmp(device.address(), dev.mac, 6) == 0 && !dev.connected) {
            ESP_LOGI(TAG, "[%s] found in scan, connecting", dev.name.c_str());
            connect_(dev);
            return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// BLE connect
// ---------------------------------------------------------------------------

void MinirigBLEComponent::connect_(MinirigDevice &dev) {
    if (gattc_if_ == ESP_GATT_IF_NONE) {
        ESP_LOGW(TAG, "[%s] gattc not ready yet", dev.name.c_str());
        return;
    }
    esp_ble_gattc_open(gattc_if_, dev.mac, BLE_ADDR_TYPE_RANDOM, true);
}

// ---------------------------------------------------------------------------
// Send command
// ---------------------------------------------------------------------------

void MinirigBLEComponent::send_command_(MinirigDevice &dev, const std::string &cmd) {
    if (!dev.connected || dev.tx_handle == 0) return;
    dev.awaiting_response = true;
    esp_ble_gattc_write_char(
        gattc_if_,
        dev.conn_id,
        dev.tx_handle,
        cmd.size(),
        reinterpret_cast<uint8_t *>(const_cast<char *>(cmd.data())),
        ESP_GATT_WRITE_TYPE_RSP,
        ESP_GATT_AUTH_REQ_NONE
    );
}

// ---------------------------------------------------------------------------
// Parse notify response
// ---------------------------------------------------------------------------

void MinirigBLEComponent::handle_notify_(MinirigDevice &dev,
                                          const uint8_t *data, uint16_t len) {
    std::string resp(reinterpret_cast<const char *>(data), len);
    ESP_LOGD(TAG, "[%s] notify: %s", dev.name.c_str(), resp.c_str());
    dev.awaiting_response = false;

    if (resp.rfind("CVB=+", 0) == 0) {
        int mv = parse_voltage_(resp);
        if (mv > 0) {
            int pct = voltage_to_pct_(mv, dev.v_min, dev.v_max);
            ESP_LOGI(TAG, "[%s] battery %d mV → %d%%", dev.name.c_str(), mv, pct);
            if (dev.battery_sensor)
                dev.battery_sensor->publish_state(static_cast<float>(pct));
        }
    }
}

int MinirigBLEComponent::parse_voltage_(const std::string &response) {
    // "CVB=+11773" → 11773
    auto pos = response.find('+');
    if (pos == std::string::npos) return -1;
    const char *start = response.c_str() + pos + 1;
    char *end = nullptr;
    long val = strtol(start, &end, 10);
    if (end == start) return -1;  // no digits parsed
    return static_cast<int>(val);
}

int MinirigBLEComponent::voltage_to_pct_(int mv, int v_min, int v_max) {
    float pct = static_cast<float>(mv - v_min) / static_cast<float>(v_max - v_min) * 100.0f;
    return static_cast<int>(std::max(0.0f, std::min(100.0f, pct)));
}

// ---------------------------------------------------------------------------
// Lookup helpers
// ---------------------------------------------------------------------------

MinirigDevice *MinirigBLEComponent::find_by_conn_id(uint16_t conn_id) {
    for (auto &dev : minirigs_)
        if (dev.connected && dev.conn_id == conn_id) return &dev;
    return nullptr;
}

MinirigDevice *MinirigBLEComponent::find_by_mac(const esp_bd_addr_t &mac) {
    for (auto &dev : minirigs_)
        if (memcmp(dev.mac, mac, 6) == 0) return &dev;
    return nullptr;
}

// ---------------------------------------------------------------------------
// GATT event handler
// ---------------------------------------------------------------------------

void MinirigBLEComponent::gattc_event_handler(esp_gattc_cb_event_t event,
                                               esp_gatt_if_t gattc_if,
                                               esp_ble_gattc_cb_param_t *param) {
    switch (event) {

    case ESP_GATTC_REG_EVT:
        if (param->reg.status == ESP_GATT_OK) {
            gattc_if_ = gattc_if;
            ESP_LOGD(TAG, "GATTC registered, if=%d", gattc_if_);
        }
        break;

    case ESP_GATTC_CONNECT_EVT: {
        auto *dev = find_by_mac(param->connect.remote_bda);
        if (!dev) break;
        dev->connected = true;
        dev->conn_id = param->connect.conn_id;
        ESP_LOGI(TAG, "[%s] connected, discovering services", dev->name.c_str());
        esp_ble_gattc_search_service(gattc_if_, dev->conn_id, nullptr);
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
        auto *dev = find_by_conn_id(param->disconnect.conn_id);
        if (!dev) break;
        ESP_LOGW(TAG, "[%s] disconnected (reason %d)", dev->name.c_str(),
                 param->disconnect.reason);
        dev->connected = false;
        dev->handles_found = false;
        dev->tx_handle = 0;
        dev->rx_handle = 0;
        break;
    }

    case ESP_GATTC_SEARCH_RES_EVT: {
        // We find the service — characteristics resolved next in SEARCH_CMPL
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        auto *dev = find_by_conn_id(param->search_cmpl.conn_id);
        if (!dev) break;

        // Get all characteristics in the ISSC UART service
        esp_gattc_char_elem_t chars[10];
        uint16_t count = 10;
        esp_gatt_status_t status = esp_ble_gattc_get_char_by_uuid(
            gattc_if_,
            dev->conn_id,
            0x0001, 0xffff,
            TX_UUID,
            chars, &count
        );
        if (status == ESP_GATT_OK && count > 0)
            dev->tx_handle = chars[0].char_handle;

        count = 10;
        status = esp_ble_gattc_get_char_by_uuid(
            gattc_if_,
            dev->conn_id,
            0x0001, 0xffff,
            RX_UUID,
            chars, &count
        );
        if (status == ESP_GATT_OK && count > 0) {
            dev->rx_handle = chars[0].char_handle;

            // Enable notifications on RX
            esp_gattc_descr_elem_t descr[1];
            uint16_t descr_count = 1;
            esp_bt_uuid_t notify_uuid = {.len = ESP_UUID_LEN_16,
                                         .uuid = {.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG}};
            if (esp_ble_gattc_get_descr_by_char_handle(
                    gattc_if_, dev->conn_id, dev->rx_handle,
                    notify_uuid, descr, &descr_count) == ESP_GATT_OK && descr_count > 0) {
                uint16_t notify_en = 1;
                esp_ble_gattc_write_char_descr(
                    gattc_if_, dev->conn_id, descr[0].handle,
                    sizeof(notify_en),
                    reinterpret_cast<uint8_t *>(&notify_en),
                    ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE
                );
            }
        }

        if (dev->tx_handle && dev->rx_handle) {
            dev->handles_found = true;
            ESP_LOGI(TAG, "[%s] handles found tx=0x%x rx=0x%x",
                     dev->name.c_str(), dev->tx_handle, dev->rx_handle);
            // Do an immediate first poll
            send_command_(*dev, "CVB");
        } else {
            ESP_LOGW(TAG, "[%s] could not find expected handles", dev->name.c_str());
        }
        break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
        auto *dev = find_by_conn_id(param->notify.conn_id);
        if (!dev) break;
        if (param->notify.handle == dev->rx_handle)
            handle_notify_(*dev, param->notify.value, param->notify.value_len);
        break;
    }

    case ESP_GATTC_WRITE_CHAR_EVT:
        // Write confirmed — nothing to do
        break;

    default:
        break;
    }
}

}  // namespace minirig_ble
}  // namespace esphome
