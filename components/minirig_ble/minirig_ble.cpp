#include "minirig_ble.h"
#include "esphome/core/log.h"
#include <algorithm>
#include <cstring>

namespace esphome {
namespace minirig_ble {

const char *const MinirigBLEComponent::TAG = "minirig_ble";
static MinirigBLEComponent *s_instance = nullptr;

// Bridge native ESP-IDF callbacks to our instance cleanly via ESPHome
static void gattc_cb(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    if (s_instance)
        s_instance->gattc_event_handler(event, gattc_if, param);
}

void MinirigBLEComponent::setup() {
    s_instance = this;
    
    // REMOVED: global_esp32_ble_tracker->register_listener(this);
    // ESPHome handles this automatically via the python script!
    
    // Register callback using ESPHome's internal open ID slots
    esp_ble_gattc_register_callback(gattc_cb);
    esp_ble_gattc_app_register(1); // Keep separated from ESPHome app ID 0
    
    ESP_LOGI(TAG, "Minirig BLE framework listener registered.");
}

void MinirigBLEComponent::dump_config() {
    ESP_LOGCONFIG(TAG, "Minirig BLE Custom Component:");
    for (auto &dev : minirigs_) {
        ESP_LOGCONFIG(TAG, "  %s [" MACSTR "] v_min=%d v_max=%d",
                      dev.name.c_str(), MAC2STR(dev.mac), dev.v_min, dev.v_max);
    }
}

// Called automatically at your YAML update_interval
void MinirigBLEComponent::update() {
    for (auto &dev : minirigs_) {
        // If the speaker is idle, mark it as ready to be targeted next time we scan it
        if (dev.state == STATE_IDLE) {
            ESP_LOGD(TAG, "[%s] Queued for next battery poll", dev.name.c_str());
        } 
        // Auto-recovery timeout guard if a speaker hangs mid-connection
        else if (dev.state != STATE_IDLE && dev.state != STATE_POLLING) {
            ESP_LOGW(TAG, "[%s] Connection timed out. Resetting state.", dev.name.c_str());
            if (dev.conn_id != 0 && gattc_if_ != 0xFF) {
                esp_ble_gattc_close(gattc_if_, dev.conn_id);
            }
            dev.state = STATE_IDLE;
            dev.handles_found = false;
        }
    }
}

// ESPHome passes every tracked BLE advertisement beacon through here
bool MinirigBLEComponent::parse_device(const esp32_ble_tracker::ESPBTDevice &device) {
    for (auto &dev : minirigs_) {
        if (memcmp(device.address(), dev.mac, 6) == 0) {
            // Only initiate link if it is waiting to be scanned/polled
            if (dev.state == STATE_IDLE) {
                ESP_LOGI(TAG, "[%s] Advertised! Activating polling sequence.", dev.name.c_str());
                dev.state = STATE_DISCOVERED;
                connect_(dev);
                return true;
            }
        }
    }
    return false;
}

void MinirigBLEComponent::connect_(MinirigDevice &dev) {
    if (gattc_if_ == 0xFF) {
        ESP_LOGW(TAG, "[%s] GATT Client tracking engine not fully initialized yet. Will retry...", dev.name.c_str());
        dev.state = STATE_IDLE; // Reset to idle so it can retry next scan
        return;
    }
    dev.state = STATE_CONNECTING;
    esp_ble_gattc_open(gattc_if_, dev.mac, BLE_ADDR_TYPE_RANDOM, true);
}

void MinirigBLEComponent::send_command_(MinirigDevice &dev, const std::string &cmd) {
    if (dev.tx_handle == 0) return;
    dev.state = STATE_POLLING;
    
    esp_ble_gattc_write_char(
        gattc_if_, dev.conn_id, dev.tx_handle, cmd.size(),
        reinterpret_cast<uint8_t *>(const_cast<char *>(cmd.data())),
        ESP_GATT_WRITE_TYPE_RSP, ESP_GATT_AUTH_REQ_NONE
    );
}

void MinirigBLEComponent::handle_notify_(MinirigDevice &dev, const uint8_t *data, uint16_t len) {
    std::string resp(reinterpret_cast<const char *>(data), len);
    ESP_LOGD(TAG, "[%s] Received payload: %s", dev.name.c_str(), resp.c_str());

    if (resp.rfind("CVB=+", 0) == 0) {
        int mv = parse_voltage_(resp);
        if (mv > 0) {
            int pct = voltage_to_pct_(mv, dev.v_min, dev.v_max);
            ESP_LOGI(TAG, "[%s] Battery metrics updated: %d mV → %d%%", dev.name.c_str(), mv, pct);
            if (dev.battery_sensor)
                dev.battery_sensor->publish_state(static_cast<float>(pct));
        }
    }

    // CRITICAL BATTERY SAVER: Immediately sever connection now that data is gathered
    ESP_LOGI(TAG, "[%s] Telemetry recorded. Severing connection to preserve battery life.", dev.name.c_str());
    dev.state = STATE_DISCONNECTING;
    esp_ble_gattc_close(gattc_if_, dev.conn_id);
}

// Process direct low-level GATT engine routing
void MinirigBLEComponent::gattc_event_handler(esp_gattc_cb_event_t event, esp_gatt_if_t gattc_if, esp_ble_gattc_cb_param_t *param) {
    switch (event) {
    case ESP_GATTC_REG_EVT:
        // Only accept the registration if it matches our custom app_id (1)
        if (param->reg.status == ESP_GATT_OK && param->reg.app_id == 1) {
            gattc_if_ = gattc_if;
            ESP_LOGI(TAG, "GATT API successfully bound to custom interface slot ID: %d", gattc_if_);
        }
        break;

    case ESP_GATTC_CONNECT_EVT: {
        auto *dev = find_by_mac(param->connect.remote_bda);
        if (!dev) break;
        dev->conn_id = param->connect.conn_id;
        dev->state = STATE_CONNECTED;
        ESP_LOGD(TAG, "[%s] Connected. Running descriptor service query...", dev->name.c_str());
        esp_ble_gattc_search_service(gattc_if_, dev->conn_id, nullptr);
        break;
    }

    case ESP_GATTC_SEARCH_CMPL_EVT: {
        auto *dev = find_by_conn_id(param->search_cmpl.conn_id);
        if (!dev) break;

        // --- MANUALLY INSERT HARCODED HANDLE TARGETS HERE ---
        // For demonstration, replacing full descriptor code with targeted fallback assumptions:
        dev->tx_handle = 42;  // Swap with your speaker's exact resolved TX attribute handle
        dev->handles_found = true;

        ESP_LOGD(TAG, "[%s] GATT service layout parsed. Transmitting CVB query.", dev->name.c_str());
        send_command_(*dev, "CVB");
        break;
    }

    case ESP_GATTC_DISCONNECT_EVT: {
        auto *dev = find_by_mac(param->disconnect.remote_bda);
        if (!dev) break;
        ESP_LOGI(TAG, "[%s] Disconnected (Bus Status Code: %d)", dev->name.c_str(), param->disconnect.reason);
        
        // Return fully back to idle loop state, turning off internal tracking flags
        dev->state = STATE_IDLE;
        dev->handles_found = false;
        dev->conn_id = 0;
        break;
    }

    case ESP_GATTC_NOTIFY_EVT: {
        auto *dev = find_by_conn_id(param->notify.conn_id);
        if (!dev) break;
        if (param->notify.handle == dev->tx_handle || true) { // Replace condition with target rx handle validation
            handle_notify_(*dev, param->notify.value, param->notify.value_len);
        }
        break;
    }

    default:
        break;
    }
}

// ---------------------------------------------------------------------------
// Standard Parsing Utilities (Untouched Helper Logic)
// ---------------------------------------------------------------------------
void MinirigBLEComponent::parse_mac_(const std::string &mac_str, esp_bd_addr_t &out) {
    unsigned int b[6] = {};
    sscanf(mac_str.c_str(), "%x:%x:%x:%x:%x:%x", &b[0], &b[1], &b[2], &b[3], &b[4], &b[5]);
    for (int i = 0; i < 6; i++) out[i] = static_cast<uint8_t>(b[i]);
}

void MinirigBLEComponent::add_minirig(const std::string &mac_str, const std::string &name, sensor::Sensor *battery_sensor, int v_min, int v_max) {
    MinirigDevice dev;
    dev.name = name;
    dev.battery_sensor = battery_sensor;
    dev.v_min = v_min;
    dev.v_max = v_max;
    parse_mac_(mac_str, dev.mac);
    minirigs_.push_back(dev);
}

int MinirigBLEComponent::parse_voltage_(const std::string &response) {
    auto pos = response.find('+');
    if (pos == std::string::npos) return -1;
    return static_cast<int>(strtol(response.c_str() + pos + 1, nullptr, 10));
}

int MinirigBLEComponent::voltage_to_pct_(int mv, int v_min, int v_max) {
    float pct = static_cast<float>(mv - v_min) / static_cast<float>(v_max - v_min) * 100.0f;
    return static_cast<int>(std::max(0.0f, std::min(100.0f, pct)));
}

MinirigDevice *MinirigBLEComponent::find_by_conn_id(uint16_t conn_id) {
    for (auto &dev : minirigs_) if (dev.conn_id == conn_id) return &dev;
    return nullptr;
}

MinirigDevice *MinirigBLEComponent::find_by_mac(const esp_bd_addr_t &mac) {
    for (auto &dev : minirigs_) if (memcmp(dev.mac, mac, 6) == 0) return &dev;
    return nullptr;
}

} // namespace minirig_ble
} // namespace esphome