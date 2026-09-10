#if !defined(ARDUINO_ARCH_ESP32)
  #error "This program requires an ESP32S3"
#endif

#include <Arduino.h>
#include <HardwareSerial.h>
#include <NimBLEDevice.h>
#include <vector>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <nvs_flash.h>
#include "opendroneid.h"
#include "odid_wifi.h"
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

const int SERIAL1_RX_PIN = 6;
const int SERIAL1_TX_PIN = 5;

struct id_data {
  uint8_t  mac[6];
  int      rssi;
  uint32_t last_seen;
  char     op_id[ODID_ID_SIZE + 1];
  char     uav_id[ODID_ID_SIZE + 1];
  double   lat_d;
  double   long_d;
  double   base_lat_d;
  double   base_long_d;
  int      altitude_msl;
  int      height_agl;
  int      speed;
  int      heading;
  int      flag;
};

void callback(void *, wifi_promiscuous_pkt_type_t);
void send_json_fast(const id_data *UAV);
void print_compact_message(const id_data *UAV);
static bool id_nonempty(const char *s);
static void copy_odid_text(char *dst, const char *src);
static void format_mac(char *mac_str, const uint8_t *mac);
static void copy_basic_id_from_uas(char *dst, const ODID_UAS_Data *uas);
static void apply_uas_to_stored(id_data *stored, const ODID_UAS_Data *uas,
                                const uint8_t *mac, int rssi);
static void enqueue_if_located(id_data *stored);
static void apply_ble_odid_msg(id_data *UAV, const uint8_t *odid);
static void handle_ble_payload(const uint8_t *payload, int length,
                               const uint8_t *mac, int rssi);

#define MAX_UAVS 8
id_data uavs[MAX_UAVS] = {0};
ODID_UAS_Data UAS_data;
unsigned long last_status = 0;

static QueueHandle_t printQueue;

id_data* next_uav(uint8_t* mac) {
  for (int i = 0; i < MAX_UAVS; i++) {
    if (memcmp(uavs[i].mac, mac, 6) == 0)
      return &uavs[i];
  }
  for (int i = 0; i < MAX_UAVS; i++) {
    if (uavs[i].mac[0] == 0)
      return &uavs[i];
  }
  return &uavs[0];
}

static bool id_nonempty(const char *s) {
  if (!s) return false;
  while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;
  return *s != '\0';
}

static void copy_odid_text(char *dst, const char *src) {
  strncpy(dst, src, ODID_ID_SIZE);
  dst[ODID_ID_SIZE] = '\0';
}

static void format_mac(char *mac_str, const uint8_t *mac) {
  snprintf(mac_str, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

static void copy_basic_id_from_uas(char *dst, const ODID_UAS_Data *uas) {
  dst[0] = '\0';
  for (int i = 0; i < ODID_BASIC_ID_MAX_MESSAGES; i++) {
    if (uas->BasicIDValid[i] && id_nonempty(uas->BasicID[i].UASID)) {
      copy_odid_text(dst, uas->BasicID[i].UASID);
      return;
    }
  }
}

// Merge one decoded pack into the MAC-keyed slot. Location-only packs must
// not wipe a previously cached BasicID / OperatorID (RidReader rejects empty id).
static void apply_uas_to_stored(id_data *stored, const ODID_UAS_Data *uas,
                                const uint8_t *mac, int rssi) {
  memcpy(stored->mac, mac, 6);
  stored->rssi = rssi;
  stored->last_seen = millis();
  stored->flag = 1;

  char incoming_id[ODID_ID_SIZE + 1];
  copy_basic_id_from_uas(incoming_id, uas);
  if (id_nonempty(incoming_id)) {
    copy_odid_text(stored->uav_id, incoming_id);
  }

  if (uas->LocationValid) {
    stored->lat_d = uas->Location.Latitude;
    stored->long_d = uas->Location.Longitude;
    stored->altitude_msl = (int)uas->Location.AltitudeGeo;
    stored->height_agl = (int)uas->Location.Height;
    stored->speed = (int)uas->Location.SpeedHorizontal;
    stored->heading = (int)uas->Location.Direction;
  }
  if (uas->SystemValid) {
    stored->base_lat_d = uas->System.OperatorLatitude;
    stored->base_long_d = uas->System.OperatorLongitude;
  }
  if (uas->OperatorIDValid && id_nonempty(uas->OperatorID.OperatorId)) {
    copy_odid_text(stored->op_id, uas->OperatorID.OperatorId);
  }
}

static void enqueue_if_located(id_data *stored) {
  if (stored->lat_d == 0.0 && stored->long_d == 0.0) return;
  id_data tmp = *stored;
  // BLE + WiFi callbacks run in tasks, not an ISR. FromISR here starved the
  // scheduler and is the wrong API for NimBLE host / WiFi RX tasks.
  xQueueSend(printQueue, &tmp, 0);
}

static void apply_ble_odid_msg(id_data *UAV, const uint8_t *odid) {
  switch (odid[0] & 0xF0) {
    case 0x00: {
      ODID_BasicID_data basic;
      decodeBasicIDMessage(&basic, (ODID_BasicID_encoded *)odid);
      if (id_nonempty(basic.UASID)) {
        copy_odid_text(UAV->uav_id, basic.UASID);
      }
      break;
    }
    case 0x10: {
      ODID_Location_data loc;
      decodeLocationMessage(&loc, (ODID_Location_encoded *)odid);
      UAV->lat_d = loc.Latitude;
      UAV->long_d = loc.Longitude;
      UAV->altitude_msl = (int)loc.AltitudeGeo;
      UAV->height_agl = (int)loc.Height;
      UAV->speed = (int)loc.SpeedHorizontal;
      UAV->heading = (int)loc.Direction;
      break;
    }
    case 0x40: {
      ODID_System_data sys;
      decodeSystemMessage(&sys, (ODID_System_encoded *)odid);
      UAV->base_lat_d = sys.OperatorLatitude;
      UAV->base_long_d = sys.OperatorLongitude;
      break;
    }
    case 0x50: {
      ODID_OperatorID_data op;
      decodeOperatorIDMessage(&op, (ODID_OperatorID_encoded *)odid);
      if (id_nonempty(op.OperatorId)) {
        copy_odid_text(UAV->op_id, op.OperatorId);
      }
      break;
    }
    default:
      break;
  }
}

// ASTM Remote ID BLE: AD type 0x16, UUID 0xFFFA, app code 0x0D, then counter + 25-byte msg.
// Walk every AD structure — Dronetag often prepends flags so the old payload[1]==0x16 check misses.
static void handle_ble_payload(const uint8_t *payload, int length,
                               const uint8_t *mac, int rssi) {
  if (!payload || length <= 5 || !mac) return;
  uint8_t mac_buf[6];
  memcpy(mac_buf, mac, 6);
  id_data *UAV = next_uav(mac_buf);
  UAV->last_seen = millis();
  UAV->rssi = rssi;
  memcpy(UAV->mac, mac_buf, 6);

  bool decoded = false;
  int offset = 0;
  while (offset + 5 < length) {
    int ad_len = payload[offset];
    if (ad_len < 1 || offset + 1 + ad_len > length) break;
    if (payload[offset + 1] == 0x16 && ad_len >= 6 &&
        payload[offset + 2] == 0xFA && payload[offset + 3] == 0xFF &&
        payload[offset + 4] == 0x0D) {
      const uint8_t *odid = &payload[offset + 6];
      int odid_len = ad_len - 5;
      if (odid_len >= 25 && (odid[0] & 0xF0) == 0xF0) {
        memset(&UAS_data, 0, sizeof(UAS_data));
        if (odid_message_process_pack(&UAS_data, (uint8_t *)odid, (size_t)odid_len) >= 0) {
          apply_uas_to_stored(UAV, &UAS_data, mac_buf, rssi);
          decoded = true;
        }
      } else if (odid_len >= 1) {
        apply_ble_odid_msg(UAV, odid);
        decoded = true;
      }
    }
    offset += ad_len + 1;
  }
  if (decoded) {
    UAV->flag = 1;
    enqueue_if_located(UAV);
  }
}

class RidBleScanCallbacks : public NimBLEScanCallbacks {
public:
  void onResult(const NimBLEAdvertisedDevice *advertisedDevice) override {
    if (!advertisedDevice) return;
    const std::vector<uint8_t> &payload = advertisedDevice->getPayload();
    const uint8_t *mac = advertisedDevice->getAddress().getVal();
    handle_ble_payload(payload.data(), (int)payload.size(), mac,
                       advertisedDevice->getRSSI());
  }

  void onScanEnd(const NimBLEScanResults &results, int reason) override {
    (void)results;
    (void)reason;
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan) scan->start(0, false, true);
  }
};

static RidBleScanCallbacks bleScanCallbacks;

void send_json_fast(const id_data *UAV) {
  // Netra pylon RidReader (newline JSON): type, id, lat, lon, alt_msl, pilot_*.
  // Extra mac/rssi fields are ignored by RidReader and must not replace required keys.
  // validate_droneid rejects empty serial and lat==lon==0.
  if (UAV->lat_d == 0.0 && UAV->long_d == 0.0) return;
  char mac_str[18];
  format_mac(mac_str, UAV->mac);
  const char *id = id_nonempty(UAV->uav_id) ? UAV->uav_id : mac_str;
  char json_msg[320];
  snprintf(json_msg, sizeof(json_msg),
    R"({"type":"detection","id":"%s","lat":%.6f,"lon":%.6f,"alt_msl":%d,"pilot_lat":%.6f,"pilot_lon":%.6f,"mac":"%s","rssi":%d})",
    id, UAV->lat_d, UAV->long_d, UAV->altitude_msl,
    UAV->base_lat_d, UAV->base_long_d, mac_str, UAV->rssi);
  Serial.println(json_msg);
}

void print_compact_message(const id_data *UAV) {
  static unsigned long lastSendTime = 0;
  const unsigned long sendInterval = 5000;
  const int MAX_MESH_SIZE = 230;
  
  if (millis() - lastSendTime < sendInterval) return;
  lastSendTime = millis();
  
  char mac_str[18];
  snprintf(mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x",
           UAV->mac[0], UAV->mac[1], UAV->mac[2],
           UAV->mac[3], UAV->mac[4], UAV->mac[5]);
  
  char mesh_msg[MAX_MESH_SIZE];
  int msg_len = 0;
  msg_len += snprintf(mesh_msg + msg_len, sizeof(mesh_msg) - msg_len,
                      "Drone: %s RSSI:%d", mac_str, UAV->rssi);
  if (msg_len < MAX_MESH_SIZE && UAV->lat_d != 0.0 && UAV->long_d != 0.0) {
    msg_len += snprintf(mesh_msg + msg_len, sizeof(mesh_msg) - msg_len,
                        " https://maps.google.com/?q=%.6f,%.6f",
                        UAV->lat_d, UAV->long_d, UAV->uav_id);
  }
  if (Serial1.availableForWrite() >= msg_len) {
    Serial1.println(mesh_msg);
  }
  
  delay(1000);
  if (UAV->base_lat_d != 0.0 && UAV->base_long_d != 0.0) {
    char pilot_msg[MAX_MESH_SIZE];
    int pilot_len = snprintf(pilot_msg, sizeof(pilot_msg),
                             "Pilot: https://maps.google.com/?q=%.6f,%.6f",
                             UAV->base_lat_d, UAV->base_long_d);
    if (Serial1.availableForWrite() >= pilot_len) {
      Serial1.println(pilot_msg);
    }
  }
}

void bleScanTask(void *parameter) {
  for (;;) {
    NimBLEScan *scan = NimBLEDevice::getScan();
    if (scan && !scan->isScanning()) {
      scan->start(0, false, true);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void wifiProcessTask(void *parameter) {
  for (;;) {
    // No-op: callback sets uavs[].flag and data, so nothing needed here
    delay(10);
  }
}

void callback(void *buffer, wifi_promiscuous_pkt_type_t type) {
  if (type != WIFI_PKT_MGMT) return;
  
  wifi_promiscuous_pkt_t *packet = (wifi_promiscuous_pkt_t *)buffer;
  uint8_t *payload = packet->payload;
  int length = packet->rx_ctrl.sig_len;
  
  static const uint8_t nan_dest[6] = {0x51, 0x6f, 0x9a, 0x01, 0x00, 0x00};
  if (memcmp(nan_dest, &payload[4], 6) == 0) {
    char nan_mac[6];
    if (odid_wifi_receive_message_pack_nan_action_frame(&UAS_data, nan_mac, payload, length) == 0) {
      uint8_t mac[6];
      memcpy(mac, &payload[10], 6);
      id_data *storedUAV = next_uav(mac);
      apply_uas_to_stored(storedUAV, &UAS_data, mac, packet->rx_ctrl.rssi);
      enqueue_if_located(storedUAV);
    }
  }
  else if (payload[0] == 0x80) {
    int offset = 36;
    while (offset < length) {
      int typ = payload[offset];
      int len = payload[offset + 1];
      if ((typ == 0xdd) &&
          (((payload[offset + 2] == 0x90 && payload[offset + 3] == 0x3a && payload[offset + 4] == 0xe6)) ||
           ((payload[offset + 2] == 0xfa && payload[offset + 3] == 0x0b && payload[offset + 4] == 0xbc)))) {
        int j = offset + 7;
        if (j < length) {
          memset(&UAS_data, 0, sizeof(UAS_data));
          odid_message_process_pack(&UAS_data, &payload[j], length - j);
          uint8_t mac[6];
          memcpy(mac, &payload[10], 6);
          id_data *storedUAV = next_uav(mac);
          apply_uas_to_stored(storedUAV, &UAS_data, mac, packet->rx_ctrl.rssi);
          enqueue_if_located(storedUAV);
        }
      }
      offset += len + 2;
    }
  }
}

void printerTask(void *param) {
  id_data UAV;
  for (;;) {
    if (xQueueReceive(printQueue, &UAV, portMAX_DELAY)) {
      send_json_fast(&UAV);
      print_compact_message(&UAV);
      // no need to reset flag on copy
    }
  }
}

void initializeSerial() {
  Serial.begin(115200);
  Serial1.begin(115200, SERIAL_8N1, SERIAL1_RX_PIN, SERIAL1_TX_PIN);
}

void setup() {
  setCpuFrequencyMhz(160);
  initializeSerial();
  nvs_flash_init();
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_rx_cb(&callback);
  esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);
  
  // NimBLE: callback-only, no result store, passive, duplicates on.
  // Bluedroid BLEAdvertisedDevice::parseAdvertisement malloc'd every ADV and
  // OOMed (`Failed to alloc`) under a nearby Dronetag + WiFi promiscuous.
  NimBLEDevice::init("");
  NimBLEScan *scan = NimBLEDevice::getScan();
  scan->setScanCallbacks(&bleScanCallbacks, true);
  scan->setActiveScan(false);
  scan->setInterval(96);
  scan->setWindow(96);
  scan->setMaxResults(0);
  scan->setDuplicateFilter(false);
  scan->start(0, false, true);

  printQueue = xQueueCreate(MAX_UAVS, sizeof(id_data));
  
  xTaskCreatePinnedToCore(bleScanTask, "BLEScanTask", 10000, NULL, 1, NULL, 1);
  xTaskCreatePinnedToCore(wifiProcessTask, "WiFiProcessTask", 10000, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(printerTask, "PrinterTask", 10000, NULL, 1, NULL, 1);
  
  memset(uavs, 0, sizeof(uavs));
}

void loop() {
  unsigned long current_millis = millis();
    if ((current_millis - last_status) > 60000UL) {
      Serial.println("   [+] Device is active and scanning...");
      last_status = current_millis;
    }
}
