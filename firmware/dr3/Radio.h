#pragma once
// ESP-NOW link: the remote sends control frames to the drone.
// Frame: format marker, project ID, sequence number and control data.
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

struct __attribute__((packed)) Frame {
  uint32_t magic;
  uint8_t project[8];
  uint32_t sequence;
  ControlData control;
};
struct RadioInbox { Frame frame; uint8_t source[6]; uint32_t arrival; };

QueueHandle_t radioInbox = nullptr;
uint8_t ownMac[6]{};
bool radioOK = false;

bool sameMac(const uint8_t *a, const uint8_t *b) { return memcmp(a, b, 6) == 0; }
// "a is newer than b", also across 32-bit counter wrap-around.
bool newerSequence(uint32_t a, uint32_t b) { uint32_t d = a - b; return d != 0 && d < 0x80000000; }

// Called by the Wi-Fi driver: only copies the frame into the queue.
// The frame is checked later, in loop().
void onRadioReceive(const esp_now_recv_info_t *info, const uint8_t *data, int n) {
  if (!info || !data || !radioInbox || n != sizeof(Frame) || !sameMac(info->des_addr, ownMac)) return;
  RadioInbox item{};
  memcpy(&item.frame, data, n);
  memcpy(item.source, info->src_addr, 6);
  item.arrival = millis();
  xQueueSend(radioInbox, &item, 0);
}

bool startRadio() {
  radioInbox = xQueueCreate(8, sizeof(RadioInbox));
  if (!radioInbox || !WiFi.mode(WIFI_STA)) return false;
  WiFi.disconnect(false, false);
  return esp_wifi_set_ps(WIFI_PS_NONE) == ESP_OK &&
         esp_wifi_set_channel(RADIO_CHANNEL, WIFI_SECOND_CHAN_NONE) == ESP_OK &&
         esp_wifi_get_mac(WIFI_IF_STA, ownMac) == ESP_OK &&
         esp_now_init() == ESP_OK &&
         esp_now_register_recv_cb(onRadioReceive) == ESP_OK;
}

bool addPeer(const uint8_t *mac) {
  esp_now_peer_info_t peer{};
  memcpy(peer.peer_addr, mac, 6);
  peer.channel = RADIO_CHANNEL;
  peer.ifidx = WIFI_IF_STA;
  return esp_now_add_peer(&peer) == ESP_OK;
}
