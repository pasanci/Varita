
// Minimal MQTT sender using raw WiFiClient (no PubSubClient)
#include <ESP8266WiFi.h>

const char* mqtt_server = "192.168.0.151"; // Home Assistant IP
const uint16_t mqtt_port = 1883;
const char* mqtt_user = "mqtt_frigate"; // Set your MQTT username
const char* mqtt_pass = "3004HomeAssistant"; // Set your MQTT password

void setup_mqtt() {
  // Nothing needed
}

// Helper to encode MQTT string (length + data)
void mqtt_write_string(WiFiClient& client, const char* str) {
  uint8_t len = strlen(str);
  client.write(len);
  client.write((const uint8_t*)str, len);
}


// Helper to encode MQTT variable length (per spec)
int mqtt_encode_varlen(uint8_t* buf, int len) {
  int i = 0;
  do {
    uint8_t encoded = len % 128;
    len /= 128;
    if (len > 0) encoded |= 0x80;
    buf[i++] = encoded;
  } while (len > 0);
  return i;
}

// Send a standards-compliant MQTT CONNECT packet
bool mqtt_connect(WiFiClient& client, const char* client_id) {
  if (!client.connect(mqtt_server, mqtt_port)) return false;
  uint8_t packet[256];
  int i = 0;
  packet[i++] = 0x10; // CONNECT
  // Build variable header and payload
  int vh_start = i;
  // Protocol Name
  packet[i++] = 0; packet[i++] = 4; packet[i++] = 'M'; packet[i++] = 'Q'; packet[i++] = 'T'; packet[i++] = 'T';
  packet[i++] = 4; // Protocol Level 4
  // Connect Flags: Clean Session + Username + Password
  uint8_t connect_flags = 0x02;
  if (strlen(mqtt_user) > 0) connect_flags |= 0x80; // Username flag
  if (strlen(mqtt_pass) > 0) connect_flags |= 0x40; // Password flag
  packet[i++] = connect_flags;
  packet[i++] = 0; packet[i++] = 60; // Keep Alive 60s
  // Client ID
  uint8_t id_len = strlen(client_id);
  packet[i++] = 0; packet[i++] = id_len;
  memcpy(&packet[i], client_id, id_len); i += id_len;
  // Username
  if (strlen(mqtt_user) > 0) {
    uint8_t ulen = strlen(mqtt_user);
    packet[i++] = 0; packet[i++] = ulen;
    memcpy(&packet[i], mqtt_user, ulen); i += ulen;
  }
  // Password
  if (strlen(mqtt_pass) > 0) {
    uint8_t plen = strlen(mqtt_pass);
    packet[i++] = 0; packet[i++] = plen;
    memcpy(&packet[i], mqtt_pass, plen); i += plen;
  }
  // Now encode remaining length
  int vh_len = i - vh_start;
  uint8_t varlen[4];
  int varlen_len = mqtt_encode_varlen(varlen, vh_len);
  // Shift packet right to make room for varlen
  memmove(&packet[1 + varlen_len], &packet[1], vh_len);
  memcpy(&packet[1], varlen, varlen_len);
  int pkt_len = 1 + varlen_len + vh_len;
  client.write(packet, pkt_len);
  // Wait for CONNACK
  unsigned long start = millis();
  while (!client.available() && millis() - start < 1000) delay(1);
  if (client.available() < 4) return false;
  uint8_t resp[4];
  client.read(resp, 4);
  return (resp[0] == 0x20 && resp[1] == 0x02 && resp[3] == 0x00);
}

// Send a minimal MQTT PUBLISH packet (QoS 0, no retain)
void mqtt_publish(WiFiClient& client, const char* topic, const char* payload) {
  uint8_t packet[256];
  int i = 0;
  packet[i++] = 0x30; // PUBLISH
  int rem_len = 2 + strlen(topic) + strlen(payload);
  // Encode remaining length (assume < 127)
  packet[i++] = rem_len;
  // Topic
  uint8_t tlen = strlen(topic);
  packet[i++] = 0; packet[i++] = tlen;
  memcpy(&packet[i], topic, tlen); i += tlen;
  // Payload
  memcpy(&packet[i], payload, strlen(payload)); i += strlen(payload);
  client.write(packet, i);
}

void send_mqtt_identified(int ident) {
  if (ident < 0) return; // Don't send for unidentified
  WiFiClient client;
  if (!mqtt_connect(client, "varitaClient")) {
    Serial.println("MQTT connect failed");
    client.stop();
    return;
  }
  char topic[] = "varita/identified";
  char payload[16];
  snprintf(payload, sizeof(payload), "%d", ident);
  mqtt_publish(client, topic, payload);
  delay(50); // Give time to send
  client.stop();
}
