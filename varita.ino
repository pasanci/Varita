#include <Wire.h>
// Modified for XIAO nRF52840 Sense Plus: no ESP8266 WiFi
// If you need network features on the XIAO use BLE or add an external
// radio/bridge. The ESP8266 WiFi include is intentionally disabled.
// #include <ESP8266WiFi.h>
// #include "mqtt_helpers.h"

//Base resistor: Rb = 1 kΩ (3.3V)
//S8050 E B C pinout: https://www.eleccircuit.com/wp-content/uploads/2018/05/S8050-Transistor-Pinout.png
//LED series resistor Red 68 Ω
//LED series resistor Green 47 Ω
//LED series resistor Blue 3.3 Ω
//Wiring (one transistor per color): emitter → GND, collector → LED cathode, LED anode → Vcc through its series resistor, base → MCU pin via base resistor.


// MPU6050 Slave Device Address
// Use the XIAO's onboard IMU. Prefer the LSM6DS3 library header.
#include <LSM6DS3.h>
LSM6DS3 myIMU(I2C_MODE, 0x6A);    //I2C device address 0x6A

// On XIAO the default I2C pins are used; no manual SDA/SCL selection here.

// --- RGB LED configuration (change pins if your wiring differs) ---
// Recommended: use series resistors per channel (see notes below)
// Update these to match how you wired the RGB LED to the XIAO.
// Example defaults (change if needed):
const uint8_t PIN_R = 0; // red channel pin
const uint8_t PIN_G = 2; // green channel pin
const uint8_t PIN_B = 3; // blue channel pin
// If your LED is common-anode (anode to 3.3V), set to true to invert PWM values
const bool COMMON_ANODE = true;
// If you're driving each cathode through an NPN low-side transistor (e.g. S8050),
// set this to true so PWM is driven to the transistor base (HIGH = ON).
// If false, PWM drives the LED cathode directly (LOW = ON for common-anode).
const bool TRANSISTOR_LOW_SIDE = true;

// Overall and per-channel scaling to balance perceived brightness.
// Tweak these to reduce green dominance. Range 0.0..1.0
const float BRIGHTNESS_SCALE = 1.00; // master brightness (0-1)
const float CHANNEL_SCALE_R = 1.00;  // red multiplier
const float CHANNEL_SCALE_G = 1.00;  // green multiplier (adjusted to restore pink)
const float CHANNEL_SCALE_B = 1.00;  // blue multiplier

// Enable/disable IMU (set false to skip I2C/init for wiring tests)
const bool IMU_ENABLED = true;

// forward declarations
void setRGB(uint8_t r, uint8_t g, uint8_t b);
void setPink();
// raw writer and test helper
void setRawRGB(uint8_t r, uint8_t g, uint8_t b);
void runColorCycleTest();
// Breathing effect configuration
const bool BREATHING_ENABLED = true;
const unsigned long BREATHING_PERIOD_MS = 3000; // full breathe cycle (ms)
const float BREATH_MIN = 0.00; // minimum brightness multiplier (0..1)
const float BREATH_MAX = 1.00; // maximum brightness multiplier (0..1)
// Debug prints for breathing calculations
const bool DEBUG_BREATHING = true;

// Base pink color (used for breathing)
// Tuned to a balanced rosy pink (reduce extreme red)
const uint8_t PINK_R = 255;
const uint8_t PINK_G = 20;
const uint8_t PINK_B = 80;

// Base white color (used for breathing)
// Tuned to a balanced rosy pink (reduce extreme red)
const uint8_t WHITE_R = 165;
const uint8_t WHITE_G = 225;
const uint8_t WHITE_B = 255;

// forward declaration for breathing updater
void updateBreathing();
// runtime control for breathing (allow temporarily disabling breathing)
bool breathing_runtime_enabled = false;
// runtime movement flag accessible by breathing logic
bool device_moving = false;
// Dimming-on-stop runtime state
bool dimming_active = true;
unsigned long dim_start_ms = 0;
unsigned long dim_duration_ms = 0;
// forward declarations for dimming
void startMaxPinkDim(unsigned long fade_ms);
void updateDimming();

// sensitivity scale factor respective to full scale setting provided in datasheet 
// Buffer configuration
#define LOOP_HZ 500
#define BUFFER_SECONDS 3
#define BUFFER_SIZE (LOOP_HZ * BUFFER_SECONDS)

struct IMUData {
  float ax, ay, az;
  float gx, gy, gz;
};

IMUData imu_buffer[BUFFER_SIZE];
volatile uint16_t imu_buffer_index = 0;
volatile bool buffer_filled = false;
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// Debug: print IMU samples to serial (CSV). Set to false to disable.
const bool DEBUG_IMU = false;
const unsigned long IMU_PRINT_MS = 100; // print interval (ms)

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

void setup() {
  Serial.begin(115200);
  // short delay to let USB-serial settle and avoid early garbled output
  delay(100);
  Serial.println("\n== varita starting ==");
  // Use default I2C pins on XIAO (guarded for test)
  if (IMU_ENABLED) {
    Wire.begin();
    MPU6050_Init();
  } else {
    Serial.println("IMU init disabled for test");
  }
  // initialize RGB so we can pulse while waiting for WiFi
  // initialize RGB removed to prevent high startup current during flashing
  // analogWriteRange(255);
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  // start with LED off by default
  setRGB(0, 0, 0);
  //randomSeed(analogRead(A0));
  // quick color test disabled
  // runColorCycleTest();

  // No WiFi on XIAO by default; run a short startup color check instead.
  runColorCycleTest();
  Serial.println("\n== varita started ==");
}

// Start a non-blocking dim sequence: set pink at max then fade to off
void startMaxPinkDim(unsigned long fade_ms) {
  dimming_active = true;
  dim_start_ms = millis();
  dim_duration_ms = fade_ms;
  // stop breathing while dimming
  breathing_runtime_enabled = false;
  // set to max pink immediately (use scaled path so channel balancing/inversion applies)
  setRGB(PINK_R, PINK_G, PINK_B);
}

// Call frequently from loop(); will update dimming state and re-enable breathing
void updateDimming() {
  if (!dimming_active) return;
  unsigned long now = millis();
  unsigned long elapsed = now - dim_start_ms;
  if (elapsed >= dim_duration_ms) {
    // finished fading — re-enable breathing and stop dimming
    dimming_active = false;
    breathing_runtime_enabled = true;
    // ensure LED fully off at end of fade (use scaled path for consistent inversion/brightness)
    setRGB(0, 0, 0);
    return;
  }
  float t = (float)elapsed / (float)dim_duration_ms; // 0..1
  // linear fade (1 -> 0). Could use easing if desired.
  float brightness = 1.0f - t;
  if (brightness < 0.0f) brightness = 0.0f;
  uint8_t r = (uint8_t)roundf((float)PINK_R * brightness);
  uint8_t g = (uint8_t)roundf((float)PINK_G * brightness);
  uint8_t b = (uint8_t)roundf((float)PINK_B * brightness);
  setRawRGB(r, g, b);
}

bool offsets_set = false;

double g0x = 0.0, g0y = 0.0, g0z = 0.0;
bool gravity_set = false;

volatile int mqtt_ident_pending = -1;

void loop() {
  analogWrite(4, 0);
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  if (IMU_ENABLED) {
    Read_RawValue();
  }
  // Divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; // temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;

  // Optional: print IMU values to Serial (human- or CSV-friendly)
  if (DEBUG_IMU) {
    static unsigned long last_imu_print = 0;
    unsigned long now = millis();
    if (now - last_imu_print >= IMU_PRINT_MS) {
      last_imu_print = now;
      // CSV: Ax,Ay,Az,Gx,Gy,Gz
      Serial.print(Ax, 4); Serial.print(',');
      Serial.print(Ay, 4); Serial.print(',');
      Serial.print(Az, 4); Serial.print(',');
      Serial.print(Gx, 4); Serial.print(',');
      Serial.print(Gy, 4); Serial.print(',');
      Serial.println(Gz, 4);
    }
  }


  // Calculate movement strength (vector magnitude)
  double accel_strength = sqrt(Ax*Ax + Ay*Ay + Az*Az);
  double gyro_strength = sqrt(Gx*Gx + Gy*Gy + Gz*Gz);

  static bool moving = false;
  static unsigned long last_movement_time = 0;
  const double gyro_movement_threshold = 150; // adjust as needed
  const double accel_movement_threshold = 1.4; // adjust as needed
  const double accel_delta_threshold = 0.05; // adjust as needed
  const double accel_delta_alpha = 0.2; // smoothing factor for weighted average
  const unsigned long movement_timeout = 300; // ms, time to consider movement finished
  static double max_gyro_strength = 0.0;
  static double min_gyro_strength = 1e9;
  static double max_accel_strength = 0.0;
  static double min_accel_strength = 1e9;
  static double last_accel_strength = 0.0;
  static double accel_delta_avg = 0.0;

  // Store IMU data in buffer if moving
  if (moving && imu_buffer_index < BUFFER_SIZE) {
    imu_buffer[imu_buffer_index].ax = Ax;
    imu_buffer[imu_buffer_index].ay = Ay;
    imu_buffer[imu_buffer_index].az = Az;
    imu_buffer[imu_buffer_index].gx = Gx;
    imu_buffer[imu_buffer_index].gy = Gy;
    imu_buffer[imu_buffer_index].gz = Gz;
    imu_buffer_index++;
    if (imu_buffer_index >= BUFFER_SIZE) {
      buffer_filled = true;
    }
  }

  // Loop frequency calculation
  static unsigned long last_loop_time = 0;
  static unsigned long loop_counter = 0;
  static float loop_hz = 0.0;
  loop_counter++;
  unsigned long now = millis();
  if (now - last_loop_time >= 1000) {
    loop_hz = loop_counter * 1000.0 / (now - last_loop_time);
    //Serial.print("Loop frequency: ");
    //Serial.print(loop_hz, 2);
    //Serial.println(" Hz");
    loop_counter = 0;
    last_loop_time = now;
  }

  // Movement detection logic
  bool movement_detected = (gyro_strength > gyro_movement_threshold) || (accel_strength > accel_movement_threshold);
  double accel_delta = fabs(accel_strength - last_accel_strength);
  last_accel_strength = accel_strength;
  // Weighted (exponential moving) average for accel_delta
  accel_delta_avg = accel_delta_alpha * accel_delta + (1.0 - accel_delta_alpha) * accel_delta_avg;

  if (movement_detected) {
    if (!moving) {
      moving = true;
      max_gyro_strength = gyro_strength;
      min_gyro_strength = gyro_strength;
      max_accel_strength = accel_strength;
      min_accel_strength = accel_strength;
      Serial.println("movement started");
      imu_buffer_index = 0;
      buffer_filled = false;
      // If a dimming sequence was running, cancel it and keep LED off
      if (dimming_active) {
        dimming_active = false;
        breathing_runtime_enabled = false;
        setRGB(0, 0, 0);
      }
    }
    if (gyro_strength > max_gyro_strength) max_gyro_strength = gyro_strength;
    if (gyro_strength < min_gyro_strength) min_gyro_strength = gyro_strength;
    if (accel_strength > max_accel_strength) max_accel_strength = accel_strength;
    if (accel_strength < min_accel_strength) min_accel_strength = accel_strength;
    //Serial.print("Accel strength: "); Serial.print(accel_strength, 4);
    //Serial.print(" | Gyro strength: "); Serial.print(gyro_strength, 4);
    //Serial.print(" | Accel delta avg: "); Serial.println(accel_delta_avg, 4);
    last_movement_time = millis();
  } else {
    if (moving && (millis() - last_movement_time > movement_timeout)
        && (accel_strength < accel_movement_threshold)
        && (accel_delta_avg < accel_delta_threshold)) {
      moving = false;
      Serial.print("movement finished | max gyro_strength: ");
      Serial.print(max_gyro_strength, 4);
      Serial.print(" | min gyro_strength: ");
      Serial.print(min_gyro_strength, 4);
      Serial.print(" | max accel_strength: ");
      Serial.print(max_accel_strength, 4);
      Serial.print(" | min accel_strength: ");
      Serial.println(min_accel_strength, 4);
      Serial.print("Buffer filled: ");
      Serial.print(imu_buffer_index);
      Serial.println(" samples");
      // Send buffered IMU data as a CSV block over TCP to a host
      int ident = send_buffer_over_tcp(imu_buffer, imu_buffer_index);
      if (ident >= 0) {
        mqtt_ident_pending = ident;
      }
      imu_buffer_index = 0;
      // Non-blocking MQTT send
      if (mqtt_ident_pending >= 0) {
        send_mqtt_identified(mqtt_ident_pending);
        mqtt_ident_pending = -1;
      }
      // Start the max-brightness-then-dim sequence (4s fade by default)
      startMaxPinkDim(4000);
    }
  }
  // update breathing animation (replaced by RGB cycle for testing)
  // inform breathing logic whether we're currently moving
  device_moving = moving;
  //cycleRGBAlternating();
  // update dimming (non-blocking)
  updateBreathing();
  updateDimming();
}

// Set RGB (0-255 per channel). Applies master brightness and per-channel
// scaling to balance perceived brightness, then inverts for common-anode.
void setRGB(uint8_t r, uint8_t g, uint8_t b) {
  // apply float scaling
  int r_scaled = (int)roundf((float)r * BRIGHTNESS_SCALE * CHANNEL_SCALE_R);
  int g_scaled = (int)roundf((float)g * BRIGHTNESS_SCALE * CHANNEL_SCALE_G);
  int b_scaled = (int)roundf((float)b * BRIGHTNESS_SCALE * CHANNEL_SCALE_B);
  // clamp
  if (r_scaled < 0) r_scaled = 0; if (r_scaled > 255) r_scaled = 255;
  if (g_scaled < 0) g_scaled = 0; if (g_scaled > 255) g_scaled = 255;
  if (b_scaled < 0) b_scaled = 0; if (b_scaled > 255) b_scaled = 255;

  uint8_t out_r = (uint8_t)r_scaled;
  uint8_t out_g = (uint8_t)g_scaled;
  uint8_t out_b = (uint8_t)b_scaled;

  // For common-anode LEDs we normally invert PWM because the MCU sinks current
  // (LOW = ON). However when using NPN low-side transistors the MCU drives
  // the transistor base and HIGH must turn the transistor ON. In that case
  // do not invert the PWM values.
  if (COMMON_ANODE && !TRANSISTOR_LOW_SIDE) {
    out_r = 255 - out_r;
    out_g = 255 - out_g;
    out_b = 255 - out_b;
  }

  Serial.print("setRGB: ");
  Serial.print(out_r); Serial.print(", ");
  Serial.print(out_g); Serial.print(", ");
  Serial.println(out_b);
  analogWrite(PIN_R, out_r);
  analogWrite(PIN_G, out_g);
  analogWrite(PIN_B, out_b);
}

// Raw output (no brightness/channel scaling)
void setRawRGB(uint8_t r, uint8_t g, uint8_t b) {
  uint8_t out_r = r;
  uint8_t out_g = g;
  uint8_t out_b = b;
  if (COMMON_ANODE && !TRANSISTOR_LOW_SIDE) {
    out_r = 255 - out_r;
    out_g = 255 - out_g;
    out_b = 255 - out_b;
  }
  Serial.print("setRawRGB: ");
  Serial.print(out_r); Serial.print(", ");
  Serial.print(out_g); Serial.print(", ");
  Serial.println(out_b);

  analogWrite(PIN_R, out_r);
  analogWrite(PIN_G, out_g);
  analogWrite(PIN_B, out_b);
}

// Quick raw color cycle to verify each channel; uses setRawRGB so scaling
// and brightness adjustments don't affect the test.
void runColorCycleTest() {
  setRawRGB(255, 0, 0); delay(800);
  setRawRGB(0, 255, 0); delay(800);
  setRawRGB(0, 0, 255); delay(800);
  setRawRGB(255, 255, 255); delay(800);
  setRawRGB(PINK_R, PINK_G, PINK_B); delay(1200);
  setRawRGB(0, 0, 0); delay(1500);
}

// Light a pleasant light-pink tone. Adjust RGB values if needed.
void setPink() {
  // Light pink: R=255, G=182, B=193 (values 0-255)
  setRGB(PINK_R, PINK_G, PINK_B);
}

// Non-blocking breathing update; call frequently from loop()
void updateBreathing() {
  if (!BREATHING_ENABLED || !breathing_runtime_enabled) return;
  // don't run breathing animation while device is moving
  if (device_moving) return;
  static unsigned long last_update = 0;
  const unsigned long min_interval = 20; // update at most ~50Hz
  unsigned long now = millis();
  if (now - last_update < min_interval) return;
  last_update = now;

  // compute phase [0..1)
  unsigned long t = now % BREATHING_PERIOD_MS;
  float phase = (float)t / (float)BREATHING_PERIOD_MS;
  // use a sine wave for smooth breathing: map sin(-pi/2..3pi/2) -> 0..1
  float v = sinf(phase * 2.0f * PI - PI/2.0f);
  float norm = (v + 1.0f) * 0.5f; // 0..1
  // scale between min and max
  float breath = BREATH_MIN + (BREATH_MAX - BREATH_MIN) * norm;

  // apply breathing to base pink and set
  uint8_t r = (uint8_t)roundf((float)WHITE_R * breath);
  uint8_t g = (uint8_t)roundf((float)WHITE_G * breath);
  uint8_t b = (uint8_t)roundf((float)WHITE_B * breath);
  setRGB(r, g, b);

  if (DEBUG_BREATHING) {
    static unsigned long last_dbg = 0;
    unsigned long now_dbg = millis();
    if (now_dbg - last_dbg >= 500) {
      last_dbg = now_dbg;
      // print raw breath (0..1) and the scaled outputs after setRGB's scaling
      int r_scaled = (int)roundf((float)r * BRIGHTNESS_SCALE * CHANNEL_SCALE_R);
      int g_scaled = (int)roundf((float)g * BRIGHTNESS_SCALE * CHANNEL_SCALE_G);
      int b_scaled = (int)roundf((float)b * BRIGHTNESS_SCALE * CHANNEL_SCALE_B);
    }
  }
}

// Alternate pure red, green, blue at full brightness for 1s each.
void cycleRGBAlternating() {
  static unsigned long last_change = 0;
  static uint8_t state = 0; // 0=red,1=green,2=blue
  unsigned long now = millis();
  if (last_change == 0) {
    last_change = now;
    state = 0;
    setRGB(255, 0, 0);
    return;
  }
  if (now - last_change >= 5000) {
    last_change = now;
    state = (state + 1) % 3;
    if (state == 0) setRGB(255, 0, 0);
    else if (state == 1) setRGB(0, 255, 0);
    else setRGB(0, 0, 255);
  }
}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  // No-op for onboard IMU path (kept for API compatibility).
  (void)deviceAddress; (void)regAddress; (void)data;
}

// read all 14 register
void Read_RawValue(){
  // Read from onboard IMU and convert to the same scaled integer representation
  // used previously (AccelScaleFactor -> raw int16; GyroScaleFactor -> raw int16).
  float ax_mss = 0.0f, ay_mss = 0.0f, az_mss = 0.0f;
  float gx_dps = 0.0f, gy_dps = 0.0f, gz_dps = 0.0f;

  // If acceleration available, read it (m/s^2)
  ax_mss = myIMU.readFloatAccelX();
  ay_mss = myIMU.readFloatAccelY();
  az_mss = myIMU.readFloatAccelZ();

  gx_dps = myIMU.readFloatGyroX();
  gy_dps = myIMU.readFloatGyroY();
  gz_dps = myIMU.readFloatGyroZ();

  // Convert acceleration m/s^2 -> g units then scale to int16 range used earlier
  const float g_conv = 9.80665f;
  float ax_g = ax_mss / g_conv;
  float ay_g = ay_mss / g_conv;
  float az_g = az_mss / g_conv;

  AccelX = (int16_t)roundf(ax_g * (float)AccelScaleFactor);
  AccelY = (int16_t)roundf(ay_g * (float)AccelScaleFactor);
  AccelZ = (int16_t)roundf(az_g * (float)AccelScaleFactor);

  // Gyroscope: assume IMU.readGyroscope returns deg/s; scale to previous factor
  GyroX = (int16_t)roundf(gx_dps * (float)GyroScaleFactor);
  GyroY = (int16_t)roundf(gy_dps * (float)GyroScaleFactor);
  GyroZ = (int16_t)roundf(gz_dps * (float)GyroScaleFactor);
  // Temperature not provided by onboard IMU here
  Temperature = 0;
}

//configure MPU6050
void MPU6050_Init(){
  // Initialize onboard IMU (LSM6DS3/LSM6DSOX via Arduino_LSM6DS3)
  Serial.println("Initializing onboard IMU...");
  delay(150);
  if (!myIMU.begin()) {
    Serial.println("Failed to initialize onboard IMU — check library/board");
  } else {
    Serial.println("Onboard IMU initialized");
  }
}

// Configuration for TCP destination (set these before uploading)
const char* TCP_SERVER_IP = "192.168.0.146"; // change to host IP
const uint16_t TCP_SERVER_PORT = 5005;

// Connect and send the buffered IMU data as CSV lines
int send_buffer_over_tcp(IMUData *buffer, uint16_t count) {
  // Networking is not implemented for the XIAO build. Return -1.
  Serial.println("send_buffer_over_tcp: network disabled on this build");
  (void)buffer;
  (void)count;
  return -1;
}

// Stub for MQTT-identification handler when networking is disabled
void send_mqtt_identified(int ident) {
  (void)ident;
}
