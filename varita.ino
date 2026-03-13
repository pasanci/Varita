#include <Wire.h>
// WiFi for ESP8266 boards
#include <ESP8266WiFi.h>
#include "mqtt_helpers.h"

// MPU6050 Slave Device Address
const uint8_t MPU6050SlaveAddress = 0x68;

// Select SDA and SCL pins for I2C communication 
const uint8_t scl = D6;
const uint8_t sda = D7;

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

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV   =  0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL    =  0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1   =  0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2   =  0x6C;
const uint8_t MPU6050_REGISTER_CONFIG       =  0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG  =  0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG =  0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN      =  0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE   =  0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H =  0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET  = 0x68;

int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;

void setup() {
  Serial.begin(115200);
  Wire.begin(sda, scl);
  MPU6050_Init();
  // connect to WiFi (configure SSID/PASS below)
  WiFi.mode(WIFI_STA);
  WiFi.begin("SKPT_2G", "04skypet30");
  unsigned long wifi_start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - wifi_start < 10000) {
    Serial.print('.');
    delay(200);
  }
  setup_mqtt();
}

bool offsets_set = false;

double g0x = 0.0, g0y = 0.0, g0z = 0.0;
bool gravity_set = false;

volatile int mqtt_ident_pending = -1;

void loop() {
  double Ax, Ay, Az, T, Gx, Gy, Gz;
  Read_RawValue(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_XOUT_H);
  // Divide each with their sensitivity scale factor
  Ax = (double)AccelX/AccelScaleFactor;
  Ay = (double)AccelY/AccelScaleFactor;
  Az = (double)AccelZ/AccelScaleFactor;
  T = (double)Temperature/340+36.53; // temperature formula
  Gx = (double)GyroX/GyroScaleFactor;
  Gy = (double)GyroY/GyroScaleFactor;
  Gz = (double)GyroZ/GyroScaleFactor;


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
    }
  }

}

void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.write(data);
  Wire.endTransmission();
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress){
  Wire.beginTransmission(deviceAddress);
  Wire.write(regAddress);
  Wire.endTransmission();
  Wire.requestFrom(deviceAddress, (uint8_t)14);
  AccelX = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelY = (((int16_t)Wire.read()<<8) | Wire.read());
  AccelZ = (((int16_t)Wire.read()<<8) | Wire.read());
  Temperature = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroX = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroY = (((int16_t)Wire.read()<<8) | Wire.read());
  GyroZ = (((int16_t)Wire.read()<<8) | Wire.read());
}

//configure MPU6050
void MPU6050_Init(){
  delay(150);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_CONFIG, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_GYRO_CONFIG, 0x00);//set +/-250 degree/second full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_ACCEL_CONFIG, 0x00);// set +/- 2g full scale
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_FIFO_EN, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_INT_ENABLE, 0x01);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
  I2C_Write(MPU6050SlaveAddress, MPU6050_REGISTER_USER_CTRL, 0x00);
}

// Configuration for TCP destination (set these before uploading)
const char* TCP_SERVER_IP = "192.168.0.146"; // change to host IP
const uint16_t TCP_SERVER_PORT = 5005;

// Connect and send the buffered IMU data as CSV lines
int send_buffer_over_tcp(IMUData *buffer, uint16_t count) {
  if (WiFi.status() != WL_CONNECTED) {
    Serial.println("WiFi not connected, skip send");
    return -1;
  }
  WiFiClient client;
  if (!client.connect(TCP_SERVER_IP, TCP_SERVER_PORT)) {
    Serial.println("TCP connect failed");
    client.stop();
    return -1;
  }
  // Send header with number of samples
  client.print("MOVEMENT_START,");
  client.println(count);
  for (uint16_t i = 0; i < count; i++) {
    client.print(buffer[i].ax, 4); client.print(',');
    client.print(buffer[i].ay, 4); client.print(',');
    client.print(buffer[i].az, 4); client.print(',');
    client.print(buffer[i].gx, 4); client.print(',');
    client.print(buffer[i].gy, 4); client.print(',');
    client.println(buffer[i].gz, 4);
    yield();
  }
  client.println("MOVEMENT_END");

  // Wait for response from server (identified integer or -1)
  unsigned long start = millis();
  String response = "";
  while (client.connected() && millis() - start < 2000) { // 2s timeout
    while (client.available()) {
      char c = client.read();
      if (c == '\n' || c == '\r') {
        if (response.length() > 0) break;
      } else {
        response += c;
      }
    }
    if (response.length() > 0) break;
    delay(10);
  }
  client.stop();
  int ident = -1;
  if (response.length() > 0) {
    ident = response.toInt();
  }
  Serial.print("Identification result: ");
  Serial.println(ident);
  return ident;
}
