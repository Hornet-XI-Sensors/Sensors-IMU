#include <FlexCAN_T4.h>
#include <Wire.h>
#include <MS5837.h>
#include <Arduino.h>
#include "Adafruit9DOF.h"
#include "MadgwickAHRS.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
#define WIRE Wire

// Create object
Adafruit9DOF imu;
MS5837 bar30;

int val;
float calc;
float scaledup;
float pres;

// Timing for 50Hz updates
const unsigned long period = 10; // milliseconds (50Hz)
static unsigned long last_update = 0;

/*
TLDR
Each of the 5 sensor values have been matched to one diff frame each
This is cause for roll pitch yaw, if i convert decimal into  int, it becomes a 32 bit unsigned integer which takes up 4 bytes alr out of 8 for the can bus frame
Im also adding a counter for debugging so thats 2 more bytes and then rest 2 are rserrved
So to be safe, 1 frame per value is better, esp since we have leeway for 3703 frames / second at 50% bus utilisation recommendation
*/

// can controller
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

// CAN set up
static const uint32_t CAN_BITRATE = 1000000; // 1 Mbps

// messages ids
static const uint32_t CAN_ID_ROLL = 0x300;
static const uint32_t CAN_ID_PITCH = 0x301;
static const uint32_t CAN_ID_YAW = 0x302;
static const uint32_t CAN_ID_PRESSURE = 0x303;
static const uint32_t CAN_ID_DEPTH = 0x304;

// scaling factors to convert float to integer
static const float SCALE_RPY = 10000.0f; // for 4 dp
static const float SCALE_PD = 100.0f;    // for 2 dp

// timings
const unsigned long period_ms = 20; // 50 Hz corresponds to a period of 20ms

// separate counters per message ID (will help debugging any frame drops per stream, especially as sending more data now)
static uint16_t ctr_roll = 0;
static uint16_t ctr_pitch = 0;
static uint16_t ctr_yaw = 0;
static uint16_t ctr_pressure = 0;
static uint16_t ctr_depth = 0;

// helpers to convert our numercial sensor values into CANBUS byte formats
// Function to convert quaternion to Euler angles (roll, pitch, yaw)
void quaternionToEuler(float q0, float q1, float q2, float q3, float &roll, float &pitch, float &yaw)
{
  // Roll (X-axis rotation)
  roll = atan2(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));

  // Pitch (Y-axis rotation)
  float sinp = 2.0f * (q0 * q2 - q3 * q1);
  if (sinp >= 1.0f)
  {
    pitch = M_PI / 2.0f;
  }
  else if (sinp <= -1.0f)
  {
    pitch = -M_PI / 2.0f;
  }
  else
  {
    pitch = asin(sinp);
  }

  // Yaw (Z-axis rotation)
  yaw = atan2(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

  // Convert to degrees
  roll *= 180.0f / M_PI;
  pitch *= 180.0f / M_PI;
  yaw *= 180.0f / M_PI;
}

// helper: to write uint16 as little-endian into buf at index i
static inline void put_u16_le(uint8_t *buf, int i, uint16_t v)
{                                          // buf - pointer to can data array (msg.buf[i]) // i - starting index // v - the sensor numerical
  buf[i + 0] = (uint8_t)(v & 0xFF);        // extracts the lowest 8 bits thus creating LSB (least significant byte)
  buf[i + 1] = (uint8_t)((v >> 8) & 0xFF); // shifts right by 8 and masks with 0xFF to extract only 8 bits, thus creating MSB most significant byte
}

// helper: to write int16 as little-endian into buf at index i
static inline void put_i16_le(uint8_t *buf, int i, int16_t v)
{
  uint16_t u = (uint16_t)v; // reinterpret bits
  put_u16_le(buf, i, u);
}

// Helper: write int32 little-endian into buf at index i
static inline void put_i32_le(uint8_t *buf, int i, int32_t v)
{
  uint32_t u = (uint32_t)v; // reinterpret bits
  buf[i + 0] = (uint8_t)(u & 0xFF);
  buf[i + 1] = (uint8_t)((u >> 8) & 0xFF);
  buf[i + 2] = (uint8_t)((u >> 16) & 0xFF);
  buf[i + 3] = (uint8_t)((u >> 24) & 0xFF);
}

// function for sending roll, pitch and yaw frames

void send_rpy_frame(uint32_t can_id, float angle_deg, uint16_t &counter_ref)
{

  // convert float to an integer, specifically an int32 since 3 digits + 4 dp = int32
  int32_t scaled = (int32_t)lroundf(angle_deg * SCALE_RPY);

  CAN_message_t msg; // struct from can library
  msg.id = can_id;   // identifier
  msg.len = 8;       // message length

  // putting the int32 value in bytes 0,1,2,3
  put_i32_le(msg.buf, 0, scaled);

  // putting counter in bytes 4,5
  put_u16_le(msg.buf, 4, counter_ref);

  // reserved bytes 6,7 LOL
  msg.buf[6] = 0;
  msg.buf[7] = 0;

  Can0.write(msg); // sending acc can message

  counter_ref++;
}

// function for sending pressure frames

void send_pressure_frame(float pressure, uint16_t &counter_ref)
{
  // pressure is never negative has been assumed

  float p = pressure;

  // scaling by 100 to remove 2p and converting float into int16
  uint16_t scaled = (uint16_t)lroundf(p * SCALE_PD);

  CAN_message_t msg;        // struct from can library
  msg.id = CAN_ID_PRESSURE; // identifier
  msg.len = 8;              // message length

  // putting the int16 value in bytes 0,1
  put_u16_le(msg.buf, 0, scaled);

  // putting counter in bytes 2,3
  put_u16_le(msg.buf, 2, counter_ref);

  // remaining bytes setting to 0
  for (int i = 4; i < 8; i++)
    msg.buf[i] = 0;

  Can0.write(msg); // sending acc can message

  counter_ref++;
}

void send_depth_frame(float depth_m, uint16_t &counter_ref)
{
  // scaling by 100 to remove 2p and converting float into int16
  int16_t scaled = (int16_t)lroundf(depth_m * SCALE_PD);

  CAN_message_t msg;     // struct from can library
  msg.id = CAN_ID_DEPTH; // identifier
  msg.len = 8;           // message length

  // putting the int16 value in bytes 0,1
  put_i16_le(msg.buf, 0, scaled);

  // putting counter in bytes 2,3
  put_u16_le(msg.buf, 2, counter_ref);

  // rremaining bytes setting to 0
  for (int i = 4; i < 8; i++)
    msg.buf[i] = 0;

  Can0.write(msg); // sending acc can message

  counter_ref++;
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial to be ready

  Wire.begin();

  if (!imu.begin())
  {
    Serial.println("Adafruit 9DOF initialization failed!");
    while (1)
      ;
  }

  bar30.init();
  Serial.println("Bar30 init OK!");
  bar30.setFluidDensity(997); // freshwater is 997 air is 1225

  // TODO: add remaining sensors set up

  Can0.begin();                  // turns on can hardware
  Can0.setBaudRate(CAN_BITRATE); // sets can timing to match can bus - 1 mbps rn
  Serial.println("Teensy CAN started");
}

void loop()
{
  // Run at 50 Hz (every 20ms)
  if (millis() - last_update >= period_ms)
  {
    //read Depth sensors
    bar30.read();
    // Read sensor data
    float ax, ay, az; // accelerometer
    float gx, gy, gz; // gyroscope
    float mx, my, mz; // magnetometer
    imu.readAll(ax, ay, az, gx, gy, gz, mx, my, mz);

    bar30.read();
    float depth = bar30.depth(); // units in meters(m)

    // Convert gyroscope from degrees/s to radians/s for Madgwick
    float gx_rad = gx * DEG_TO_RAD;
    float gy_rad = gy * DEG_TO_RAD;
    float gz_rad = gz * DEG_TO_RAD;

    // Update Madgwick AHRS algorithm
    MadgwickAHRSupdateIMU(gx_rad, gy_rad, gz_rad, ax, ay, az);

    // Convert quaternion to Euler angles
    float roll, pitch, yaw;
    quaternionToEuler(q0, q1, q2, q3, roll, pitch, yaw);

    // read internal hull pressure
    val = analogRead(A0);
    calc = (val / 1023.0) * (3.3);
    scaledup = calc * 1.5;
    pres = (scaledup + 0.204) / 0.0204;

    //Depth Value
    float depth_m = bar30.depth();

    // 3) Send 5 separate frames (one per value)
    send_rpy_frame(CAN_ID_ROLL, roll, ctr_roll);
    send_rpy_frame(CAN_ID_PITCH, pitch, ctr_pitch);
    send_rpy_frame(CAN_ID_YAW, yaw, ctr_yaw);
    send_pressure_frame(pres, ctr_pressure);
    send_depth_frame(depth_m, ctr_depth);
    
    //For Checking Purpose
    Serial.print(roll, 4);
    Serial.print(pitch, 4);
    Serial.print(yaw, 4);
    Serial.print(pres);
    Serial.println(depth);

    last_update = millis();
  }
}
