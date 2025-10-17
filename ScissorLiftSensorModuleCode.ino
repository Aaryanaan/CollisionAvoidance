/*
  Full Scissor-Lift Corner Module
  --------------------------------
  Components:
    • VL53L0X (ToF) on I2C
    • HC-SR04 (Ultrasonic) on TRIG/ECHO
    • Servo (245° sweep)
    • Piezo buzzer for distance warning

  Features:
    - 1-D Kalman fusion (ToF + Ultrasonic)
    - Adaptive measurement noise (auto-down-weights noisy sensor)
    - Non-blocking beeper (<30 cm)
    - Servo sweep 0↔245° with live fusion
    - Serial logging: t(ms), angle(°), z_tof, z_us, R_tof, R_us, x_fused(mm), P
*/

// We use I²C (Wire) and adafruit driver for ToF sensor. Mature libraries reduce low-level I²C edge cases, allowing focus on fusion logic
#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <Servo.h>

// single lox instance kept for ranging calls
Adafruit_VL53L0X lox = Adafruit_VL53L0X();
Servo scanServo;

// ---------- GPIO assignment ----------
// U.S. needs trigger and echo pins
const int PIN_TRIG   = 2;
const int PIN_ECHO   = 3;
const int PIN_BUZZER = 9;
const int SERVO_PIN  = 6;

// ---------- Servo sweep parameters ----------
const int SERVO_MIN_DEG = 0;
const int SERVO_MAX_DEG = 245; // sweep range
int servoAngle = SERVO_MIN_DEG;
int servoStep  = 3; // 3 degree step size
unsigned long lastServoMove = 0; 
const unsigned long SERVO_DELAY_MS = 30; // non-blocking timing - allows other operations to continue running

// ---------- Kalman filter ----------
float x = 1000.0f;    // initialize fused distance (mm)
float P = 10000.0f;   // variance/uncertainty
const float Q = 8.0f; // process noise

const float R_TOF_BASE = 60.0f;
const float R_US_BASE  = 180.0f;

// outliers get scaled variance to minimize their impact on "distance" value
const float R_SCALE_GOOD = 1.0f; 
const float R_SCALE_SUS  = 6.0f;
const float R_SCALE_BAD  = 50.0f;

// ---------- Beeper ----------
const int   BEEP_THRESH_MM   = 300;
const int   BEEP_NEAR_MM     = 60;
const float PITCH_MIN_HZ     = 2300.0;
const float PITCH_MAX_HZ     = 3600.0;
const int   INTERVAL_MAX_MS  = 550;
const int   INTERVAL_MIN_MS  = 70;
bool        buzzerOn = false;
unsigned long lastBeepToggle = 0;

// ---------- Misc ----------
unsigned long t_prev_ms = 0;
float ztof_prev = NAN;
float zus_prev  = NAN;

// ---------- Utilities ----------
float clamp(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max) {
  if (in_max == in_min) return out_min;
  float t = (x - in_min) / (in_max - in_min);
  return out_min + t * (out_max - out_min);
}

// ---------- Kalman update ----------
void kf_update(float z, float R) {
  if (isnan(z) || isinf(z)) return;
  float S = P + R;
  float K = P / S;
  x = x + K * (z - x);
  P = (1.0f - K) * P;
}

// ---------- Sensors ----------
float readUltrasonicMM() {
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long us = pulseIn(PIN_ECHO, HIGH, 30000UL);
  if (us == 0) return NAN;
  float mm = (us * 0.343f) / 2.0f;
  if (mm < 20.0f || mm > 4000.0f) return NAN;
  return mm;
}

float readToFMM(bool &ok, uint8_t &statusCode) {
  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure, false);
  statusCode = measure.RangeStatus;
  if (measure.RangeStatus != 4) {
    ok = true;
    int mm = (int)measure.RangeMilliMeter;
    if (mm < 30) mm = 30; // clamp below min reliable
    if (mm > 4000) return NAN;
    return (float)mm;
  } else {
    ok = false;
    return NAN;
  }
}

// ---------- Adaptive noise ----------
float adaptiveR_TOF(float z, float z_prev, bool ok, uint8_t status) {
  if (!ok || isnan(z)) return R_TOF_BASE * R_SCALE_BAD;
  float scale = (status == 0) ? R_SCALE_GOOD : R_SCALE_SUS;
  if (!isnan(z_prev)) {
    float jump = fabs(z - z_prev);
    if (jump > 100.0f) scale = max(scale, R_SCALE_SUS);
    if (jump > 250.0f) scale = max(scale, R_SCALE_BAD);
  }
  return R_TOF_BASE * scale;
}

float adaptiveR_US(float z, float z_prev) {
  if (isnan(z)) return R_US_BASE * R_SCALE_BAD;
  float scale = R_SCALE_GOOD;
  if (!isnan(z_prev)) {
    float jump = fabs(z - z_prev);
    if (jump > 150.0f) scale = max(scale, R_SCALE_SUS);
    if (jump > 300.0f) scale = max(scale, R_SCALE_BAD);
  }
  return R_US_BASE * scale;
}

// ---------- Beeper ----------
void updateBeeper(int fused_mm) {
  if (fused_mm >= BEEP_THRESH_MM || fused_mm <= 0) {
    noTone(PIN_BUZZER);
    buzzerOn = false;
    return;
  }
  int interval = (int)mapfloat((float)fused_mm, BEEP_NEAR_MM, BEEP_THRESH_MM,
                               INTERVAL_MIN_MS, INTERVAL_MAX_MS);
  interval = clamp(interval, INTERVAL_MIN_MS, INTERVAL_MAX_MS);
  float pitch = mapfloat((float)fused_mm, BEEP_NEAR_MM, BEEP_THRESH_MM,
                         PITCH_MAX_HZ, PITCH_MIN_HZ);
  pitch = clamp(pitch, PITCH_MIN_HZ, PITCH_MAX_HZ);
  unsigned long now = millis();
  if (now - lastBeepToggle >= (unsigned long)interval) {
    lastBeepToggle = now;
    if (buzzerOn) {
      noTone(PIN_BUZZER);
      buzzerOn = false;
    } else {
      tone(PIN_BUZZER, (unsigned int)pitch);
      buzzerOn = true;
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  pinMode(PIN_BUZZER, OUTPUT);

  Wire.begin();
  if (!lox.begin()) {
    Serial.println(F("VL53L0X not found, continuing with Ultrasonic only."));
  }

  scanServo.attach(SERVO_PIN);
  scanServo.write(SERVO_MIN_DEG);

  Serial.println(F("t(ms), angle(°), z_tof, z_us, R_tof, R_us, x_fused, P"));
}

// ---------- Main loop ----------
void loop() {
  // Predict
  P = P + Q;

  // Read sensors
  bool tof_ok = false; uint8_t tof_status = 0xFF;
  float z_tof = readToFMM(tof_ok, tof_status);
  float z_us  = readUltrasonicMM();

  // Adaptive noise
  float R_tof = adaptiveR_TOF(z_tof, ztof_prev, tof_ok, tof_status);
  float R_us  = adaptiveR_US(z_us,  zus_prev);

  // Fuse
  kf_update(z_tof, R_tof);
  kf_update(z_us,  R_us);

  // Beeper
  updateBeeper((int)x);

  // Servo sweep
  unsigned long now = millis();
  if (now - lastServoMove >= SERVO_DELAY_MS) {
    lastServoMove = now;
    servoAngle += servoStep;
    if (servoAngle >= SERVO_MAX_DEG || servoAngle <= SERVO_MIN_DEG) {
      servoStep = -servoStep;
      servoAngle = clamp(servoAngle, SERVO_MIN_DEG, SERVO_MAX_DEG);
    }
    scanServo.write(servoAngle);
  }

  // Logging every ~100 ms
  if (now - t_prev_ms >= 100) {
    t_prev_ms = now;
    Serial.print(now);       Serial.print(", ");
    Serial.print(servoAngle);Serial.print(", ");
    Serial.print(isnan(z_tof)?NAN:z_tof); Serial.print(", ");
    Serial.print(isnan(z_us)?NAN:z_us);   Serial.print(", ");
    Serial.print(R_tof);     Serial.print(", ");
    Serial.print(R_us);      Serial.print(", ");
    Serial.print(x);         Serial.print(", ");
    Serial.println(P);
  }

  // Store previous
  ztof_prev = z_tof;
  zus_prev  = z_us;

  delay(10);
}
