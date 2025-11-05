// ===== SCISSOR LIFT SAFETY SENSOR =====
// VL53L0X Time-of-Flight + 270° Servo + Proximity Buzzer
// Author: Aaryan / ChatGPT Collab
// Version: v2.1 — Rectangular Zone Alert + Distance-Based Beep
// ---------------------------------------------------------------
// Connections:
//   - VL53L0X: VCC→5V, GND→GND, SDA→A4, SCL→A5
//   - Servo: Signal→D7, 5V→External 5V (≥1A), GND shared with Arduino
//   - Buzzer: Signal→D8, GND→GND
// ---------------------------------------------------------------

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X tof;
Servo scanServo;

// ---------------- USER CONFIG ----------------
const int SERVO_PIN = 7;
const int BUZZER_PIN = 8;

// Servo motion calibration
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

// Sweep geometry
const float TOTAL_FOV_DEG = 270.0;
const float TOF_FOV_DEG   = 25.0;
const float CENTER_DEG    = 135.0;  // 0° = right, 90° = forward

const float DEG_STEP      = 2.0;
const uint16_t SERVO_SETTLE_MS = 10;
const uint16_t TOF_TIMEOUT_MS  = 30;

// Sensor performance
const uint32_t TIMING_BUDGET_US = 20000;  // 20 ms read time
const uint16_t IM_INTERVAL_MS   = TIMING_BUDGET_US / 1000;

// Safety distances
const int ALERT_DISTANCE_MM = 200;   // 20 cm proximity alert
const int MIN_DISTANCE_MM   = 30;    // 3 cm scaling floor

// Lift geometry (define in millimeters)
const float RECT_X_MIN = -200;   // left safety margin
const float RECT_X_MAX = 800;    // right boundary (width)
const float RECT_Y_MIN = -100;   // behind sensor
const float RECT_Y_MAX = 600;    // in front of lift

// ---------------------------------------------------------------

// Derived sweep bounds (keep ToF beam inside 270°)
const float HALF_TOTAL   = TOTAL_FOV_DEG * 0.5f;
const float HALF_TOF_FOV = TOF_FOV_DEG * 0.5f;
const float SWEEP_START_DEG = (CENTER_DEG - HALF_TOTAL) + HALF_TOF_FOV;
const float SWEEP_END_DEG   = (CENTER_DEG + HALF_TOTAL) - HALF_TOF_FOV;

// Map angle [0–270] to servo microseconds
int angle270ToMicros(float angleDeg) {
  angleDeg = constrain(angleDeg, 0, 270);
  float t = angleDeg / 270.0f;
  return SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US);
}

void gotoAngle(float ang) { scanServo.writeMicroseconds(angle270ToMicros(ang)); }

uint16_t readToFmm() { return tof.readRangeContinuousMillimeters(); }

// ---------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  pinMode(BUZZER_PIN, OUTPUT);

  scanServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);

  // Initialize sensor
  if (!tof.init()) {
    Serial.println("ERR:init");
    while (1);
  }

  tof.setTimeout(TOF_TIMEOUT_MS);
  tof.setSignalRateLimit(0.25f);
  tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodPreRange, 14);
  tof.setVcselPulsePeriod(VL53L0X::VcselPeriodFinalRange, 10);
  tof.setMeasurementTimingBudget(TIMING_BUDGET_US);
  tof.startContinuous(IM_INTERVAL_MS);

  gotoAngle(SWEEP_START_DEG);
  delay(200);
  Serial.println("=== Scissor Lift Safety System Active ===");
}

// ---------------------------------------------------------------
void loop() {
  // Sweep forward
  for (float ang = SWEEP_START_DEG; ang <= SWEEP_END_DEG; ang += DEG_STEP)
    scanAndAlert(ang);

  // Sweep backward
  for (float ang = SWEEP_END_DEG; ang >= SWEEP_START_DEG; ang -= DEG_STEP)
    scanAndAlert(ang);
}

// ---------------------------------------------------------------
void scanAndAlert(float ang) {
  gotoAngle(ang);
  delay(SERVO_SETTLE_MS);

  uint16_t mm = readToFmm();
  if (tof.timeoutOccurred() || mm == 0 || mm > 4000) {
    noTone(BUZZER_PIN);
    return;
  }

  // Convert polar → Cartesian (mm)
  float x = mm * cos(radians(ang));
  float y = mm * sin(radians(ang));

  // Check if point lies inside rectangular safety perimeter
  bool inRect = (x >= RECT_X_MIN && x <= RECT_X_MAX &&
                 y >= RECT_Y_MIN && y <= RECT_Y_MAX);

  // Serial data for visualization
  Serial.print(ang, 1);
  Serial.print(",");
  Serial.print(mm);
  Serial.print(",");
  Serial.print(x, 0);
  Serial.print(",");
  Serial.println(y, 0);

  // Alert only if inside rectangular zone and too close
  if (inRect && mm <= ALERT_DISTANCE_MM) {
    proximityBeep(mm);
  } else {
    noTone(BUZZER_PIN);
  }
}

// ---------------------------------------------------------------
void proximityBeep(uint16_t distance) {
  // Map distance → pitch (closer = higher)
  int freq = map(distance, ALERT_DISTANCE_MM, MIN_DISTANCE_MM, 400, 2500);
  freq = constrain(freq, 400, 2500);

  // Map distance → delay between beeps (closer = faster ping rate)
  int pingDelay = map(distance, ALERT_DISTANCE_MM, MIN_DISTANCE_MM, 300, 50);
  pingDelay = constrain(pingDelay, 50, 300);

  // Play short tone
  tone(BUZZER_PIN, freq, 40);
  delay(pingDelay);
}