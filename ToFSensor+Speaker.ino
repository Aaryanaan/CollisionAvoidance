// ===== FAST SWEEP + PROXIMITY BEEPER =====
// Board: Uno/Nano/Teensy (Uno/Nano I2C on A4/A5)

#include <Wire.h>
#include <VL53L0X.h>
#include <Servo.h>

VL53L0X tof;
Servo scanServo;

// ---------------- CONFIG ----------------
const int SERVO_PIN = 7;
const int SERVO_MIN_US = 500;
const int SERVO_MAX_US = 2500;

const int BUZZER_PIN = 8; // Active or passive buzzer

const float TOTAL_FOV_DEG = 270.0;
const float TOF_FOV_DEG = 25.0;
const float CENTER_DEG = 135.0;

const float DEG_STEP = 2.0;
const uint16_t SERVO_SETTLE_MS = 10;
const uint16_t TOF_TIMEOUT_MS = 30;

const uint32_t TIMING_BUDGET_US = 20000;
const uint16_t IM_INTERVAL_MS = TIMING_BUDGET_US / 1000;

// Proximity thresholds
const int ALERT_DISTANCE_MM = 200;   // 20 cm start threshold
const int MIN_DISTANCE_MM = 30;      // Cap for scaling (3 cm)

// Servo sweep bounds
const float HALF_TOTAL = TOTAL_FOV_DEG * 0.5f;
const float HALF_TOF_FOV = TOF_FOV_DEG * 0.5f;
const float SWEEP_START_DEG = (CENTER_DEG - HALF_TOTAL) + HALF_TOF_FOV;
const float SWEEP_END_DEG = (CENTER_DEG + HALF_TOTAL) - HALF_TOF_FOV;
// ----------------------------------------

int angle270ToMicros(float a) {
  a = constrain(a, 0, 270);
  float t = a / 270.0f;
  return (int)(SERVO_MIN_US + t * (SERVO_MAX_US - SERVO_MIN_US));
}

void gotoAngle(float a) { scanServo.writeMicroseconds(angle270ToMicros(a)); }
uint16_t readToFmm() { return tof.readRangeContinuousMillimeters(); }

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);

  scanServo.attach(SERVO_PIN, SERVO_MIN_US, SERVO_MAX_US);
  pinMode(BUZZER_PIN, OUTPUT);

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
}

void loop() {
  // forward
  for (float ang = SWEEP_START_DEG; ang <= SWEEP_END_DEG; ang += DEG_STEP)
    scanAndPrint(ang);
  // backward
  for (float ang = SWEEP_END_DEG; ang >= SWEEP_START_DEG; ang -= DEG_STEP)
    scanAndPrint(ang);
}

// ---------------- BEEP LOGIC ----------------
void proximityBeep(uint16_t distance) {
  if (distance > ALERT_DISTANCE_MM || distance == 0) {
    noTone(BUZZER_PIN);
    return;
  }

  // Map closer distance → higher frequency (pitch)
  int freq = map(distance, ALERT_DISTANCE_MM, MIN_DISTANCE_MM, 400, 2000);
  freq = constrain(freq, 400, 2000);

  // Volume simulation (via duty cycle): shorter pulse = quieter
  // Active buzzers ignore tone() volume, but for passive, tone() pitch works.
  int beepDuration = map(distance, ALERT_DISTANCE_MM, MIN_DISTANCE_MM, 30, 200);
  beepDuration = constrain(beepDuration, 30, 200);

  tone(BUZZER_PIN, freq, beepDuration);
}
// --------------------------------------------

void scanAndPrint(float ang) {
  gotoAngle(ang);
  delay(SERVO_SETTLE_MS);

  uint16_t mm = readToFmm();

  if (!tof.timeoutOccurred()) {
    Serial.print(ang, 1);
    Serial.print(",");
    Serial.println(mm);

    // Trigger beep if object close
    proximityBeep(mm);

  } else {
    Serial.print(ang, 1);
    Serial.println(",-1");
    noTone(BUZZER_PIN);
  }
}
