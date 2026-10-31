#include <Arduino.h>
#include <SimpleFOC.h>

// PAN ONLY (tilt disconnected)
// PAN (bottom): driver D32/D33/D18/D19
// The PC sends a SPEED for pan ~20-30 times a second ("V" command).
// The ESP32 ramps smoothly to that speed and glides the head.
// If the PC stops talking for 0.3 s, pan stops by itself (safety).

BLDCMotor pan = BLDCMotor(7);
BLDCDriver3PWM panDriver = BLDCDriver3PWM(32, 33, 18, 19);

const float PAN_MOVE_V = 4.5;    // power while turning. Skips/buzzes -> 5. Never above 5.
const float PAN_HOLD_V = 1.0;    // power while still
const float PAN_R      = 5.0;    // ohms per phase (for the power estimate) - your measured value

const float PAN_LIMIT = 3.0;     // radians (~170 deg each way) - hard safety limit
const float MAX_VEL   = 1.5;     // rad/s (~86 deg/s) top pan speed
const float ACCEL     = 6.0;     // rad/s^2 - how fast it speeds up/slows down. Lower = softer.

float cmdVel = 0, curVel = 0;
unsigned long lastCmd = 0, lastStatus = 0, lastMicros = 0, panLastMove = 0;

Commander command = Commander(Serial);

// V<rad/s>  pan speed (sent continuously while tracking)
void doVel(char* cmd) { cmdVel = constrain(strtof(cmd, nullptr), -MAX_VEL, MAX_VEL); lastCmd = millis(); }
// X<rad>    pan go-to angle (startup check, centering, a/d keys)
void doAim(char* cmd) { cmdVel = 0; curVel = 0; pan.target = constrain(strtof(cmd, nullptr), -PAN_LIMIT, PAN_LIMIT); }

void setup() {
  Serial.begin(115200);
  delay(1000);

  panDriver.voltage_power_supply = 12;
  panDriver.init();

  pan.linkDriver(&panDriver);
  pan.controller = MotionControlType::angle_openloop;
  pan.voltage_limit = PAN_HOLD_V;
  pan.velocity_limit = MAX_VEL;
  pan.init();
  pan.target = 0;

  command.add('V', doVel, "pan speed");
  command.add('X', doAim, "pan angle");
  lastMicros = micros();
  Serial.println("Ready");
}

void loop() {
  unsigned long now = micros();
  float dt = (now - lastMicros) * 1e-6f;
  lastMicros = now;
  if (dt > 0.05f) dt = 0.05f;

  if (millis() - lastCmd > 300) cmdVel = 0;          // PC went quiet -> stop

  // ramp the speed smoothly toward what the PC asked for, then glide the target along
  float maxDv = ACCEL * dt;
  curVel += constrain(cmdVel - curVel, -maxDv, maxDv);
  if (fabs(curVel) > 1e-4f) {
    pan.target += curVel * dt;
    if (pan.target >  PAN_LIMIT) { pan.target =  PAN_LIMIT; curVel = 0; }
    if (pan.target < -PAN_LIMIT) { pan.target = -PAN_LIMIT; curVel = 0; }
  }

  // full power while moving (and 0.3 s after), low holding power when still
  if (fabs(curVel) > 1e-3f || fabs(pan.target - pan.shaft_angle) > 0.005f) panLastMove = millis();
  pan.voltage_limit = (millis() - panLastMove < 300) ? PAN_MOVE_V : PAN_HOLD_V;

  pan.move();
  command.run();

  // status 20x a second: pan angle (rad), estimated pan power (W)
  if (millis() - lastStatus > 50) {
    lastStatus = millis();
    float pp = 1.5f * pan.voltage_limit * pan.voltage_limit / PAN_R;
    Serial.printf("S %.4f %.2f\n", pan.shaft_angle, pp);
  }
}