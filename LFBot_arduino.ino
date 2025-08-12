// --- Include necessary libraries ---
#include <QTRSensors.h>        // For Pololu QTR reflectance sensor array
#include <SparkFun_TB6612.h>   // For TB6612FNG motor driver
#include <SoftwareSerial.h>    // For Bluetooth (HC-05) communication

// --- Bluetooth module setup ---
SoftwareSerial HC05(2, 13); // RX pin = 2, TX pin = 13
char data;                  // Variable to store incoming Bluetooth data

// --- QTR Line Sensor setup ---
QTRSensors qtr;
const uint8_t SensorCount = 8;        // We have 8 sensors in the array
uint16_t sensorValues[SensorCount];   // Stores the latest sensor readings

// --- PID Control variables ---
float Kp = 0.09;     // Proportional gain
float Ki = 0.0001;   // Integral gain
float Kd = 0.33;     // Derivative gain
int P, I, D;         // PID terms
int lastError = 0;   // For derivative calculation

// --- Turning settings ---
int TURN_SPEED = 200;               // Speed when making turns
const int BLACK_THRESHOLD = 700;    // Value above which sensor detects black line
bool turning = false;               // Whether bot is currently turning
String turnDirection = "";          // Direction of current turn ("left" or "right")

// --- Speed limits ---
uint8_t maxspeeda = 225;     // Max speed for motor A
uint8_t maxspeedb = 225;     // Max speed for motor B
uint8_t basespeeda = 175;    // Normal cruising speed for motor A
uint8_t basespeedb = 175;    // Normal cruising speed for motor B

// --- Motor driver pin mapping ---
#define AIN1 3
#define BIN1 7
#define AIN2 4
#define BIN2 8
#define PWMA 5
#define PWMB 6
#define STBY 9

// --- Button pins ---
int buttoncalibrate = 10;  // Button to start sensor calibration
int buttonstart = 11;      // Button to start/stop the bot

// --- Motor driver setup ---
const int offsetA = 1;  // Direction correction for motor A
const int offsetB = 1;  // Direction correction for motor B
Motor motor1 = Motor(AIN1, AIN2, PWMA, offsetA, STBY);
Motor motor2 = Motor(BIN1, BIN2, PWMB, offsetB, STBY);

// --- Ramp-up speed control ---
uint8_t currentSpeedA = 0;
uint8_t currentSpeedB = 0;
const uint8_t rampStep = 5;       // Speed increment step
const uint16_t rampDelay = 15;    // Delay between each ramp step (ms)
unsigned long lastRampTime = 0;   // Last time we increased speed

boolean onoff = false;  // Whether bot is active or stopped


// -------------------- SETUP --------------------
void setup() {
  // Configure QTR sensor array
  qtr.setTypeAnalog();
  qtr.setSensorPins((const uint8_t[]){A0, A1, A2, A3, A4, A5, A6, A7}, SensorCount);
  qtr.setEmitterPin(12); // LED emitter pin

  // Start Bluetooth and Serial
  HC05.begin(9600);
  Serial.begin(9600);

  // Setup pins
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(buttoncalibrate, INPUT);
  pinMode(buttonstart, INPUT);

  // Wait until calibration button is pressed
  boolean Ok = false;
  while (!Ok) {
    if (digitalRead(buttoncalibrate) == HIGH) {
      delay(2000);     // Short delay before starting
      calibration();   // Calibrate sensors
      Ok = true;
    }
  }
  brake(motor1, motor2); // Make sure bot is stopped initially
}


// -------------------- CALIBRATION --------------------
// Spins the bot in both directions to let sensors "see" the line
void calibration() {
  digitalWrite(LED_BUILTIN, HIGH); // LED ON during calibration

  // Rotate right and calibrate
  for (uint16_t i = 0; i < 50; i++) {
    motor1.drive(25);
    motor2.drive(-25);
    qtr.calibrate();
    delay(20);
  }

  // Rotate left and calibrate
  for (uint16_t i = 0; i < 50; i++) {
    motor1.drive(-25);
    motor2.drive(25);
    qtr.calibrate();
    delay(20);
  }

  brake(motor1, motor2); // Stop motors
  digitalWrite(LED_BUILTIN, LOW); // LED OFF after calibration
}


// -------------------- LOOP --------------------
void loop() {
  // --- Handle Bluetooth Commands ---
  if (HC05.available()) {
    data = HC05.read();

    // Adjust PID values
    if (data == 'i') Kp += 0.005;
    else if (data == 'd') Kp -= 0.005;
    else if (data == 'j') Ki += 0.00005;
    else if (data == 'e') Ki -= 0.00005;
    else if (data == 'k') Kd += 0.01;
    else if (data == 'f') Kd -= 0.01;

    // Adjust speeds
    else if (data == 'l') { maxspeeda += 25; maxspeedb += 25; basespeeda += 25; basespeedb += 25; }
    else if (data == 'g') { maxspeeda -= 25; maxspeedb -= 25; basespeeda -= 25; basespeedb -= 25; }

    // Start/stop bot from Bluetooth
    else if (data == 's') { onoff = false; P = I = D = 0; }  // Stop
    else if (data == 'S') { onoff = true; currentSpeedA = 0; currentSpeedB = 0; lastRampTime = millis(); } // Start

    // Adjust turning speed
    else if (data == 't') TURN_SPEED += 10;
    else if (data == 'T') TURN_SPEED -= 10;

    // Send updated PID & speed values back to Bluetooth
    HC05.print(Kp); HC05.print(" ");
    HC05.print(Ki); HC05.print(" ");
    HC05.println(Kd);
    HC05.print(basespeeda); HC05.print(" ");
    HC05.print(maxspeeda);
  }

  // --- Handle Start/Stop Button ---
  if (digitalRead(buttonstart) == HIGH) {
    onoff = !onoff; // Toggle state
    delay(500); // Debounce delay

    // If starting, reset ramp-up
    if (onoff) {
      currentSpeedA = 0;
      currentSpeedB = 0;
      lastRampTime = millis();
    }
  }

  // --- If bot is ON, move forward with ramp-up speed ---
  if (onoff) {
    if (millis() - lastRampTime >= rampDelay) {
      lastRampTime = millis();
      if (currentSpeedA < basespeeda) currentSpeedA += rampStep;
      if (currentSpeedB < basespeedb) currentSpeedB += rampStep;
    }
    PID_control(currentSpeedA, currentSpeedB);
  } else {
    brake(motor1, motor2); // Stop motors if off
  }
}


// -------------------- MOTOR CONTROL FUNCTIONS --------------------
void forward_brake(int posa, int posb) {
  motor1.drive(posa);
  motor2.drive(posb);
}

// PID line-following logic
void PID_control(uint8_t baseA, uint8_t baseB) {
  // Get current position of the line (0 = far left, 7000 = far right)
  uint16_t position = qtr.readLineBlack(sensorValues);
  int error = 3500 - position; // 3500 = center of the line

  // PID calculations
  P = error;
  I += error;
  D = error - lastError;
  lastError = error;

  // Compute motor speed adjustments
  int motorspeed = P * Kp + I * Ki + D * Kd;
  int motorspeeda = baseA + motorspeed;
  int motorspeedb = baseB - motorspeed;

  // Limit speeds to max values
  motorspeeda = constrain(motorspeeda, 0, maxspeeda);
  motorspeedb = constrain(motorspeedb, 0, maxspeedb);

  // Read raw sensor values for turn detection
  qtr.read(sensorValues);

  // Determine where the line is
  bool lineCentered = (sensorValues[3] > BLACK_THRESHOLD || sensorValues[4] > BLACK_THRESHOLD);
  bool leftTurn = (sensorValues[0] > BLACK_THRESHOLD || sensorValues[1] > BLACK_THRESHOLD);
  bool rightTurn = (sensorValues[6] > BLACK_THRESHOLD || sensorValues[7] > BLACK_THRESHOLD);

  // --- If currently turning ---
  if (turning) {
    // Stop turning when back on track
    if (lineCentered) {
      turning = false;
      turnDirection = "";
    } else {
      // Continue the turn
      if (turnDirection == "left") {
        motor1.drive(TURN_SPEED);
        motor2.drive(-(TURN_SPEED * 0.30));
      } else if (turnDirection == "right") {
        motor1.drive(-(TURN_SPEED * 0.30));
        motor2.drive(TURN_SPEED);
      }
      return; // Skip rest of function until turn finishes
    }
  }

  // --- Detect and handle turns ---
  if (leftTurn && !rightTurn) {
    motor1.drive(-30);
    motor2.drive(-30);
    delay(50);
    turning = true;
    turnDirection = "left";
  } 
  else if (rightTurn && !leftTurn) {
    motor1.drive(-30);
    motor2.drive(-30);
    delay(50);
    turning = true;
    turnDirection = "right";
  } 
  else {
    // Normal forward movement
    forward_brake(motorspeeda, motorspeedb);
  }
}
