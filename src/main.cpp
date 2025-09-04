#include <TMCStepper.h>
#include <Wire.h>
// I2C pins - SDA (Data), SCL (Clock)
// Wire = SDA0, SCL0
// Wire1 = SDA1, SCL1
// Wire2 = SDA2, SCL2 

// -------------------- User Settings ------------------------------------
#define R_SENSE 0.11f                     // TMC2209 sense resistor
#define DRIVER_ADDRESS 0b00               // UART address
#define SERIAL_PORT Serial1               // Teensy UART port

#define STEP_PIN 2                        // STEP pin
#define DIR_PIN 3                         // DIR pin

#define MOTOR_CURRENT 1000                // RMS current in mA
#define MICROSTEPS 16                     // Fixed microsteps for your board
#define STEPS_PER_REV (200 * MICROSTEPS)  // Full rotation steps
#define STEP_DELAY 120                    // microseconds (us) per step pulse.
 
// -------------------- Initialize driver and encoder --------------------
#define AS5600_ADDRESS 0x36               // Fixed I2C AS5600 address
#define ANGLE_REG_HIGH 0x0C               // Register addresses for angle high byte
#define ANGLE_REG_LOW  0x0D               // Register addresses for angle low byte

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
bool as5600_found = false;

uint16_t scaledAbsPos;

uint16_t calculateScaledAbsPos();

void setup() {
  Serial.begin(115200);         
  SERIAL_PORT.begin(115200);     // TMC2209 UART (might not work if UART broken)
  Wire.begin();                  // I2C0 (SDA0 = pin 18, SCL0 = pin 19 )
  Wire.setClock(400000);
  delay(100);

  // Detecting device logic
  Wire.beginTransmission(AS5600_ADDRESS);
  if (Wire.endTransmission() == 0) {
    Serial.println("AS5600 detected!");
    as5600_found = true;
  } else {
    Serial.println("AS5600 NOT found.");
  }

  // Stepper pins
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, LOW);

  driver.begin();
  driver.rms_current(MOTOR_CURRENT);
  driver.microsteps(MICROSTEPS);

  // Initialize scaled absolute position once
  scaledAbsPos = as5600_found ? calculateScaledAbsPos() : 0xFFFF;
}

uint16_t readAngle() {
  Wire.beginTransmission(AS5600_ADDRESS);
  Wire.write(ANGLE_REG_HIGH);   // Start at high byte
  Wire.endTransmission(false);

  Wire.requestFrom(AS5600_ADDRESS, 2);
  if (Wire.available() == 2) {
    uint8_t high = Wire.read();
    uint8_t low = Wire.read();
    return ((uint16_t)high << 8) | low;
  }
  return 0xFFFF;  // Error code = max 16 bit value
}

uint16_t calculateScaledAbsPos() {
  uint16_t rawAngle = readAngle();
  if (rawAngle != 0xFFFF) {
    // AS5600 gives 12-bit value (0 - 4095)
    uint16_t angle12bit = rawAngle & 0x0FFF;  // ANDing with 0x0FFF keeps the lower 12 bits and sets the upper 4 bits to 0.
    uint32_t mappedMicrosteps = ((uint32_t)angle12bit * (uint32_t)STEPS_PER_REV + 2048u) / 4096u;  // Map to 0-3199 microsteps
    return (uint16_t)mappedMicrosteps;
  }
  return 0xFFFF;  // Error code = max 16 bit value
}

int16_t deltaMicrosteps(uint16_t startSteps, uint16_t endSteps) {
  int16_t delta = endSteps - startSteps;

  // Handle wrap-around
  if (delta > STEPS_PER_REV / 2) delta -= STEPS_PER_REV;
  if (delta < -STEPS_PER_REV / 2) delta += STEPS_PER_REV;

  return delta;
}

static inline void stepPulse() {
  // Generate a step pulse for the stepper motor driver

  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(STEP_DELAY);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(STEP_DELAY);
}

void stepMotor(int steps, bool direction) {
  // This function steps the motor a specified number of steps in a given direction.

  digitalWrite(DIR_PIN, direction ? HIGH : LOW);

  uint16_t microstepBuffer = scaledAbsPos;  // ensure initialized
  int32_t totalSteps = 0;                   // widen to avoid overflow

  // Take a baseline BEFORE the first step so Steps Done starts at 0
  if (as5600_found) {
    uint16_t startPos = calculateScaledAbsPos();
    if (startPos != 0xFFFF) {
      microstepBuffer = startPos;
      scaledAbsPos = startPos;

      float degrees0 = (startPos * 360.0f) / (float)STEPS_PER_REV;
      int32_t stepsDone0 = totalSteps >= 0 ? totalSteps : -totalSteps;
      Serial.print("Scaled absolute position: ");
      Serial.print(startPos);
      Serial.print(" | Steps Done: ");
      Serial.print(stepsDone0);
      Serial.print(" | Degrees: ");
      Serial.println(degrees0, 2);
    }
  }

  for (int i = 0; i < steps; i++) {
    stepPulse();

    // Now sample after actual steps: every 400 steps and at the end
    if (as5600_found && (((i + 1) % 400) == 0 || i == steps - 1)) {
      scaledAbsPos = calculateScaledAbsPos();
      if (scaledAbsPos == 0xFFFF) {
        Serial.println("Error reading angle");
        continue;
      }

      float degrees = (scaledAbsPos * 360.0f) / (float)STEPS_PER_REV;
      totalSteps += deltaMicrosteps(microstepBuffer, scaledAbsPos);
      microstepBuffer = scaledAbsPos;

      int32_t stepsDone = totalSteps >= 0 ? totalSteps : -totalSteps;
      Serial.print("Scaled absolute position: ");
      Serial.print(scaledAbsPos);
      Serial.print(" | Steps Done: ");
      Serial.print(stepsDone);
      Serial.print(" | Degrees: ");
      Serial.println(degrees, 2);
    }
  }
}

void stepMotorWithCompensation(int steps, bool direction) {
  // This function steps the motor with closed-loop compensation using the AS5600 encoder.
  // It adjusts the motor steps based on the encoder feedback to improve accuracy.

  if (!as5600_found) { 
    Serial.println("Step motor with compensation unavaible. AS5600 not found.");
    stepMotor(steps, direction); 
    return; 
  }

  // Baseline before motion
  uint16_t prevPos = calculateScaledAbsPos();
  if (prevPos == 0xFFFF) {
    Serial.println("Step motor with compensation unavaible. Start position error.");
    stepMotor(steps, direction); 
    return; 
  }

  // Closed-loop parameters
  const int      CHUNK_SIZE = 1500;   // Max chunk size is STEPS_PER_REV/2 - 1 but because of quantization/jitter we use a smaller safe chunk <1580
  const int16_t  TOLERANCE = 0;       // microsteps tolerance
  const int16_t  MAX_CORR = 32;       // max correction microsteps per burst
  const int      MAX_FINAL_ITERS = 8;  

  // The "net" values represent the cumulative microsteps commanded and measured during motion.
  int32_t commanded_net = 0;  // Commanded microsteps (net)
  int32_t measured_net  = 0;  // Measured microsteps (net)

  // Start in commanded direction
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);

  Serial.println("Starting scaled absolute position: " + String(prevPos));

  uint16_t chunk_index = 0;
  while (abs(commanded_net) < steps) {
    // 1) Issue next open-loop chunk
    int remaining = steps - abs(commanded_net);
    int chunk = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
    for (int i = 0; i < chunk; i++)
      stepPulse();
    commanded_net += direction ? chunk : -chunk;  

    // 2) Read encoder and accumulate measured delta
    delay(2);   // small settle
    uint16_t newPos = calculateScaledAbsPos();
    if (newPos == 0xFFFF) {
      Serial.println("Error reading new position. Skipping correction.");
      continue;
    }
    int16_t d = deltaMicrosteps(prevPos, newPos);
    measured_net += d;
    prevPos = newPos;

    // 3) Compute error and apply a small correction burst if needed
    int32_t error = commanded_net - measured_net;

    chunk_index++; 
    Serial.print("Chunk " + String(chunk_index) + " Error: " + String(error));
    Serial.println(abs(error) > TOLERANCE ? " Above Tolerance" : " Within Tolerance");

    if (abs(error) > TOLERANCE) {
      bool corrDir = (error > 0);   // Correction direction 
      digitalWrite(DIR_PIN, corrDir ? HIGH : LOW);

      int16_t burst = abs(error) > MAX_CORR ? MAX_CORR : (int16_t)abs(error);
      for (int i = 0; i < burst; i++) 
        stepPulse();

      delay(2);   // small settle
      // Read position after correction and adjust 
      uint16_t pos2 = calculateScaledAbsPos();
      if (pos2 != 0xFFFF) {
        int16_t d2 = deltaMicrosteps(prevPos, pos2);
        measured_net += d2;
        prevPos = pos2;
      }

      // Restore commanded direction
      digitalWrite(DIR_PIN, direction ? HIGH : LOW);
    }
  }

  // Final tighten within tolerance
  int32_t finalError = (direction ? steps : -steps) - measured_net;

  Serial.print("Final error: " + String(finalError));
  Serial.println(abs(finalError) > TOLERANCE ? " Above Tolerance" : " Within Tolerance");

  int iter = 0;
  while (abs(finalError) > TOLERANCE && iter++ < MAX_FINAL_ITERS) {
    bool corrDir = (finalError > 0);    // Correction direction
    digitalWrite(DIR_PIN, corrDir ? HIGH : LOW);

    int16_t burst = abs(finalError) > MAX_CORR ? MAX_CORR : (int16_t)abs(finalError);
    for (int i = 0; i < burst; i++) 
      stepPulse();

    delay(2);   // small settle
    uint16_t pos3 = calculateScaledAbsPos();
    if (pos3 == 0xFFFF) {
      Serial.println("Error reading new position (pos3). Skipping correction.");
      break;
    }

    int16_t d3 = deltaMicrosteps(prevPos, pos3);
    measured_net += d3;
    prevPos = pos3;

    finalError = (direction ? steps : -steps) - measured_net;
  }

  Serial.print("Final scaled absolute position: ");
  Serial.print(calculateScaledAbsPos());
  Serial.print(" | Commanded steps: ");
  Serial.print(steps);
  Serial.print(" | Measured: ");
  Serial.print(abs(measured_net));
  Serial.print(" | Error: ");
  Serial.println((direction ? steps : -steps) - measured_net);
}

inline void printRMSCurrent() {
  Serial.println();
  Serial.print("RMS current: ");
  Serial.print(driver.rms_current());
  Serial.println(" mA");
}

void loop() {
  printRMSCurrent();
  stepMotorWithCompensation(STEPS_PER_REV, true);
  delay(500);

  printRMSCurrent();
  stepMotorWithCompensation(STEPS_PER_REV, false);
  delay(500);
}
