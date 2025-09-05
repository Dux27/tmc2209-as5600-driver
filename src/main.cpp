#include <TMCStepper.h>
#include <Wire.h>
#include <ctime>
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
uint16_t STEP_DELAY = 140;                // microseconds (us) per step pulse.
bool ACCELERATE = true;                  // Whether to use acceleration profile 

// -------------------- Initialize driver and encoder --------------------
#define AS5600_ADDRESS 0x36               // Fixed I2C AS5600 address
#define ANGLE_REG_HIGH 0x0C               // Register addresses for angle high byte
#define ANGLE_REG_LOW  0x0D               // Register addresses for angle low byte

TMC2209Stepper driver(&SERIAL_PORT, R_SENSE, DRIVER_ADDRESS);
bool as5600_found = false;

uint16_t scaled_abs_pos;

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
  scaled_abs_pos = as5600_found ? calculateScaledAbsPos() : 0xFFFF;
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
  uint16_t raw_angle = readAngle();
  if (raw_angle != 0xFFFF) {
    // AS5600 gives 12-bit value (0 - 4095)
    uint16_t angle_12bit = raw_angle & 0x0FFF;  // ANDing with 0x0FFF keeps the lower 12 bits and sets the upper 4 bits to 0.
    uint32_t mapped_microsteps = ((uint32_t)angle_12bit * (uint32_t)STEPS_PER_REV + 2048u) / 4096u;  // Map to 0-3199 microsteps
    return (uint16_t)mapped_microsteps;
  }
  return 0xFFFF;  // Error code = max 16 bit value
}

int16_t deltaMicrosteps(uint16_t start_steps, uint16_t end_steps) {
  int16_t delta = end_steps - start_steps;

  // Handle wrap-around
  if (delta > STEPS_PER_REV / 2) delta -= STEPS_PER_REV;
  if (delta < -STEPS_PER_REV / 2) delta += STEPS_PER_REV;

  return delta;
}

void stepAcc(uint32_t commanded_steps, uint32_t step_index) {
  // This function do trapzoidal acceleration profile
  // It modifies the global STEP_DELAY variable based on the commanded steps and current step index
  
  if (!ACCELERATE) return;
  // microseconds (us)
  const uint16_t MAX_STEP_DELAY = 160;      // 6.25 kHz
  const uint16_t MIN_STEP_DELAY = 80;       // 12.5 kHZ
  uint16_t acc_ramp = MICROSTEPS * 50;      // Steps in the ramp

  uint32_t mid_point = commanded_steps / 2u;
  if (acc_ramp > mid_point) acc_ramp = mid_point;

  // Guard for very short moves
  if (commanded_steps == 0u || acc_ramp == 0u) {
    STEP_DELAY = MIN_STEP_DELAY;
    return;
  }

  if (step_index < acc_ramp) {
    // Accelerate 
    STEP_DELAY = MAX_STEP_DELAY - ((MAX_STEP_DELAY - MIN_STEP_DELAY) * (step_index + 1u)) / acc_ramp;
  } else if (step_index >= commanded_steps - acc_ramp) {
    // Decelerate 
    uint32_t decelerate_step_index = (commanded_steps - 1u) - step_index; // 0..acc_ramp-1 from end
    STEP_DELAY = MIN_STEP_DELAY + ((MAX_STEP_DELAY - MIN_STEP_DELAY) * ((acc_ramp - 1u) - decelerate_step_index)) / acc_ramp;
  } else {
    STEP_DELAY = MIN_STEP_DELAY;
  }

  // Don't exceed limits for secure purposes
  if (STEP_DELAY < MIN_STEP_DELAY) STEP_DELAY = MIN_STEP_DELAY;
  if (STEP_DELAY > MAX_STEP_DELAY) STEP_DELAY = MAX_STEP_DELAY;
}

float calcRPM(uint32_t start_time, uint32_t end_time, uint32_t commanded_steps) {
  // This function take time in microseconds

  uint32_t delta = end_time - start_time;
  if (delta == 0) return 0.0f;

  float rotations = float(commanded_steps) / float(STEPS_PER_REV);
  float RPM = rotations / (float(delta) / 60.0f / 1.0e6f);

  return RPM;
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

  uint16_t microstep_buffer = scaled_abs_pos;  // ensure initialized
  int32_t total_steps = 0;                     // widen to avoid overflow

  // Take a baseline BEFORE the first step so Steps Done starts at 0
  if (as5600_found) {
    uint16_t start_pos = calculateScaledAbsPos();
    if (start_pos != 0xFFFF) {
      microstep_buffer = start_pos;
      scaled_abs_pos = start_pos;

      float degrees_0 = (start_pos * 360.0f) / (float)STEPS_PER_REV;
      int32_t steps_done_0 = total_steps >= 0 ? total_steps : -total_steps;
      Serial.print("Scaled absolute position: ");
      Serial.print(start_pos);
      Serial.print(" | Steps Done: ");
      Serial.print(steps_done_0);
      Serial.print(" | Degrees: ");
      Serial.println(degrees_0, 2);
    }
  }

  uint32_t start_time = micros();

  for (int i = 0; i < steps; i++) {
    stepAcc(steps, i);
    stepPulse();

    // Now sample after actual steps: every 400 steps and at the end
    if (as5600_found && (((i + 1) % 400) == 0 || i == steps - 1)) {
      scaled_abs_pos = calculateScaledAbsPos();
      if (scaled_abs_pos == 0xFFFF) {
        Serial.println("Error reading angle");
        continue;
      }

      float degrees = (scaled_abs_pos * 360.0f) / (float)STEPS_PER_REV;
      total_steps += deltaMicrosteps(microstep_buffer, scaled_abs_pos);
      microstep_buffer = scaled_abs_pos;

      int32_t steps_done = total_steps >= 0 ? total_steps : -total_steps;
      Serial.print("Scaled absolute position: ");
      Serial.print(scaled_abs_pos);
      Serial.print(" | Steps Done: ");
      Serial.print(steps_done);
      Serial.print(" | Degrees: ");
      Serial.println(degrees, 2);
    }
  }
  uint32_t end_time = micros();
  float RPM = calcRPM(start_time, end_time, steps);
  Serial.print("RPM: ");
  Serial.println(RPM, 2); 
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
  uint16_t prev_pos = calculateScaledAbsPos();
  if (prev_pos == 0xFFFF) {
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

  Serial.println("Starting scaled absolute position: " + String(prev_pos));

  uint32_t start_time = micros();

  uint16_t chunk_index = 0;
  while (abs(commanded_net) < steps) {
    // 1) Issue next open-loop chunk
    int remaining = steps - abs(commanded_net);
    int chunk = remaining < CHUNK_SIZE ? remaining : CHUNK_SIZE;
    for (int i = 0; i < chunk; i++) {
      stepAcc(chunk, i);
      stepPulse();
    }
      
    commanded_net += direction ? chunk : -chunk;  

    // 2) Read encoder and accumulate measured delta
    delay(2);   // small settle
    uint16_t new_pos = calculateScaledAbsPos();
    if (new_pos == 0xFFFF) {
      Serial.println("Error reading new position. Skipping correction.");
      continue;
    }
    int16_t d = deltaMicrosteps(prev_pos, new_pos);
    measured_net += d;
    prev_pos = new_pos;

    // 3) Compute error and apply a small correction burst if needed
    int32_t error = commanded_net - measured_net;

    chunk_index++; 
    Serial.print("Chunk " + String(chunk_index) + " Error: " + String(error));
    Serial.println(abs(error) > TOLERANCE ? " Above Tolerance" : " Within Tolerance");

    if (abs(error) > TOLERANCE) {
      bool corr_dir = (error > 0);   // Correction direction 
      digitalWrite(DIR_PIN, corr_dir ? HIGH : LOW);

      int16_t burst = abs(error) > MAX_CORR ? MAX_CORR : (int16_t)abs(error);
      for (int i = 0; i < burst; i++) {
        stepAcc(steps, i);
        stepPulse();
      }

      delay(2);   // small settle
      // Read position after correction and adjust 
      uint16_t pos_2 = calculateScaledAbsPos();
      if (pos_2 != 0xFFFF) {
        int16_t d_2 = deltaMicrosteps(prev_pos, pos_2);
        measured_net += d_2;
        prev_pos = pos_2;
      }

      // Restore commanded direction
      digitalWrite(DIR_PIN, direction ? HIGH : LOW);
    }
  }

  // Final tighten within tolerance
  int32_t final_error = (direction ? steps : -steps) - measured_net;

  Serial.print("Final error: " + String(final_error));
  Serial.println(abs(final_error) > TOLERANCE ? " Above Tolerance" : " Within Tolerance");

  int iter = 0;
  while (abs(final_error) > TOLERANCE && iter++ < MAX_FINAL_ITERS) {
    bool corr_dir = (final_error > 0);    // Correction direction
    digitalWrite(DIR_PIN, corr_dir ? HIGH : LOW);

    int16_t burst = abs(final_error) > MAX_CORR ? MAX_CORR : (int16_t)abs(final_error);
    for (int i = 0; i < burst; i++) {
      stepAcc(burst, i);
      stepPulse();
    }

    delay(2);   // small settle
    uint16_t pos_3 = calculateScaledAbsPos();
    if (pos_3 == 0xFFFF) {
      Serial.println("Error reading new position (pos3). Skipping correction.");
      break;
    }

    int16_t d_3 = deltaMicrosteps(prev_pos, pos_3);
    measured_net += d_3;
    prev_pos = pos_3;

    final_error = (direction ? steps : -steps) - measured_net;
  }

  Serial.print("Final scaled absolute position: ");
  Serial.print(calculateScaledAbsPos());
  Serial.print(" | Commanded steps: ");
  Serial.print(steps);
  Serial.print(" | Measured: ");
  Serial.print(abs(measured_net));
  Serial.print(" | Error: ");
  Serial.println((direction ? steps : -steps) - measured_net);

  uint32_t end_time = micros();
  float RPM = calcRPM(start_time, end_time, steps);
  Serial.print("RPM: ");
  Serial.println(RPM, 2); 
}

inline void printRMSCurrent() {
  Serial.println();
  Serial.print("RMS current: ");
  Serial.print(driver.rms_current());
  Serial.println(" mA");
}

void loop() {
  printRMSCurrent();
  stepMotorWithCompensation(4 * STEPS_PER_REV, true);
  delay(500);

  printRMSCurrent();
  stepMotor(4 * STEPS_PER_REV, false);
  delay(500);
}
