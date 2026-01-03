#include "config.h"
#include "as5600.h"
#include "stepperMotor.h"
#include <TeensyThreads.h>

AS5600 encoder_1(0, "sensor1");
// AS5600 encoder_2(1, "sensor2");

StepperMotor stepper_1(6, 5, 41, 0xFF, "stepper1", cfg::GEAR_RATIO);

bool motorRunning = true;         // Motor starts ON
bool lastEndstopState = false;    // Track previous state for edge detection

void telemetryTask()
{
  while (true) // Run continuously in a separate thread
  {
    stepper_1.refreshConfigIfNeeded();

    Serial.println();
    Serial.println(F("------Telemetry------"));
    encoder_1.printTelemetry(stepper_1.gear_ratio);
    Serial.println();
    // encoder_2.printTelemetry(stepper_1.gear_ratio);
    Serial.println();
    stepper_1.printTelemetry();
    Serial.println(F("---------------------"));

    threads.delay(1000); // ms
  }
}

void moveStepperToTask(int steps)
{
  stepper_1.stepper.move(steps);
} 
 
void setup()
{
  Serial.begin(cfg::BAUD_RATE);
  // while (!Serial)
  // {
  // }
  Wire.begin();         // I2C0 (SDA0 = pin 18, SCL0 = pin 19 )
  Wire.setClock(1e5);   // 4e5 might block uart

  encoder_1.init();
  // encoder_2.init();
    
  stepper_1.init();
  stepper_1.startEndstopMonitor();
  delay(500); 
  //threads.addThread(telemetryTask);
  
  stepper_1.home(encoder_1);
  stepper_1.moveToDeg(90.0f, encoder_1);
  stepper_1.moveToDeg(0.0f, encoder_1);
  stepper_1.moveToDeg(180.0f, encoder_1);
  stepper_1.moveToDeg(0.0f, encoder_1);
}

void loop()
{
  // TODO: add lost steps compensation using AS5600 feedback
}
