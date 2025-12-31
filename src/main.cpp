#include "config.h"
#include "as5600.h"
#include "stepperMotor.h"
#include <TeensyThreads.h>

AS5600 sensor1(0, "sensor1");
// AS5600 sensor2(1, "sensor2");

StepperMotor stepper1(6, 5, 41, 0xFF, "stepper1", cfg::GEAR_RATIO);

bool motorRunning = true;         // Motor starts ON
bool lastEndstopState = false;    // Track previous state for edge detection

void telemetryTask()
{
  while (true) // Run continuously in a separate thread
  {
    stepper1.refreshConfigIfNeeded();

    Serial.println();
    Serial.println(F("------Telemetry------")); 
    sensor1.printTelemetry(stepper1.gear_ratio);
    Serial.println();
    // sensor2.printTelemetry(stepper1.gear_ratio);
    Serial.println();                                       
    stepper1.printTelemetry();
    Serial.println(F("---------------------"));

    threads.delay(1000); // ms
  }
}

void moveStepperToTask(int steps)
{
  stepper1.stepper.move(steps);
} 
 
void setup()
{
  Serial.begin(cfg::BAUD_RATE);
  while (!Serial)
  {
  }
  Wire.begin();         // I2C0 (SDA0 = pin 18, SCL0 = pin 19 )
  Wire.setClock(1e5);   // 4e5 might block uart

  sensor1.init();
  // sensor2.init();
   
  stepper1.init();
  stepper1.startEndstopMonitor();

  delay(500);
  //threads.addThread(telemetryTask);
  
  stepper1.home();
  stepper1.stepper.moveTo(cfg::MICROSTEPS_PER_DEGREE * 90); 
}

void loop()
{
  stepper1.stepper.run();
}
