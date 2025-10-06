#include <Arduino.h>
#include "config.h"
#include "as5600.h"
#include "stepperMotor.h"

AS5600 sensor1(0x36, "sensor1");
StepperMotor stepper1(6, 5, 0xFF, "stepper1");

void setup()
{
  Serial.begin(cfg::BAUD_RATE);
  while (!Serial) {}
  Wire.begin();                 // I2C0 (SDA0 = pin 18, SCL0 = pin 19 )
  Wire.setClock(1e5);           // 4e5 might block uart 

  sensor1.init();

  stepper1.init();
  stepper1.stepper.moveTo(35.4444 * cfg::STEPS_PER_REV);

  delay(500);   // small delay to ensure driver is ready
}

void loop()
{
  stepper1.stepper.run();

  static uint32_t last = 0;
  uint32_t now = millis();
  if (now - last >= 2000)
  {
    stepper1.printTelemetry();

    Serial.print(F("[AS5600] "));
    Serial.print(sensor1.name);
    Serial.print(F(" - raw absolute position: "));
    Serial.println(sensor1.readAbsPosition());
    Serial.print(F("[AS5600] "));
    Serial.print(sensor1.name);
    Serial.print(F(" - scaled absolute position: "));
    Serial.println(sensor1.calcMappedAbsPosition());

    last = now;
  }
}
