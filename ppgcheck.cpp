#include <Wire.h>
#include "MAX30105.h"

MAX30105 particleSensor;

#define SDA_PIN 22
#define SCL_PIN 23

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("MAX30102 not found. Check wiring.");
    while (1);
  }

  particleSensor.setup(
    60,    // LED brightness
    4,     // sample average
    2,     // Red + IR
    100,   // sample rate (100 Hz)
    411,   // pulse width
    4096   // ADC range
  );

  Serial.println("IR");
}

void loop()
{
  long irValue = particleSensor.getIR();

  Serial.println(irValue);

  delay(10);
}
