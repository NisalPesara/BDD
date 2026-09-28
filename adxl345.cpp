#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println("Initializing ADXL345...");

  if (!accel.begin())
  {
    Serial.println("ADXL345 not found. Check wiring.");
    while (1);
  }

  Serial.println("ADXL345 found!");

  accel.setRange(ADXL345_RANGE_4_G);

  Serial.println("X,Y,Z");
}

void loop()
{
  sensors_event_t event;

  accel.getEvent(&event);

  Serial.print(event.acceleration.x);
  Serial.print(",");

  Serial.print(event.acceleration.y);
  Serial.print(",");

  Serial.println(event.acceleration.z);

  delay(10);
}
