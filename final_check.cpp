#include <Wire.h>
#include "MAX30105.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

// ================= I2C =================

#define SDA_PIN 22
#define SCL_PIN 23

// ================= MAX30102 =================

MAX30105 particleSensor;

// ================= ADXL345 =================

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

// ================= SETUP =================

void setup()
{
  Serial.begin(115200);
  delay(1000);

  // Start I2C
  Wire.begin(SDA_PIN, SCL_PIN);

  Serial.println();
  Serial.println("================================");
  Serial.println("PPG + ADXL345 TEST");
  Serial.println("================================");

  // ------------------------------------------------
  // MAX30102
  // ------------------------------------------------

  Serial.println("Initializing MAX30102...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("ERROR: MAX30102 not found!");
    Serial.println("Check VCC, GND, SDA and SCL.");
    
    while (1);
  }

  Serial.println("MAX30102 found!");

  particleSensor.setup(
    60,     // LED brightness
    4,      // Sample averaging
    2,      // Red + IR
    100,    // Sample rate = 100 Hz
    411,    // Pulse width
    4096    // ADC range
  );

  // ------------------------------------------------
  // ADXL345
  // ------------------------------------------------

  Serial.println("Initializing ADXL345...");

  if (!accel.begin())
  {
    Serial.println("ERROR: ADXL345 not found!");
    Serial.println("Check VCC, GND, SDA, SCL and CS.");

    while (1);
  }

  Serial.println("ADXL345 found!");

  // Set accelerometer range
  accel.setRange(ADXL345_RANGE_4_G);

  Serial.println();
  Serial.println("Both sensors initialized.");
  Serial.println();
  Serial.println("Serial Monitor format:");
  Serial.println("IR | RED | X | Y | Z");
  Serial.println();

  delay(1000);
}

// ================= LOOP =================

void loop()
{
  // ------------------------------------------------
  // Read MAX30102
  // ------------------------------------------------

  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // ------------------------------------------------
  // Read ADXL345
  // ------------------------------------------------

  sensors_event_t event;

  accel.getEvent(&event);

  float x = event.acceleration.x;
  float y = event.acceleration.y;
  float z = event.acceleration.z;

  // ------------------------------------------------
  // Serial Monitor
  // ------------------------------------------------

  Serial.print("IR: ");
  Serial.print(irValue);

  Serial.print(" | RED: ");
  Serial.print(redValue);

  Serial.print(" | X: ");
  Serial.print(x, 2);

  Serial.print(" | Y: ");
  Serial.print(y, 2);

  Serial.print(" | Z: ");
  Serial.println(z, 2);

  delay(10);
}
