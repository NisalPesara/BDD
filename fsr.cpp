#define FSR1_PIN 4
#define FSR2_PIN 5

void setup()
{
  Serial.begin(115200);
  delay(1000);

  analogReadResolution(12);
}

void loop()
{
  int fsr1 = analogRead(FSR1_PIN);
  int fsr2 = analogRead(FSR2_PIN);

  Serial.print("FSR1 = ");
  Serial.print(fsr1);

  Serial.print("    FSR2 = ");
  Serial.println(fsr2);

  delay(200);
}
