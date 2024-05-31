const int sensorInputPin = 33;

int adcValueRead = 0;

void setup()
{
  Serial.begin(115200);
  delay(100);
  Serial.println("Starting...");
}

void loop()
{
  adcValueRead = analogRead(sensorInputPin);
  Serial.println(adcValueRead);
  delay(100);
}