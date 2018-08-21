ss#include <TinyGPS++.h>
#include <SoftwareSerial.h>

static const int RXPin = 2, TXPin = 3;
static const uint32_t GPSBaud = 9600;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

int pulseSensorPin = 1;
float pulseReading;
int UpperThreshold = 518;
int LowerThreshold = 490;
int BPM = 0;
bool IgnoreReading = false;
bool FirstPulseDetected = false;
unsigned long FirstPulseTime = 0;
unsigned long SecondPulseTime = 0;
unsigned long PulseInterval = 0;

float temp;
int tempPin = 0;

float targetDistance;
float pingTime;
int trigPin = 6;
int echoPin = 7;



void setup() {
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  Serial.begin(115200);
  ss.begin(GPSBaud);

}

void loop() {
  targetDistance = getTargetDistance();
  BPM = getBPM();
  temp = getTemperature();
  float weight = getWeight();
  Serial.print(targetDistance);
  Serial.print(",");
  Serial.print(BPM);
  Serial.print(",");
  Serial.print(temp);
  Serial.print(",");
  /*Serial.print("0.0");
    Serial.print(",");
    Serial.print("0.0");
  */
  getGPS();
  Serial.print(",");
  Serial.println(weight);
  // distance, bpm, temp, lat, lon, weight
  // wifi password: iwheel@2018
}

float getTargetDistance(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(1000);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(15);
  digitalWrite(trigPin, LOW);

  pingTime = pulseIn(echoPin, HIGH);
  targetDistance = (346.0 * pingTime) / 2.0; //in micrometers
  targetDistance = targetDistance * 0.0001;
  delay(500);
  return targetDistance;
}
int getTemperature(){
  temp = analogRead(tempPin);
  temp = temp * (5.0 / 1023.0);
  temp = temp * 1000.0;
  delay(50);
  return int((temp - 500.0) / 10.0);
}

int getBPM(){
  pulseReading = analogRead(pulseSensorPin);

  if (pulseReading > UpperThreshold && IgnoreReading == false) {
    if (FirstPulseDetected == false) {
      FirstPulseTime = millis();
      FirstPulseDetected = true;
    }
    else {
      SecondPulseTime = millis();
      PulseInterval = SecondPulseTime - FirstPulseTime;
      FirstPulseTime = SecondPulseTime;
    }
    IgnoreReading = true;
  }

  // Heart beat trailing edge detected.
  if (pulseReading < LowerThreshold) {
    IgnoreReading = false;
  }

  BPM = (1.0 / PulseInterval) * 60.0 * 1000.0;
  return int(BPM);
}

float getWeight(){
  return 0.0;
}

void getGPS()
{
  if (gps.location.isValid())
  {
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print(gps.location.lng(), 6);
  }
  else
  {
    Serial.print(F("0.0"));
    Serial.print(F(","));
    Serial.print(F("0.0"));
  }
}

  
