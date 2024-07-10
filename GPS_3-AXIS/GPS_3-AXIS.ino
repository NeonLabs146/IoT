#include <ThingSpeak.h>
#include <WiFi101.h>
#include <TinyGPS++.h>

// Pin definitions
#define LM35_PIN A1
#define triggerPin 5
#define echoPin 4
#define LEDpin 7
#define ADXL335_X A2
#define ADXL335_Y A3
#define ADXL335_Z A4

// WiFi credentials
char ssid[] = "NancyNeon_2.4"; 
char pass[] = "CLB2723C28"; 

// ThingSpeak information
char* writeAPIKey = "M5XPPOTFAYPF2P6L";
const long channelID = 2594975; 

const unsigned int distanceField = 1;
const unsigned int temperatureField = 2;
const unsigned int adxlXField = 3;
const unsigned int adxlYField = 4;
const unsigned int adxlZField = 5;
const unsigned int latitudeField = 6;
const unsigned int longitudeField = 7;

// Other constants
const unsigned long postingInterval = 60L * 1000L; 

// Global variables
unsigned long lastConnectionTime = 0;
long lastUpdateTime = 0;
float distanceThreshold = 100; // Set a default threshold
bool getInfo = 1;
int points = 7; 
WiFiClient client;
TinyGPSPlus gps;

// Function declarations
void connectWifi();
void updateThingSpeak();
float getDistance(int tPin, int ePin);
void readADXL335Data(float &x, float &y, float &z);
float readLM35Temperature();
float getAverageTemperature();

void setup() {
  Serial.begin(9600); 
  Serial1.begin(9600); // Use Serial1 for the GPS module
  pinMode(triggerPin, OUTPUT); 
  pinMode(LEDpin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(LM35_PIN, INPUT);
  pinMode(ADXL335_X, INPUT);
  pinMode(ADXL335_Y, INPUT);
  pinMode(ADXL335_Z, INPUT);

  connectWifi();
  ThingSpeak.begin(client);
  Serial.println("Setup complete");
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    connectWifi();  
  }

  float distance = getDistance(triggerPin, echoPin);  // Get a single measurement
  float temperature = getAverageTemperature();
  float x, y, z;
  readADXL335Data(x, y, z);
  
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (distance >= 0) { 
    if (distance < distanceThreshold) {
      digitalWrite(LEDpin, HIGH);
    } else {
      digitalWrite(LEDpin, LOW); 
    }

    Serial.println("Distance: " + String(distance) + " cm");
    Serial.println("Temperature: " + String(temperature) + " °C");

    if (millis() - lastUpdateTime >= postingInterval) {  
      lastUpdateTime = millis();

      updateThingSpeak(distance, temperature, x, y, z, gps.location.lat(), gps.location.lng());
    }
  } else {
    Serial.println("Invalid distance reading");
  }

  delay(1000);  
}

void updateThingSpeak(float distance, float temperature, float x, float y, float z, float latitude, float longitude) {
  ThingSpeak.setField(distanceField, distance);
  ThingSpeak.setField(temperatureField, temperature);
  ThingSpeak.setField(adxlXField, x);
  ThingSpeak.setField(adxlYField, y);
  ThingSpeak.setField(adxlZField, z);
  if (gps.location.isUpdated()) {
    ThingSpeak.setField(latitudeField, latitude);
    ThingSpeak.setField(longitudeField, longitude);
  }

  int writeSuccess = ThingSpeak.writeFields(channelID, writeAPIKey);
  if (writeSuccess == 200) {
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(writeSuccess));
  }
}

float getDistance(int tPin, int ePin) {
  long duration, distance;

  digitalWrite(tPin, LOW);  
  delayMicroseconds(2); 
  digitalWrite(tPin, HIGH);  
  delayMicroseconds(10); 
  digitalWrite(tPin, LOW);   
  duration = pulseIn(ePin, HIGH);  
  distance = (duration / 2) / 29.1;  

  if (duration == 0) {
    Serial.println("No echo received");
    return -1.0;  
  }

  return distance;
}

void readADXL335Data(float &x, float &y, float &z) {
  x = analogRead(ADXL335_X);
  y = analogRead(ADXL335_Y);
  z = analogRead(ADXL335_Z);

  // Convert to g's
  x = (x - 512) / 102.3;
  y = (y - 512) / 102.3;
  z = (z - 512) / 102.3;

  Serial.print("X: ");
  Serial.print(x);
  Serial.print(" Y: ");
  Serial.print(y);
  Serial.print(" Z: ");
  Serial.println(z);
}

float readLM35Temperature() {
  int reading = analogRead(LM35_PIN);
  float voltage = reading * (3.3 / 1023.0); // MKR1000 uses 3.3V reference voltage
  float temperature = voltage * 100.0;
  return temperature;
}

float getAverageTemperature() {
  float total = 0;
  const int samples = 10;
  for (int i = 0; i < samples; i++) {
    total += readLM35Temperature();
    delay(10);
  }
  return total / samples;
}

void connectWifi() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(ssid, pass);
    delay(2500);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
}
