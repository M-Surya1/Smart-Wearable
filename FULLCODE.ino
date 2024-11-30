#include <Wire.h>
#include <TinyGPS++.h>
#include <DHT.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>  // Include the SpO2 algorithm

#define DHTPIN 2        // Pin where the DHT sensor is connected
#define DHTTYPE DHT11   // DHT sensor type
DHT dht(DHTPIN, DHTTYPE);  // Initialize DHT sensor

TinyGPSPlus gps;       // Create a TinyGPS++ object for GPS data
int ADXL345 = 0x53;    // ADXL345 I2C address
const int flameSensorPin = A0;
float X_out, Y_out, Z_out;
volatile int stepCount = 0;
float stepLength = 0.8;
float distanceCovered = 0.0;
float threshold = 0.2;
bool isStepping = false;

int GPSBaud = 9600;

// MAX30105 sensor for heart rate and SpO2
MAX30105 particleSensor;
unsigned long lastBeat = 0;    // Time of last beat
unsigned long beatDuration = 0; // Time between beats
int beatsPerMinute = 0;
float oxygenSaturation = 0.0;
float pulseAmplitudeRatio = 0.0;

void setup() {
  Serial.begin(9600);    // Serial Monitor
  Serial1.begin(GPSBaud); // GPS on Serial1
  Serial2.begin(115200);  // ESP32 on Serial2
  Wire.begin();           // Start I2C for ADXL345
  dht.begin();            // Start DHT22 sensor
  pinMode(flameSensorPin, INPUT);
  // Configure ADXL345 and calibrate
  configureADXL345();
  calibrateADXL345();

  // Initialize MAX30105 sensor
  Wire1.begin();
  if (!particleSensor.begin(Wire1)) {
    Serial.println("MAX30105 initialization failed!");
    while (1);  // Halt further execution if initialization fails
  }
  Serial.println("MAX30105 initialized successfully");

  particleSensor.setup();  // Basic setup to start reading
}

void configureADXL345() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x2D);  // POWER_CTL register
  Wire.write(8);     // Enable measurement mode
  Wire.endTransmission();
  delay(10);
}

void calibrateADXL345() {
  float numReadings = 500;
  float zSum = 0;

  for (int i = 0; i < numReadings; i++) {
    readADXL345Data();
    zSum += Z_out;
  }

  int Z_offset = (256 - (zSum / numReadings)) / 4;
  Serial.print("Calibration Offset Z= ");
  Serial.println(Z_offset);
  delay(1000);
}

void readADXL345Data() {
  Wire.beginTransmission(ADXL345);
  Wire.write(0x32);  // Start at ACCEL_XOUT_H register
  Wire.endTransmission(false);
  Wire.requestFrom(ADXL345, 6, true);

  X_out = (Wire.read() | Wire.read() << 8) / 256;
  Y_out = (Wire.read() | Wire.read() << 8) / 256;
  Z_out = (Wire.read() | Wire.read() << 8) / 256;
}

void loop() {
  // Read and display GPS data
  while (Serial1.available() > 0) {
    if (gps.encode(Serial1.read())) {
      displayGPSInfo();
    }
  }

  // Read accelerometer data and detect steps
  readADXL345Data();
  detectStep();

  // Read heart rate and SpO2 data
  readHeartRateAndSpO2();

  // Print accelerometer data
  printADXLData();

  // Print step count and distance
  printStepInfo();
  flamesos();
  // Read temperature and humidity from DHT22
 // Serial.println("Reading DHT22 sensor...");
  float temperature = dht.readTemperature();
  float humidity = dht.readHumidity();

  // Display and send DHT22 data
  if (isnan(temperature) || isnan(humidity)) {
    Serial.println("Failed to read from DHT sensor!");
    Serial2.println("Failed to read from DHT sensor!");
  } else {
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    Serial.print("Humidity: ");
    Serial.print(humidity);
    Serial.println(" %");

    Serial2.print("Temperature: ");
    Serial2.print(temperature);
    Serial2.println("°C");
    Serial2.print("Humidity: ");
    Serial2.print(humidity);
    Serial2.println(" %");
  }

  delay(1000);  // Delay for readability and to give DHT22 more time
}

void displayGPSInfo() {
  if (gps.location.isValid()) {
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial.print("Altitude: ");
    Serial.println(gps.altitude.meters());

    Serial2.print("Latitude: ");
    Serial2.println(gps.location.lat(), 6);
    Serial2.print("Longitude: ");
    Serial2.println(gps.location.lng(), 6);
    Serial2.print("Altitude: ");
    Serial2.println(gps.altitude.meters());
  } else {
    Serial.println("Location: Not Available");
    Serial2.println("Location: Not Available");
  }

  Serial.print("Date: ");
  Serial2.print("Date: ");
  if (gps.date.isValid()) {
    Serial.print(gps.date.month());
    Serial.print("/");
    Serial.print(gps.date.day());
    Serial.print("/");
    Serial.println(gps.date.year());

    Serial2.print(gps.date.month());
    Serial2.print("/");
    Serial2.print(gps.date.day());
    Serial2.println(gps.date.year());
  } else {
    Serial.println("Date: Not Available");
    Serial2.println("Date: Not Available");
  }

  Serial.print("Time: ");
  Serial2.print("Time: ");
  if (gps.time.isValid()) {
    printFormattedTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.time.centisecond());
  } else {
    Serial.println("Time: Not Available");
    Serial2.println("Time: Not Available");
  }

  //Serial.println();
  //Serial2.println();
}

void printFormattedTime(int hour, int minute, int second, int centisecond) {
  int offsetHours = 5;    // Adjust this for your timezone
  int offsetMinutes = 30; // Use 0 if no additional minutes offset is needed
  int adjustedHour = hour + offsetHours;
  int adjustedMinute = minute + offsetMinutes;

  // Handle any overflow in minutes
  if (adjustedMinute >= 60) {
    adjustedMinute -= 60;
    adjustedHour++;
  }
  
  // Adjust for hour overflow or underflow
  if (adjustedHour >= 24) adjustedHour -= 24;
  else if (adjustedHour < 0) adjustedHour += 24;

  Serial.print(adjustedHour < 10 ? "0" : "");
  Serial.print(adjustedHour);
  Serial.print(":");
  Serial.print(adjustedMinute < 10 ? "0" : "");
  Serial.print(adjustedMinute);
  Serial.print(":");
  Serial.print(second < 10 ? "0" : "");
  Serial.print(second);
  Serial.print(".");
  Serial.println(centisecond < 10 ? "0" : "");
  Serial.println(centisecond);

  Serial2.print(adjustedHour < 10 ? "0" : "");
  Serial2.print(adjustedHour);
  Serial2.print(":");
  Serial2.print(adjustedMinute < 10 ? "0" : "");
  Serial2.print(adjustedMinute);
  Serial2.print(":");
  Serial2.print(second < 10 ? "0" : "");
  Serial2.print(second);
  Serial2.print(".");
  Serial2.println(centisecond < 10 ? "0" : "");
  Serial2.println(centisecond);
}

void detectStep() {
  if (Z_out > threshold && !isStepping) {
    stepCount++;
    isStepping = true;
  }

  if (Z_out < threshold) {
    isStepping = false;
  }

  distanceCovered = stepCount * stepLength;
}

void printADXLData() {
  Serial.print("Xa: ");
  Serial.println(X_out);
  Serial.print("Ya: ");
  Serial.println(Y_out);
  Serial.print("Za: ");
  Serial.println(Z_out);

  Serial2.print("Xa: ");
  Serial2.println(X_out);
  Serial2.print("Ya: ");
  Serial2.println(Y_out);
  Serial2.print("Za: ");
  Serial2.println(Z_out);
}

void printStepInfo() {
  Serial.print("Step Count: ");
  Serial.println(stepCount);
  Serial.print("Distance Covered (m): ");
  Serial.println(distanceCovered);

  Serial2.print("Step Count: ");
  Serial2.println(stepCount);
  Serial2.print("Distance Covered (m): ");
  Serial2.println(distanceCovered);
}
void flamesos()
{
  int flameDetected = analogRead(flameSensorPin);
  
  // Map the sensor value to a readable range
  int threshold = 300;  // Set the threshold value for flame detection

  if (flameDetected < threshold) {  // If sensor value is below threshold, flame is detected
    Serial.print("Flame Status: ");
    Serial.println("Flame Detected");
    Serial2.print("Flame Status: ");
    Serial2.println("Flame Detected");
    interruptserviceroutine();
  } else {
    Serial.print("Flame Status: ");
    Serial.println("No Flame Detected");
    Serial2.print("Flame Status: ");
    Serial2.println("No Flame Detected");
  }

  delay(1000);  // Wait before the next reading
}
void readHeartRateAndSpO2() {
  long irValue = particleSensor.getIR();  // Get the IR sensor value
  long redValue = particleSensor.getRed();  // Get the red sensor value

  // SpO2 calculation using the ratio of red and infrared light absorption
  if (irValue > 50000 && redValue > 50000) {
    oxygenSaturation = calculateSpO2(redValue, irValue);
    pulseAmplitudeRatio = (float) redValue / (float) irValue;
  }

  // Heart rate calculation
  if (irValue > 50000) {
    if (millis() - lastBeat > 1000) {
      beatDuration = millis() - lastBeat;
      beatsPerMinute = 60000 / beatDuration;
      lastBeat = millis();

      // Output the heart rate and SpO2
      Serial.print("Heart Rate: ");
      Serial.println(beatsPerMinute);
      Serial2.print("Heart Rate: ");
      Serial2.println(beatsPerMinute);
      Serial.print("SpO2: ");
      Serial.print(oxygenSaturation);
      Serial.println(" %");
      Serial2.print("SpO2: ");
      Serial2.print(oxygenSaturation);
      Serial2.println(" %");
    }
  } else {
    Serial.print("Heart Rate: ");
    Serial.println("No pulse detected.");
    Serial2.print("Heart Rate: ");
    Serial2.println("No pulse detected.");
  }
}

float calculateSpO2(long red, long ir) {
  // Simple SpO2 calculation algorithm based on red/IR ratio
  float ratio = (float) red / (float) ir;
  float spo2 = 0.0;

  if (ratio > 0.2) {
    spo2 = 97.5;
  } else if (ratio > 0.1) {
    spo2 = 90.0;
  } else {
    spo2 = 80.0;
  }

  return spo2;
}
void interruptserviceroutine()
{
  int i=0;
  Serial.println("Fire Detected, Sending the Coordinates to the Rescue team ");
  Serial2.println("Fire Detected, Sending the Coordinates to the Rescue team ");
  while(i<100)
  {
  if (gps.location.isValid()) {

    Serial.print("Flame Status: Flame Detected");
    Serial2.print("Flame Status: Flame Detected");
    Serial.print("Latitude: ");
    Serial.println(gps.location.lat(), 6);
    Serial2.print("Latitude: ");
    Serial2.println(gps.location.lat(), 6);
    Serial.print("Longitude: ");
    Serial.println(gps.location.lng(), 6);
    Serial2.print("Longitude: ");
    Serial2.println(gps.location.lng(), 6);
  }
  i++;
  }
  while(true);
}
