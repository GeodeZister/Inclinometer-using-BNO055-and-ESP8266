// Include necessary libraries
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>
#include <ESP8266WiFi.h>
#include "ThingSpeak.h"
#include <SHT2x.h>

// WiFi credentials
char ssid[] = "Name";   // your network SSID (name)
char pass[] = "Pass";   // your network password

// ThingSpeak channel information
unsigned long myChannelNumber = "NUMBER";
const char * myWriteAPIKey = "KEY";

// Create an instance of WiFiClient
WiFiClient  client;

// Create an instance of SHT2x (Temperature and Humidity Sensor)
SHT2x sht;

// Variables to hold sensor readings and filter data
float thetaM, phiM, thetaFold = 0, thetaFnew, phiFold = 0, phiFnew, thetaG = 0, phiG = 0, theta, phi, dt;
unsigned long millisOld;

// BNO055 sampling rate delay in milliseconds
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Create an instance of Adafruit_BNO055
Adafruit_BNO055 myIMU = Adafruit_BNO055();

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  
  // Initialize BNO055 IMU sensor
  myIMU.begin();
  
  // Initialize SHT2x temperature and humidity sensor
  sht.begin();
  
  // Initialize I2C (Wire library)
  Wire.begin();
  
  // Small delay to ensure everything is properly set up
  delay(1000);
  
  // Read temperature from BNO055 and use external crystal
  int8_t temp = myIMU.getTemp();
  myIMU.setExtCrystalUse(true);
  
  // Initialize millisOld to current time
  millisOld = millis();
  
  // Set up WiFi mode to station (client)
  WiFi.mode(WIFI_STA);
  
  // Begin ThingSpeak client
  ThingSpeak.begin(client);
}

void loop() {
  // Check Wi-Fi connection status
  if(WiFi.status() != WL_CONNECTED){
    Serial.print("Attempting to connect to SSID: ");
    
    // Try to connect to Wi-Fi
    while(WiFi.status() != WL_CONNECTED){
      WiFi.begin(ssid, pass);
      Serial.print(".");
      delay(5000);
    }
    Serial.println("\nConnected.");
  }
  
  // Get system, gyro, accelerometer calibration status from BNO055
  uint8_t system, gyro, accel, mg = 0;
  myIMU.getCalibration(&system, &gyro, &accel, &mg);
  
  // Get acceleration vector from BNO055
  imu::Vector<3> acc = myIMU.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  
  // Get gyroscope vector from BNO055
  imu::Vector<3> gyr = myIMU.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  
  // Calculate pitch and roll angles from acceleration (in degrees)
  thetaM = -atan2(acc.x()/9.8, acc.z()/9.8) / 2 / PI * 360;
  phiM = -atan2(acc.y()/9.8, acc.z()/9.8) / 2 / PI * 360;
  
  // Apply a low-pass filter to these angles
  phiFnew = .95 * phiFold + .05 * phiM;
  thetaFnew = .95 * thetaFold + .05 * thetaM;
  
  // Calculate the time interval dt
  dt = (millis() - millisOld) / 1000.;
  millisOld = millis();
  
  // Mix gyro data and filtered angle for phi and theta
  theta = (theta + gyr.y() * dt) * .95 + thetaM * .05;
  phi = (phi - gyr.x() * dt) * .95 + phiM * .05;
  
  // Read temperature and humidity data from SHT2x sensor
  sht.read();
  
  // Set fields with sensor data to ThingSpeak
  ThingSpeak.setField(1, theta);
  ThingSpeak.setField(2, phi);
  ThingSpeak.setField(3, sht.getTemperature());
  ThingSpeak.setField(4, sht.getHumidity());
  ThingSpeak.setField(5, (acc.z() / 9.8) * 100);
  ThingSpeak.setField(6, thetaM);
  
  // Update ThingSpeak Channel with data
  int x = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
  
  // Check if update was successful and print result
  if(x == 200){
    Serial.println("Channel update successful.");
  } else {
    Serial.println("Problem updating channel. HTTP error code " + String(x));
  }
  
  // Store filtered angles for the next loop iteration
  phiFold = phiFnew;
  thetaFold = thetaFnew;
  
  // Delay for BNO055 sampling rate
  delay(BNO055_SAMPLERATE_DELAY_MS);
}
