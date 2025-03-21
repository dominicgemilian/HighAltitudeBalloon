#include <ICM_20948.h>

#include <Wire.h>

ICM_20948_I2C myICM;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("ICM-20948 Bearing and Neo M9N GPS Example");

  // Initialize I2C
  Wire.begin();

  // Initialize the ICM-20948
  if (myICM.begin(Wire, 0x69) != ICM_20948_Stat_Ok) { // 0x69 is the default I2C address
    Serial.println("ICM-20948 initialization failed");
    while (1);
  }
  Serial.println("ICM-20948 initialized successfully");

  Serial.println("Listening for GPS data...");
}

void loop() {
  // Read GPS data and process it
  while (Serial.available() > 0) {
    char c = Serial.read();
    Serial.print(c);
  }

  Serial.println();
  
  // Update the sensor data
  if (myICM.dataReady()) {
    myICM.getAGMT(); // Updates accelerometer, gyroscope, magnetometer, and temperature data

    // Read magnetometer data
    float magX = myICM.magX();
    float magY = myICM.magY();

    // Calculate the bearing (heading)
    float heading = atan2(magY, magX) * 180.0 / PI; // Convert from radians to degrees
    if (heading < 0) {
      heading += 360.0; // Normalize to 0-360°
    }

    // Print magnetometer data
    // Serial.print("Mag (uT): X=");
    // Serial.print(magX);
    // Serial.print(", Y=");
    // Serial.print(magY);
    // Serial.println();

    // Print the heading (bearing)
    Serial.print("bearing: ");
    Serial.println(heading);

    delay(100);
  }

  delay(1000); // Delay for readability
}
