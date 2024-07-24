#include <BluetoothSerial.h>

// Bluetooth serial object
BluetoothSerial SerialBT;

// Motor control pins
const int motor1Pin1 = 2;   // P2 -> IN1
const int motor1Pin2 = 4;   // P4 -> IN2
const int motor2Pin1 = 5;   // P5 -> IN3
const int motor2Pin2 = 18;  // P18 -> IN4

// Target coordinates
const float targetLat = 37.7749;    // Replace with your target latitude
const float targetLon = -122.4194;  // Replace with your target longitude

void setup() {
  Serial.begin(115200);                // For debugging
  if (!SerialBT.begin("ESP32test")) {  // Bluetooth device name
    Serial.println("An error occurred initializing Bluetooth");
  } else {
    Serial.println("ESP32 Bluetooth is ready to receive coordinates.");
  }

  // Initialize motor control pins
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);

  // Set initial motor states
  stopMotors();
}

void loop() {
  if (SerialBT.hasClient()) {
    if (SerialBT.available()) {
      String data = SerialBT.readStringUntil('\n');
      data.trim();
      Serial.print("Received data: ");
      Serial.println(data);
      int commaIndex = data.indexOf(',');
      if (commaIndex > -1) {
        String latString = data.substring(0, commaIndex);
        String lonString = data.substring(commaIndex + 1);
        Serial.print("Latitude String: ");
        Serial.println(latString);
        Serial.print("Longitude String: ");
        Serial.println(lonString);
        float lat = latString.toFloat();
        float lon = lonString.toFloat();
        Serial.print("Parsed coordinates: ");
        Serial.print(lat);
        Serial.print(", ");
        Serial.println(lon);

        // Calculate the difference and control the motors
        controlMotors(lat, lon);
      } else {
        Serial.println("Invalid data format.");
      }
    }
  } else {
    Serial.println("Waiting for Bluetooth client...");
    delay(1000);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);
  }
}

void controlMotors(float lat, float lon) {
  float latDiff = targetLat - lat;
  float lonDiff = targetLon - lon;

  Serial.print("Latitude difference: ");
  Serial.println(latDiff);
  Serial.print("Longitude difference: ");
  Serial.println(lonDiff);

  // Control the motors based on the difference
  if (abs(latDiff) > 0.0001) {
    if (latDiff > 0) {
      driveForward();
    } else {
      driveBackward();
    }
  } else if (abs(lonDiff) > 0.0001) {
    if (lonDiff > 0) {
      turnLeft();
    } else {
      turnRight();
    }
  } else {
    stopMotors();
  }
}

void driveForward() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  Serial.println("FORWARD");
}

void driveBackward() {

  Serial.println("Driving backward");
}

void turnLeft() {
  digitalWrite(motor1Pin1, HIGH);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, HIGH);
  Serial.println("Turning left");
}

void turnRight() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, HIGH);
  digitalWrite(motor2Pin1, HIGH);
  digitalWrite(motor2Pin2, LOW);
  Serial.println("Turning right");
}

void stopMotors() {
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  Serial.println("Stopping motors");
}
