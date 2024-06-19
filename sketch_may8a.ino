#include "DHT.h"
#include <SoftwareSerial.h>
#include <Servo.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <TinyGPS++.h>

#define DHT_PIN A3
#define DHT_TYPE DHT11
#define MQ2pin A0
#define IR_SENSOR_PIN 4
#define PWR_SENSOR_PIN A1
#define SERVO_PIN_1 5
#define SERVO_PIN_2 6
#define BUZZER_PIN 9 // Choose any digital pin for the buzzer
#define GAS_ALERT_THRESHOLD 600
#define TEMPERATURE_ALERT_THRESHOLD 38

#define BLUETOOTH_RX_PIN 2
#define BLUETOOTH_TX_PIN 3

DHT dht(DHT_PIN, DHT_TYPE);
SoftwareSerial bluetoothSerial(BLUETOOTH_RX_PIN, BLUETOOTH_TX_PIN); // SoftwareSerial for Bluetooth communication
Servo motor1;
Servo motor2;
LiquidCrystal_I2C lcd(0x27, 16, 2);
TinyGPSPlus gps;

bool gasAlert = false;
bool tempAlert = false;

void soundBuzzer(bool state) {
  if (state) {
    tone(BUZZER_PIN, 1000);
  } else {
    noTone(BUZZER_PIN);
  }
}

void setup() {
  Serial.begin(9600);
  bluetoothSerial.begin(9600); // Initialize Bluetooth serial communication
  dht.begin();
  pinMode(IR_SENSOR_PIN, INPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  motor1.attach(SERVO_PIN_1);
  motor2.attach(SERVO_PIN_2);
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Let's Start Work");
  delay(2000);
  lcd.clear();
  
  // Initially set both motors to 90 degrees
  motor1.write(90);
  motor2.write(90);
}

void loop() {
  
  // Read temperature sensor
  float temperature = dht.readTemperature();

  // Check if temperature reading is valid
  if (isnan(temperature)) {
    Serial.println("Failed to read from DHT sensor!");
    return;
  }

  // Read gas sensor value
  int sensorValue = analogRead(MQ2pin);
  // Read IR sensor state
  int irState = digitalRead(IR_SENSOR_PIN);
  // Read power sensor value
  int pwrSensorValue = analogRead(PWR_SENSOR_PIN);

  // Temperature alert handling
  if (temperature > TEMPERATURE_ALERT_THRESHOLD && !tempAlert) {
    Serial.println("ALERT: High Temperature Detected!");
    lcd.setCursor(0, 0);
    lcd.print("ALERT: High Temp!   ");
    bluetoothSerial.println("ALERT: High Temperature Detected!");
    tempAlert = true;
    gasAlert = false;
    soundBuzzer(true);
    motor1.write(0); // Move motor 1 to 0 degrees
    motor2.write(0); // Move motor 2 to 0 degrees
    // Print latitude and longitude when alert occurs
    Serial.print("Latitude: ");
    Serial.print(29.9056482, 6);
    Serial.print(" Longitude: ");
    Serial.println(77.8305253, 6);
  } else if (temperature <= TEMPERATURE_ALERT_THRESHOLD && tempAlert) {
    tempAlert = false;
    lcd.setCursor(0, 0);
    lcd.print("Tem Alert OVER");
    bluetoothSerial.println("Temperature alert cleared.");
    soundBuzzer(false);
    motor1.write(90); // Move motor 1 back to 90 degrees
    motor2.write(90); // Move motor 2 back to 90 degrees
  }

  // Gas alert handling
  if (sensorValue > GAS_ALERT_THRESHOLD && !gasAlert) {
    Serial.println("ALERT: Gas detected!");
    lcd.setCursor(0, 1);
    lcd.print("ALERT: Move Back!   ");
    bluetoothSerial.println("ALERT: Gas detected!");
    gasAlert = true;
    tempAlert = false;
    soundBuzzer(true);
    motor1.write(0); // Move motor 1 to 0 degrees
    motor2.write(0); // Move motor 2 to 0 degrees
    
  } else if (sensorValue <= GAS_ALERT_THRESHOLD && gasAlert) {
    gasAlert = false;
    lcd.setCursor(0, 1);
    lcd.print("GAS Alert OVER...");
    bluetoothSerial.println("Gas alert cleared.");
    soundBuzzer(false);
    motor1.write(90); // Move motor 1 back to 90 degrees
    motor2.write(90); // Move motor 2 back to 90 degrees
  }

  // Send sensor data to mobile via Bluetooth
  bluetoothSerial.print("Temperature: ");
  bluetoothSerial.print(temperature);
  bluetoothSerial.print(" C, Gas Sensor Value: ");
  bluetoothSerial.print(sensorValue);
  bluetoothSerial.print(", IR Sensor State: ");
  bluetoothSerial.print(irState);
  bluetoothSerial.print(", Power Sensor Value: ");
  bluetoothSerial.println(pwrSensorValue);

  // Print sensor values on the serial monitor
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" C, Gas Sensor Value: ");
  Serial.print(sensorValue);
  Serial.print(", IR Sensor State: ");
  Serial.print(irState);
  Serial.print(", Power Sensor Value: ");
  Serial.println(pwrSensorValue);

  delay(1000); // Delay for stability
}
