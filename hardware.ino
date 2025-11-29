#include "config.h"

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//  LCD Setup 
LiquidCrystal_I2C lcd(0x27, 16, 2);

// WiFi from config.h
char ssid[] = WIFI_SSID;
char pass[] = WIFI_PASS;

// Sensor Pins 
#define WATER_LEVEL_SENSOR_PIN 34
#define MOISTURE_SENSOR_PIN 35
#define PIR_SENSOR_PIN 32
#define GAS_SENSOR_PIN 33

//  Motor Driver Pins 
#define WATER_PUMP_IN1 26
#define WATER_PUMP_IN2 27
#define FAN_IN3 14
#define FAN_IN4 12

//  Blynk Virtual Pins 
#define WATER_LEVEL_VIRTUAL_PIN V1
#define MOISTURE_VIRTUAL_PIN V0
#define PIR_VIRTUAL_PIN V2
#define GAS_SENSOR_VIRTUAL_PIN V3

BlynkTimer timer;
volatile int motionCount = 0;
bool lastMotionState = LOW;

// PIR Interrupt
void IRAM_ATTR handleMotion() {
  int currentState = digitalRead(PIR_SENSOR_PIN);
  if (currentState == HIGH && lastMotionState == LOW) {
    motionCount++;
    lastMotionState = HIGH;
  } else if (currentState == LOW) {
    lastMotionState = LOW;
  }
}

void controlWaterPump(bool state) {
  digitalWrite(WATER_PUMP_IN1, state ? HIGH : LOW);
  digitalWrite(WATER_PUMP_IN2, LOW);
}

void controlExhaustFan(bool state) {
  digitalWrite(FAN_IN3, state ? HIGH : LOW);
  digitalWrite(FAN_IN4, LOW);
}

void sendSensorData() {
  // Water Level
  int waterVal = analogRead(WATER_LEVEL_SENSOR_PIN);
  int waterPercent = map(waterVal, 4095, 0, 0, 100);
  Blynk.virtualWrite(WATER_LEVEL_VIRTUAL_PIN, waterPercent);

  // Moisture
  int moistureState = digitalRead(MOISTURE_SENSOR_PIN);
  Blynk.virtualWrite(MOISTURE_VIRTUAL_PIN, moistureState);

  // PIR
  Blynk.virtualWrite(PIR_VIRTUAL_PIN, motionCount);

  // Gas
  int gasVal = analogRead(GAS_SENSOR_PIN);
  int gasPercent = map(gasVal, 0, 4095, 0, 100);
  Blynk.virtualWrite(GAS_SENSOR_VIRTUAL_PIN, gasPercent);

  // Motor Control
  controlWaterPump(gasPercent >= 10);
  controlExhaustFan(gasPercent >= 20);

  // LCD Messages
  lcd.clear();
  if (moistureState == HIGH) {
    lcd.setCursor(0, 0);
    lcd.print("NOT READY TO USE");
    lcd.setCursor(0, 1);
    lcd.print("WORK IN PROGRESS");
  } else {
    lcd.setCursor(0, 0);
    lcd.print("READY TO USE");

    lcd.setCursor(0, 1);
    lcd.print("Gas:");
    lcd.print(gasPercent);
    lcd.print(" Water:");
    lcd.print(waterPercent);
  }
}

void setup() {
  Serial.begin(115200);

  // Sensor Pins
  pinMode(WATER_LEVEL_SENSOR_PIN, INPUT);
  pinMode(MOISTURE_SENSOR_PIN, INPUT);
  pinMode(PIR_SENSOR_PIN, INPUT);
  pinMode(GAS_SENSOR_PIN, INPUT);

  // Motor Pins
  pinMode(WATER_PUMP_IN1, OUTPUT);
  pinMode(WATER_PUMP_IN2, OUTPUT);
  pinMode(FAN_IN3, OUTPUT);
  pinMode(FAN_IN4, OUTPUT);

  controlWaterPump(false);
  controlExhaustFan(false);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print(" Smart Toilet ");
  lcd.setCursor(0, 1);
  lcd.print("   Starting...  ");
  delay(2000);
  lcd.clear();

  // Blynk + Interrupt
  attachInterrupt(digitalPinToInterrupt(PIR_SENSOR_PIN), handleMotion, CHANGE);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  timer.setInterval(10000L, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();
}
