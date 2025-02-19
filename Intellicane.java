#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include <MPU6050.h>

#define PULSE_PIN A0       // Pulse sensor
#define LED_PIN 3          // LED light
#define BUZZER_PIN 4       // Alert buzzer
#define BUTTON_PIN 5       // Button for LED
#define SEAT_BUTTON_PIN 6  // Button for folding seat
#define GPS_RX 7           // GPS Module RX
#define GPS_TX 8           // GPS Module TX
#define GSM_TX 9           // GSM Module TX
#define GSM_RX 10          // GSM Module RX
#define SDA_PIN A4         // Accelerometer SDA
#define SCL_PIN A5         // Accelerometer SCL

SoftwareSerial gpsSerial(GPS_RX, GPS_TX);
SoftwareSerial gsmSerial(GSM_RX, GSM_TX);
TinyGPSPlus gps;
MPU6050 mpu;

int pulseValue = 0;
bool ledState = false;
bool fallDetected = false;
bool seatExtended = false;

void setup() {
    Serial.begin(9600);
    gpsSerial.begin(9600);
    gsmSerial.begin(9600);
    Wire.begin();
    mpu.initialize();
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(BUTTON_PIN, INPUT_PULLUP);
    pinMode(SEAT_BUTTON_PIN, INPUT_PULLUP);
    
    Serial.println("IntelliCANE System Started");
}

void loop() {
    pulseMonitoring();
    gpsTracking();
    checkEmergencyButton();
    fallDetection();
    handleFoldingSeat();
}

void pulseMonitoring() {
    pulseValue = analogRead(PULSE_PIN);
    Serial.print("Pulse: ");
    Serial.println(pulseValue);
    
    if (pulseValue > 900) {  //for irregular heartbeat
        Serial.println("Alert: Abnormal Pulse Detected!");
        digitalWrite(BUZZER_PIN, HIGH);
        sendSMS("Abnormal pulse detected! Check on the user.");
        delay(2000);
        digitalWrite(BUZZER_PIN, LOW);
    }
    delay(1000);
}

void gpsTracking() {
    while (gpsSerial.available() > 0) {
        gps.encode(gpsSerial.read());
        if (gps.location.isUpdated()) {
            Serial.print("Location: ");
            Serial.print(gps.location.lat(), 6);
            Serial.print(", ");
            Serial.println(gps.location.lng(), 6);
        }
    }
}

void checkEmergencyButton() {
    if (digitalRead(BUTTON_PIN) == LOW) {
        ledState = !ledState;
        digitalWrite(LED_PIN, ledState);
        delay(500);
    }
}

void fallDetection() {
    int16_t ax, ay, az;
    mpu.getAcceleration(&ax, &ay, &az);
    
    if (abs(ax) < 500 && abs(ay) < 500 && abs(az) < 500) { // for fall detection
        if (!fallDetected) {
            Serial.println("Fall detected! Sending alert.");
            digitalWrite(BUZZER_PIN, HIGH);
            sendSMS("Fall detected! Please check on the user.");
            delay(2000);
            digitalWrite(BUZZER_PIN, LOW);
            fallDetected = true;
        }
    } else {
        fallDetected = false;
    }
}

void handleFoldingSeat() {
    if (digitalRead(SEAT_BUTTON_PIN) == LOW) {
        seatExtended = !seatExtended;
        Serial.print("Seat is now ");
        Serial.println(seatExtended ? "Extended" : "Hidden");
        delay(500);
    }
}

void sendSMS(String message) {
    gsmSerial.println("AT+CMGF=1");  // Set SMS mode
    delay(100);
    gsmSerial.println("AT+CMGS=\"+639946096568\"");  // Replace with emergency contact number
    delay(100);
    gsmSerial.print(message);
    delay(100);
    gsmSerial.write(26);  // End SMS with CTRL+Z
    delay(1000);
    Serial.println("SMS Sent: " + message);
}
