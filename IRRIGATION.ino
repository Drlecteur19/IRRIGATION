// 🚀 Smart Irrigation System with Emergency Mode, Serial Plotter & DS3231 RTC 🚀
// Author: [TAOURIRT LAMRI] for short Dr LECTER
// Description: Reads temperature & humidity from a DHT21 sensor, controls an LED & buzzer,
// displays real-time data on Serial Plotter, and logs timestamps using DS3231 RTC.

#include <Wire.h>
#include <RTClib.h>
#include <dht.h>  // DHT Library for Temperature & Humidity

RTC_DS3231 rtc;
dht DHT;

// 🟢 Define Sensor & Output Pins
#define DHT21_PIN 5   // DHT21 Sensor Pin
#define LED_PIN 4     // System Indicator LED
#define BUZZER_PIN 6  // Emergency Buzzer
#define ALARM_LED 7   // Emergency LED

// 🟢 Define Button Pins
#define START_BUTTON 2       // Start Button
#define EMERGENCY_BUTTON 3   // Emergency Switch Button

// 🟢 System State Variables
bool systemActive = false;      // Tracks if system is ON or OFF
bool emergencyTriggered = false; // Tracks emergency state
unsigned long lastBlinkTime = 0; // Stores last blink time for buzzer & LED
bool blinkState = false;        // Tracks blinking state

// 🟢 Sensor Data Variables
float hum = 0.0, temp = 0.0;    // Stores humidity & temperature

void setup() {
    // Initialize Serial Communication
    Serial.begin(9600);
    Serial.println("🔄 System Ready! Press 'Start' to begin.");

    // Initialize RTC
    if (!rtc.begin()) {
        Serial.println("❌ RTC not found!");
    }
    if (rtc.lostPower()) {
        Serial.println("⏳ RTC lost power, setting time...");
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__))); // Set RTC time to compile time
    }

    // Configure Button Inputs with Internal Pull-up Resistors
    pinMode(START_BUTTON, INPUT_PULLUP);
    pinMode(EMERGENCY_BUTTON, INPUT_PULLUP);

    // Configure Outputs
    pinMode(LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    pinMode(ALARM_LED, OUTPUT);
}

void loop() {
    // 🔴 Check if Emergency Button is Pressed
    if (digitalRead(EMERGENCY_BUTTON) == LOW) { 
        emergencyTriggered = true;  // Activate emergency mode
    } else {
        emergencyTriggered = false; // Reset emergency mode
    }

    // 🚨 Emergency Mode: Turn Everything Off & Blink Alarm
    if (emergencyTriggered) {
        digitalWrite(LED_PIN, LOW);  // Turn off system LED
        systemActive = false;        // Force system to stop

        // Blink Buzzer & LED every 200ms
        if (millis() - lastBlinkTime >= 200) {
            blinkState = !blinkState;
            digitalWrite(BUZZER_PIN, blinkState);
            digitalWrite(ALARM_LED, blinkState);
            lastBlinkTime = millis();
        }
        Serial.println("🚨 EMERGENCY ACTIVATED! System Stopped!");
        return; // Exit loop, no need to continue
    } else {
        digitalWrite(BUZZER_PIN, LOW);
        digitalWrite(ALARM_LED, LOW);
    }

    // 🟢 Handle Start Button: Toggle System ON/OFF
    if (digitalRead(START_BUTTON) == LOW) {
        delay(50); // Debounce
        if (digitalRead(START_BUTTON) == LOW) { // Confirm button press
            systemActive = !systemActive; // Toggle system state
            Serial.println(systemActive ? "✅ SYSTEM STARTED!" : "⛔ SYSTEM STOPPED!");
        }
    }

    // 🟢 If System is ON, Read Sensor Data & Display on Serial Plotter
    if (systemActive) {
        digitalWrite(LED_PIN, HIGH);  // Turn ON system LED

        int chk = DHT.read21(DHT21_PIN); // Read data from DHT21
        hum = DHT.humidity;  // Store humidity value
        temp = DHT.temperature;  // Store temperature value

        // ⏰ Get Time from RTC
        DateTime now = rtc.now();
        Serial.print("📅 Date: ");
    Serial.print(now.day());
    Serial.print("/");
    Serial.print(now.month());
    Serial.print("/");
    Serial.print(now.year());

        Serial.print("⏰ Time: ");
        Serial.print(now.hour()); Serial.print(":");
        Serial.print(now.minute()); Serial.print(":");
        Serial.print(now.second());

        // 📊 Send Data to Serial Plotter in Proper Format
        Serial.print(" | 📊Temperature: ");
        Serial.print(temp);
        Serial.print("°C, 📊Humidity: ");
        Serial.print(hum);
        Serial.println("%");

        // Serial Plotter Output (time, temperature, humidity)
        //Serial.print(now.hour()); Serial.print(":");
        //Serial.print(now.minute()); Serial.print(",");
        Serial.print(temp);
        Serial.print(",");
        Serial.println(hum);
    } else {
        digitalWrite(LED_PIN, LOW);  // Turn OFF system LED
    }

    delay(500); // Delay for smoother graph plotting
}
