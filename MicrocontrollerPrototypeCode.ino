#include "LoRaWan_APP.h"
#include "Arduino.h"
#include <ESP32Servo.h>

#define RF_FREQUENCY 915000000
#define BUFFER_SIZE 50

// Pins
#define SERVO_PIN 5
#define MOTOR_PIN 2

// Servo object for steering
Servo myServo;

// ESC parameters
const int ESC_MIN = 1000; // µs
const int ESC_MAX = 2000; // µs
const int PWM_FREQ = 50;  // Hz
const long PERIOD_US = 1000000 / PWM_FREQ; // 20ms period

// LoRa
static RadioEvents_t RadioEvents;
char rxpacket[BUFFER_SIZE];

// Global throttle variable (updated by LoRa)
volatile int currentThrottle = 1500; // neutral

// FAILSAFE TIMER
unsigned long lastPacketTime = 0;
const unsigned long FAILSAFE_TIMEOUT = 3000; // 3 seconds


// Function prototypes
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);
void processControl(int x, int y);
void sendESC(int pulseWidth);

void setup() {
    Serial.begin(115200);
    Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

    // Steering servo
    myServo.attach(SERVO_PIN);
    myServo.write(90); // center steering

    // ESC pin
    pinMode(MOTOR_PIN, OUTPUT);

    // Arm ESC (continuous pulses for ~3s)
    Serial.println("Arming ESC...");
    for (int i = 0; i < 150; i++) {
        sendESC(ESC_MIN);
    }
    Serial.println("ESC armed!");

    // LoRa setup
    RadioEvents.RxDone = OnRxDone;
    Radio.Init(&RadioEvents);
    Radio.SetChannel(RF_FREQUENCY);
    Radio.SetRxConfig(MODEM_LORA, 0, 10, 1, 0, 12, 5, false,
                      0, true, 0, 0, false, true);
    Serial.println("📡 Receiver Ready — Waiting for joystick packets...");
    Radio.Rx(0);

    lastPacketTime = millis();
}

void loop() {

    // ---------------- FAILSAFE CHECK ----------------
    if (millis() - lastPacketTime > FAILSAFE_TIMEOUT) {
        currentThrottle = 1500; // neutral
        myServo.write(90);      // center steering
        Serial.println("⚠️ FAILSAFE ACTIVE — No signal for 3 seconds! Resetting to safe values...");
    }

    // Continuously send ESC pulses using currentThrottle
    sendESC(currentThrottle);

    // Handle LoRa interrupts
    Radio.IrqProcess();
}

// ---------------------- RADIO CALLBACK -----------------------
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
    memcpy(rxpacket, payload, size);
    rxpacket[size] = '\0';
    Serial.printf("📥 Packet Received: %s | RSSI: %d | SNR: %d\n",
                  rxpacket, rssi, snr);

    int x = 0, y = 0;
    sscanf(rxpacket, "X=%d,Y=%d", &x, &y);

    processControl(x, y);

    // Reset watchdog timer
    lastPacketTime = millis();

    Radio.Rx(0);
}

// ---------------------- CONTROL LOGIC -----------------------
void processControl(int x, int y) {
    // Steering    
    if (x >= 0 && x < 570) myServo.write(140);
else if (x >= 570 && x < 850) myServo.write(133);
else if (x >= 850 && x < 1130) myServo.write(126);
else if (x >= 1130 && x < 1410) myServo.write(119);
else if (x >= 1410 && x < 1690) myServo.write(112);
else if (x >= 1690 && x < 1970) myServo.write(105);
else if (x >= 1970 && x < 2039) myServo.write(90);   // deadzone
else if (x >= 2039 && x < 2119) myServo.write(83);
else if (x >= 2119 && x < 2399) myServo.write(76);
else if (x >= 2399 && x < 2679) myServo.write(69);
else if (x >= 2679 && x < 2959) myServo.write(62);
else if (x >= 2959 && x < 3239) myServo.write(55);
else if (x >= 3239 && x < 3519) myServo.write(48);
else if (x >= 3519 && x < 3799) myServo.write(44);
else if (x >= 3799 && x <= 4095) myServo.write(40);

    //if (x >= 0 && x < 570) myServo.write(180);
    //else if (x >= 570 && x < 850) myServo.write(160);
    //else if (x >= 850 && x < 1130) myServo.write(140);
    //else if (x >= 1130 && x < 1410) myServo.write(120);
    //else if (x >= 1410 && x < 1690) myServo.write(110);
    //else if (x >= 1690 && x < 1970) myServo.write(100);
    //else if (x >= 1970 && x < 2039) myServo.write(90);   // deadzone
    //else if (x >= 2039 && x < 2119) myServo.write(80);
    //else if (x >= 2119 && x < 2399) myServo.write(70);
    //else if (x >= 2399 && x < 2679) myServo.write(60);
    //else if (x >= 2679 && x < 2959) myServo.write(50);
    //else if (x >= 2959 && x < 3239) myServo.write(40);
    //else if (x >= 3239 && x < 3519) myServo.write(30);
    //else if (x >= 3519 && x < 3799) myServo.write(20);
    //else if (x >= 3799 && x <= 4095) myServo.write(0);

    // Throttle
    int throttleUS = 1500;

    if (y >= 0 && y <= 399) throttleUS = ESC_MIN;        // HARD REVERSE
    else if (y >= 400 && y <= 999) throttleUS = 1200;    // SOFT REVERSE
    else if (y >= 1000 && y <= 1599) throttleUS = 1350;  // MEDIUM REVERSE
    else if (y >= 1600 && y <= 2499) throttleUS = 1500;  // NEUTRAL
    else if (y >= 2500 && y <= 3099) throttleUS = 1650;  // MEDIUM FORWARD
    else if (y >= 3100 && y <= 3500) throttleUS = 1800;  // SOFT FORWARD
    else if (y >= 3501 && y <= 4095) throttleUS = ESC_MAX; // HARD FORWARD

    currentThrottle = throttleUS;

    Serial.printf("Updated Throttle → ESC=%d us\n", throttleUS);
}

// ---------------------- BIT-BANG ESC -----------------------
void sendESC(int pulseWidth) {
    pulseWidth = constrain(pulseWidth, ESC_MIN, ESC_MAX);
    digitalWrite(MOTOR_PIN, HIGH);
    delayMicroseconds(pulseWidth);
    digitalWrite(MOTOR_PIN, LOW);
    delayMicroseconds(PERIOD_US - pulseWidth);
}
