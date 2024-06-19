#include <Arduino.h>
#include <ArduinoJson.h>
#include <PicoMQTT.h>
#include <Preferences.h>
#include <Wire.h>
#include <dsps_biquad_gen.h>
#include <esp_dsp.h>

#include <CircularBuffer.hpp>

#include "MAX30105.h"
#include "spo2_algorithm.h"

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

MAX30105 particleSensor;
Preferences prefs;
PicoMQTT::Server mqtt;

#define APP_NAME "protorx-spo2"
#define BUFFER_LENGTH 1024
#define PIN_READ_LED 10

#define PIN_SENSOR 8
#define PIN_MOTOR_PWM_1 5
#define PIN_MOTOR_PWM_2 6
#define PIN_MOTOR_PWM PIN_MOTOR_PWM_1

#define PIN_SDA 9
#define PIN_SCL 2

#define PIN_BTN_UP 21
#define PIN_BTN_DOWN 20
#define PIN_BTN_SHARP 3
#define PIN_BTN_STAR 4

#define INTERVAL_BASIC_MSG_MILLIS 5000
#define INTERVAL_WATCHDOG_MILLIS 10000
#define THREADHOLD_RED 100000

#define MX3010X_MAX_RETRY 10
#define IS_LED_BLINK false

#define SCREEN_ADDRESS 0x3C
#define OLED_RESET     -1

String MQTT_TOPIC;
uint32_t irArray[BUFFER_LENGTH];  // infrared LED sensor data
uint32_t redArray[BUFFER_LENGTH]; // red LED sensor data
CircularBuffer<uint32_t, BUFFER_LENGTH> irBuffer;
CircularBuffer<uint32_t, BUFFER_LENGTH> redBuffer;

int32_t spo2;          // SPO2 value
int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
int32_t heartRate;     // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

// iir param
float coeffs_lpf[5];
float lowpassFreq = 0.1;
float qFactor = 3;
float w_lpf[5] = {0, 0};
float *irFloatArray;
float *redFloatArray;
float *irFloatArrayOut;
float *redFloatArrayOut;

bool mx30102_available = false;

long ts_last_basic_msg;

long ts_last_watchdog_ok;

long ts_sensor_last;
long ts_sensor_this;
long last_interval;
bool sensor_switch_update = false;

long target_interval = 400;
long current_pwm = 128;

CircularBuffer<long, 4> interval_buffer;

Adafruit_SSD1306 display;

void IRAM_ATTR SensorSwitchInterrupt() {
    // deal with bounce
    if (millis() - ts_sensor_this < 150) {
        return;
    }
    ts_sensor_last = ts_sensor_this;
    ts_sensor_this = millis();
    sensor_switch_update = true;
}

void displayNetworkInfo() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println("RSSI: " + String(WiFi.RSSI()) + "  TS: " + millis() / 1000);
    display.println("IP: " + WiFi.localIP().toString());
    display.println("MQTT: ");
    display.println(MQTT_TOPIC);
    display.display();
}

void setup() {
    // Usual setup
    pinMode(PIN_READ_LED, OUTPUT);
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_READ_LED, HIGH);
        delay(100);
        digitalWrite(PIN_READ_LED, LOW);
        delay(100);
    }

    Serial.begin(115200);
    delay(2500);
    Serial.println("Starting protorx-spo2...");

    // load config
    prefs.begin(APP_NAME);
    // TODO: set wifi config in ap mode
    prefs.putString("ssid", "DoNotUseIt-ESP");
    // prefs.putString("ssid", "CMCC_h3k2");
    prefs.putString("wlan_password", "showmethemoney");
    String ssid = prefs.getString("ssid", "");
    String password = prefs.getString("wlan_password", "");
    // ssid = "SSID";
    // password = "88888888";


    // Initialize I2C
    Wire.setPins(PIN_SDA, PIN_SCL);

    // init oled
    display = Adafruit_SSD1306(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
    display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS);
    display.display();
    delay(500); 
    display.clearDisplay();

    display.fillScreen(SSD1306_WHITE);
    display.display();
    delay(500); 
    display.clearDisplay();
    display.display();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE, SSD1306_BLACK);

    // WiFi setup
    WiFi.mode(WIFI_STA);
    String macAddr = WiFi.macAddress();
    Serial.println("\nUsing mac addr: " + macAddr);
    macAddr.replace(":", "");
    String hostname = String(APP_NAME) + "-" + macAddr.substring(macAddr.length() - 4, macAddr.length());
    WiFi.hostname(hostname);
    WiFi.begin(ssid, password);
    Serial.println("Connecting to WiFi " + ssid + " with hostname " + hostname);
    display.setCursor(0, 0);
    display.println("Connecting to WiFi");
    display.println("SSID: " + ssid);
    display.println("Hostname: " + hostname);
    display.println("MAC: " + macAddr);
    display.display();
    for(int i = 0; i < 20; i++) {
        delay(500);
        Serial.print(".");
        display.print(".");
        display.display();
        if(WiFi.status() == WL_CONNECTED) {
            break;
        }
    }
    if(WiFi.status() != WL_CONNECTED) {
        display.clearDisplay();
        display.setCursor(0, 0);
        Serial.println("Failed to connect to WiFi");
        display.println("Failed to connect to WiFi");
        display.display();
        delay(2000);
        // reset soc
        ESP.restart();
    }
    while (WiFi.status() != WL_CONNECTED) {
        delay(2000);
        Serial.print(".");
        display.print(".");
        display.display();
    }
    Serial.println(String("\nUsing ip addr: ") + WiFi.localIP().toString());

    // MQTT broker setup
    // Subscribe to a topic pattern and attach a callback
    MQTT_TOPIC = String("device/") + macAddr;
    Serial.println("Subscribing to topic: " + MQTT_TOPIC);
    mqtt.subscribe(MQTT_TOPIC.c_str(), [](const char *topic, const char *payload) {
        Serial.printf("Received message in topic '%s': %s\n", topic, payload);
        JsonDocument doc;
        deserializeJson(doc, payload);
        if (doc.containsKey("motor") && doc["motor"].containsKey("target")) {
            target_interval = doc["motor"]["target"];
            prefs.putInt("target_interval", target_interval);
        }
    });
    mqtt.begin();
    displayNetworkInfo();

    // Interrupt sensor switch
    pinMode(PIN_SENSOR, INPUT_PULLUP);
    ts_sensor_last = millis();
    ts_sensor_this = millis();
    sensor_switch_update = false;
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), SensorSwitchInterrupt, RISING);
    for (int i = 0; i < interval_buffer.capacity; i++) {
        interval_buffer.push(300);
    }

    // Initialize motor
    current_pwm = 125;
    target_interval = prefs.getInt("target_interval", 400);
    pinMode(PIN_MOTOR_PWM, OUTPUT);
    analogWrite(PIN_MOTOR_PWM, current_pwm);

    // Initialize spo2 sensor
    mx30102_available = particleSensor.begin();
    for (int i = 0; i < MX3010X_MAX_RETRY; i++) {
        if (mx30102_available) {
            break;
        }
        mx30102_available = particleSensor.begin();
        Serial.println("MAX30105 is not loaded, retrying.");
        delay(100);
    }
    if (mx30102_available) {
        byte ledBrightness = prefs.getChar("led_brightness", 60); // Options: 0=Off to 255=50mA
        byte sampleAverage = prefs.getChar("sample_average", 4);  // Options: 1, 2, 4, 8, 16, 32
        byte ledMode = prefs.getChar("led_mode", 2);              // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
        // Real sample rate may be only about 20hz
        byte sampleRate = prefs.getChar("sample_rate", 200); // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
        int pulseWidth = prefs.getInt("pulse_width", 411);   // Options: 69, 118, 215, 411
        int adcRange = prefs.getInt("adc_range", 4096);      // Options: 2048, 4096, 8192, 16384
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
        Serial.print("Red,IR=");
        Serial.print(particleSensor.getRed());
        Serial.print(",");
        Serial.println(particleSensor.getIR());
        Serial.println("MAX30105 is set");
        esp_err_t res = dsps_biquad_gen_lpf_f32(coeffs_lpf, lowpassFreq, qFactor);
        if (res != ESP_OK) {
            Serial.println("iir init failed");
        }
        irFloatArray = (float *)malloc(BUFFER_LENGTH * sizeof(float));
        redFloatArray = (float *)malloc(BUFFER_LENGTH * sizeof(float));
        irFloatArrayOut = (float *)malloc(BUFFER_LENGTH * sizeof(float));
        redFloatArrayOut = (float *)malloc(BUFFER_LENGTH * sizeof(float));

        digitalWrite(PIN_READ_LED, LOW);
    } else {
        Serial.println("MAX30105 is not loaded, skipping.");
        mx30102_available = false;
    }
}

bool readSensor() {
    particleSensor.check();
    if (particleSensor.available() == false) {
        return false;
    }
    uint32_t ir = particleSensor.getIR();
    uint32_t red = particleSensor.getRed();
    if (red < THREADHOLD_RED) {
        return false;
    }

    irBuffer.push(ir);
    redBuffer.push(red);
    particleSensor.nextSample();

    digitalWrite(PIN_READ_LED, !(!digitalRead(PIN_READ_LED) && IS_LED_BLINK));

    return true;
}

void loop() {
    mqtt.loop();

    // Watchdog
    if (millis() - ts_last_watchdog_ok > INTERVAL_WATCHDOG_MILLIS || millis() < ts_last_watchdog_ok) {
        // TODO: Watchdog task
        ts_last_watchdog_ok = millis();
        JsonDocument doc;
        doc["ts"] = millis();
        doc["watchdog"]["ok"] = true;

        String res;
        serializeJson(doc, res);
        mqtt.publish(MQTT_TOPIC, res);
    }

    // Publish basic info every 5 seconds
    if (millis() - ts_last_basic_msg > INTERVAL_BASIC_MSG_MILLIS || millis() < ts_last_basic_msg) {
        ts_last_basic_msg = millis();
        JsonDocument doc;
        doc["ts"] = millis();
        doc["wifi"]["ip"] = String(WiFi.localIP().toString());
        doc["wifi"]["mac"] = WiFi.macAddress();
        doc["wifi"]["hostname"] = WiFi.getHostname();
        doc["wifi"]["status"] = WiFi.status();
        doc["wifi"]["ssid"] = WiFi.SSID();
        doc["wifi"]["rssi"] = WiFi.RSSI();

        doc["mqtt"]["topic"] = MQTT_TOPIC;

        doc["mx30102"]["ok"] = mx30102_available;
        doc["mx30102"]["available"] = particleSensor.available();
        doc["mx30102"]["red"] = redBuffer.last();
        doc["mx30102"]["ir"] = irBuffer.last();

        doc["watchdog"]["timeout"] = ts_last_watchdog_ok - millis();
        doc["watchdog"]["limit"] = INTERVAL_WATCHDOG_MILLIS;

        doc["sensor_switch"]["interval"] = ts_sensor_this - ts_sensor_last;
        doc["sensor_switch"]["last"] = ts_sensor_last;
        doc["sensor_switch"]["this"] = ts_sensor_this;

        doc["motor"]["current"] = current_pwm;
        doc["motor"]["target"] = target_interval;

        String res;
        serializeJson(doc, res);
        mqtt.publish(MQTT_TOPIC, res);
        displayNetworkInfo();
    }

    // Sensor switch
    if (sensor_switch_update) {
        sensor_switch_update = false;
        JsonDocument doc;

        interval_buffer.push(ts_sensor_this - ts_sensor_last);
        // calc average
        long sum = 0;
        for (int i = 0; i < interval_buffer.capacity; i++) {
            sum += interval_buffer[i];
        }
        long avg = sum / interval_buffer.capacity;
        doc["sensor_switch"]["avg"] = avg;

        if (avg > target_interval) {
            current_pwm = current_pwm + 5;
            if (current_pwm > 200) {
                current_pwm = 200;
            }
            analogWrite(PIN_MOTOR_PWM, current_pwm);
        } else {
            current_pwm = current_pwm - 1;
            if (current_pwm < 100) {
                current_pwm = 100;
            }
            analogWrite(PIN_MOTOR_PWM, current_pwm);
        }

        doc["ts"] = millis();
        doc["sensor_switch"]["interval"] = ts_sensor_this - ts_sensor_last;
        doc["sensor_switch"]["last"] = ts_sensor_last;
        doc["sensor_switch"]["this"] = ts_sensor_this;

        String res;
        serializeJson(doc, res);
        Serial.println(res);
        mqtt.publish(MQTT_TOPIC, res);
    }
    return;

    // If we have enough data, run the algorithm
    if (mx30102_available && readSensor() && irBuffer.isFull()) {
        redBuffer.copyToArray(redArray);
        irBuffer.copyToArray(irArray);

        for (int i = 0; i < BUFFER_LENGTH; i++) {
            redFloatArray[i] = redArray[i] * 1.0;
            irFloatArray[i] = irArray[i] * 1.0;
        }
        dsps_biquad_f32(redFloatArray, redFloatArrayOut, BUFFER_LENGTH, coeffs_lpf, w_lpf);
        dsps_biquad_f32(irFloatArray, irFloatArrayOut, BUFFER_LENGTH, coeffs_lpf, w_lpf);
        for (int i = 0; i < BUFFER_LENGTH; i++) {
            redArray[i] = (uint16_t)redFloatArrayOut[i];
            irArray[i] = (uint16_t)irFloatArrayOut[i];
        }

        // Serial.println(irBuffer.last());
        // maxim_heart_rate_and_oxygen_saturation(irArray, BUFFER_LENGTH, redArray, &spo2, &validSPO2, &heartRate, &validHeartRate);
        if (validSPO2 || random(1000) <= 1) {
            // make json object
            JsonDocument doc;
            doc["ts"] = millis();
            doc["mx30102"]["available"] = particleSensor.available();
            doc["mx30102"]["red"] = redBuffer.last();
            doc["mx30102"]["ir"] = irBuffer.last();
            doc["mx30102"]["spo2"] = spo2;
            doc["mx30102"]["validSPO2"] = validSPO2;
            doc["mx30102"]["heartRate"] = heartRate;
            doc["mx30102"]["validHeartRate"] = validHeartRate;

            String res;
            serializeJson(doc, res);
            Serial.println(res);
            mqtt.publish(MQTT_TOPIC, res);
            // Remove 25 oldest points
            // for (int i = 0; i < 25; i++) {
            //     irBuffer.shift();
            //     redBuffer.shift();
            // }
            for (int i = 0; i < BUFFER_LENGTH; i++) {
                Serial.print(irArray[i]);
                Serial.print("\t");
                Serial.print(irFloatArray[i]);
                Serial.println("");
            }
        }
    }
}