#include <Arduino.h>
#include <ArduinoJson.h>
#include <PicoMQTT.h>
#include <Preferences.h>
#include <Wire.h>

#include <CircularBuffer.hpp>

#include "ESP32TimerInterrupt.h"
#include <Adafruit_AHTX0.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SGP30.h>
#include <Adafruit_SSD1306.h>
#include <MAX30105.h>
#include <spo2_algorithm.h>

#include "btn.hpp"
#include "logger.hpp"
#include "menu.hpp"
#include "page_logger.hpp"
#include "page_sys_status.hpp"
#include "ssd_1306_display.hpp"

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

MAX30105 particleSensor;
Adafruit_SGP30 sgp;
Adafruit_AHTX0 aht;
Adafruit_BMP280 bmp;
Preferences prefs;
PicoMQTT::Server mqtt;

Logger logger;
Btn btn;

Menu menu;

#define APP_NAME "protorx-spo2"
#define BUFFER_LENGTH 100
#define PIN_READ_LED 10

#define PIN_SENSOR 9
#define PIN_MOTOR_PWM_1 5
#define PIN_MOTOR_PWM_2 6

#define PIN_SDA 8
#define PIN_SCL 2

#define INTERVAL_BASIC_MSG_MILLIS 30000
#define INTERVAL_WATCHDOG_MILLIS 10000
#define INTERVAL_SGP30_UPDATE_MILLIS 1000
#define INTERVAL_SGP30_BASELINE_MILLIS 60000
#define THREADHOLD_RED 100000
#define INTERVAL_SCREEN_UPDATE 50
#define INTERVAL_SCREEN_OFF 60000

#define INTERVAL_BTN_SCAN_MS 150

#define MX3010X_MAX_RETRY 10
#define IS_LED_BLINK false

#define SCREEN_ADDRESS 0x3C
#define OLED_RESET -1

#define MOTOR_MAX_VOLTAGE 12
#define MOTOR_MIN_VOLTAGE 4

enum Screen { SCREEN_CONF,
              SCREEN_INFO,
              SCREEN_LOG,
              SCREEN_MAX,
              SCREEN_OFF };

enum Config { CONFIG_MOTOR,
              CONFIG_INTERVAL,
              CONFIG_DIR,
              CONFIG_VOLTAGE };

enum DirOption { DIR_UP,
                 DIR_DOWN,
                 DIR_AUTO,
                 DIR_RANDOM };

String MQTT_TOPIC;
uint32_t irArray[BUFFER_LENGTH];  // infrared LED sensor data
uint32_t redArray[BUFFER_LENGTH]; // red LED sensor data
CircularBuffer<uint32_t, BUFFER_LENGTH> irBuffer;
CircularBuffer<uint32_t, BUFFER_LENGTH> redBuffer;

int32_t spo2;          // SPO2 value
int8_t validSPO2;      // indicator to show if the SPO2 calculation is valid
int32_t heartRate;     // heart rate value
int8_t validHeartRate; // indicator to show if the heart rate calculation is valid

bool mx30102_available = false;
bool sgp30_available = false;
bool aht20_available = false;
bool bmp280_available = false;
uint32_t sample_count = 0;
uint32_t sample_freq_hz = 16;
#define TARGET_FREQ 16
CircularBuffer<long, BUFFER_LENGTH> sampleTsBuffer;

long ts_last_basic_msg;

long ts_last_watchdog_ok;
long ts_last_screen_update;
long ts_last_sgp30_update;
long ts_last_sgp30_baseline;
long ts_last_btn_press;

long ts_sensor_last;
long ts_sensor_this;
long ts_last_stop = 0;
long last_interval;
bool sensor_switch_update = false;
long sensor_switch_count = 0;

long power_voltage = 20;

long motor_interval = 400;
long motor_percent = 50;
long motor_pwm_1 = 0;
long motor_pwm_2 = 0;
bool current_dir = 0;
uint8_t motor_diroption = 0;

uint8_t current_config = 0;
uint8_t current_screen = 0;
bool config_update = false;

CircularBuffer<long, 4> interval_buffer;
ESP32Timer IBtnScanTimer(1);

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity);                                                                // [mg/m^3]
    return absoluteHumidityScaled;
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
    sample_count++;
    sampleTsBuffer.push(millis());
    sample_freq_hz = 1000 * BUFFER_LENGTH / (sampleTsBuffer.last() - sampleTsBuffer.first());
    particleSensor.nextSample();

    digitalWrite(PIN_READ_LED, !(!digitalRead(PIN_READ_LED) && IS_LED_BLINK));

    return true;
}

void setMotor(long power_percent, uint8_t dir) {
    current_dir = dir;
    long motor_voltage = (MOTOR_MAX_VOLTAGE - MOTOR_MIN_VOLTAGE) * power_percent * 10 + MOTOR_MIN_VOLTAGE * 1000;
    long pwm = 0;
    if (power_percent > 0) {
        pwm = 255 * motor_voltage / (power_voltage * 1000);
        if (dir) {
            motor_pwm_1 = pwm;
            motor_pwm_2 = 0;
        } else {
            motor_pwm_1 = 0;
            motor_pwm_2 = pwm;
        }
    } else {
        motor_pwm_1 = 0;
        motor_pwm_2 = 0;
    }
    logger.log("Set motor: 1p:" + String(motor_pwm_1) + " 2p:" + String(motor_pwm_2) + " " + String(power_percent) + "% " +
               String(dir) + "d " + String(pwm) + "w " + String(motor_voltage / 1000.0) + "mV");
}

void calcMotor() {
    if (motor_diroption == DIR_AUTO) {
        setMotor(motor_percent, !current_dir);
    } else if (motor_diroption == DIR_RANDOM) {
        setMotor(motor_percent, random(2));
    } else {
        setMotor(motor_percent, motor_diroption % 2);
    }
}

void writeMotor() {
    analogWrite(PIN_MOTOR_PWM_1, motor_pwm_1);
    analogWrite(PIN_MOTOR_PWM_2, motor_pwm_2);
}

void IRAM_ATTR SensorSwitchInterrupt() {
    // deal with bounce
    if (millis() - ts_sensor_this < 500) {
        return;
    }
    ts_sensor_last = ts_sensor_this;
    ts_sensor_this = millis();
    sensor_switch_update = true;
    sensor_switch_count++;
}

bool IRAM_ATTR BtnScanTimerHandler(void *timerNo) {
    // Turn off screen if no button pressed
    if (digitalRead(PIN_BTN_UP) == LOW || digitalRead(PIN_BTN_DOWN) == LOW || digitalRead(PIN_BTN_SHARP) == LOW ||
        digitalRead(PIN_BTN_STAR) == LOW) {
        ts_last_btn_press = millis();
    }
    if (digitalRead(PIN_BTN_STAR) == LOW) {
        current_screen = (current_screen + 1) % 5;
    }
    if (digitalRead(PIN_BTN_SHARP) == LOW) {
        current_config = (current_config + 1) % 4;
    }
    if (current_screen == SCREEN_CONF) {
        if (digitalRead(PIN_BTN_UP) == LOW && digitalRead(PIN_BTN_STAR) == LOW) {
            // restart
            ESP.restart();
        }
        if (digitalRead(PIN_BTN_UP) == LOW) {
            config_update = true;
            switch (current_config) {
            case CONFIG_MOTOR:
                motor_percent += 5;
                if (motor_percent > 100) {
                    motor_percent = 100;
                }
                break;
            case CONFIG_INTERVAL:
                motor_interval = motor_interval * 101 / 100 + 1;
                if (motor_interval > 3000) {
                    motor_interval = 3000;
                }
                break;
            case CONFIG_DIR:
                motor_diroption = (motor_diroption + 1) % 4;
                break;
            case CONFIG_VOLTAGE:
                power_voltage += 1;
                break;
            }
        }
        if (digitalRead(PIN_BTN_DOWN) == LOW) {
            config_update = true;
            switch (current_config) {
            case CONFIG_MOTOR:
                motor_percent -= 5;
                if (motor_percent <= 0) {
                    motor_percent = 1;
                }
                break;
            case CONFIG_INTERVAL:
                motor_interval = motor_interval * 99 / 100 - 1;
                if (motor_interval < 0) {
                    motor_interval = 0;
                }
                break;
            case CONFIG_DIR:
                motor_diroption = (motor_diroption + 3) % 4;
                break;
            case CONFIG_VOLTAGE:
                // power_voltage -= 1;
                if (power_voltage < 12) {
                    power_voltage = 12;
                }
                break;
            }
        }
    }

    return true;
}

void displayConfig() {
    display.clearDisplay();
    display.setCursor(0, 0);
    float voltage = (motor_pwm_1 + motor_pwm_2) * power_voltage / 255.0;
    display.println(String(millis() / 1000) + "  " + String(motor_pwm_1) + "  " + String(motor_pwm_2) + "  " + String(voltage));
    if (mx30102_available) {
        if (validHeartRate) {
            display.print("$");
        } else {
            display.print(" ");
        }
        display.print("HR: " + String(heartRate) + "|");
        if (validSPO2) {
            display.print("$");
        } else {
            display.print(" ");
        }
        display.print("SPO2: " + String(spo2));
        display.println("");
    }
    if (sgp30_available) {
        display.println("V: " + String(sgp.TVOC) + " C: " + String(sgp.eCO2));
    } else {
        display.println("SGP30 not available");
    }
    if (current_config == CONFIG_MOTOR) {
        display.print("*");
    }
    display.println("Motor: " + String(motor_percent) + "%");
    if (current_config == CONFIG_INTERVAL) {
        display.print("*");
    }
    display.println("Interval: " + String(motor_interval));
    if (current_config == CONFIG_DIR) {
        display.print("*");
    }
    display.print("Dir: ");
    switch (motor_diroption) {
    case DIR_UP:
        display.println("FWD");
        break;
    case DIR_DOWN:
        display.println("REV");
        break;
    case DIR_AUTO:
        display.println("AUTO");
        break;
    case DIR_RANDOM:
        display.println("RANDOM");
        break;
    }
    if (current_config == CONFIG_VOLTAGE) {
        display.print("*");
    }
    display.println("Power voltage: " + String(power_voltage));
    display.display();
}

void displayMx30102Chart() {
    display.clearDisplay();
    display.setCursor(0, 0);
    // draw ir and red chart with buffer
    // find max and min in buffers to make chart
    uint32_t max_red = 0;
    uint32_t min_red = u_int32_t(-1);
    uint32_t max_ir = 0;
    uint32_t min_ir = u_int32_t(-1);
    for (int i = 0; i < BUFFER_LENGTH; i++) {
        if (redBuffer[i] > max_red) {
            max_red = redBuffer[i];
        }
        if (redBuffer[i] < min_red) {
            min_red = redBuffer[i];
        }
        if (irBuffer[i] > max_ir) {
            max_ir = irBuffer[i];
        }
        if (irBuffer[i] < min_ir) {
            min_ir = irBuffer[i];
        }
    }

    uint32_t max_pt = max(max_red, max_ir);
    uint32_t min_pt = min(min_red, min_ir);

    if (validHeartRate) {
        display.print("$");
    } else {
        display.print(" ");
    }
    display.print("HR: " + String(heartRate) + "|");
    if (validSPO2) {
        display.print("$");
    } else {
        display.print(" ");
    }
    display.print("SPO2: " + String(spo2));
    display.println("");
    switch (current_config) {
    case CONFIG_MOTOR:
        display.print("R");
        break;
    case CONFIG_INTERVAL:
        display.print("I");
        break;
    case CONFIG_DIR:
        display.print("B");
        break;
    case CONFIG_VOLTAGE:
        display.print("D");
        break;
    default:
        display.print("B");
        break;
    }
    display.println(": " + String(min_pt) + " - " + String(max_pt));
    display.println("F: " + String(sample_freq_hz) + "Hz");
    // display.println(String(irBuffer[3]) + "," + String(map(irBuffer[3], min_pt, max_pt, 0, SCREEN_HEIGHT)));
    // display.println(String(redBuffer[3]) + "," + String(map(redBuffer[3], min_pt, max_pt, 0, SCREEN_HEIGHT)));
    for (int i = 0; i < SCREEN_WIDTH; i++) {
        if (i >= BUFFER_LENGTH) {
            break;
        }
        // display.drawPixel(i, SCREEN_HEIGHT - red, SSD1306_BLACK);

        switch (current_config) {
            uint32_t red;
            uint32_t ir;
        case CONFIG_MOTOR:
            red = map(redBuffer[i], min_red, max_red, 0, SCREEN_HEIGHT);
            display.drawPixel(i, SCREEN_HEIGHT - red, SSD1306_WHITE);
            break;
        case CONFIG_INTERVAL:
            ir = map(irBuffer[i], min_ir, max_ir, 0, SCREEN_HEIGHT);
            display.drawPixel(i, SCREEN_HEIGHT - ir, SSD1306_WHITE);
            break;
        case CONFIG_DIR:
            red = map(redBuffer[i], min_red, max_red, 0, SCREEN_HEIGHT);
            ir = map(irBuffer[i], min_ir, max_ir, 0, SCREEN_HEIGHT);
            display.drawPixel(i, SCREEN_HEIGHT - red, SSD1306_WHITE);
            display.drawPixel(i, SCREEN_HEIGHT - ir, SSD1306_WHITE);
            break;
        default:
            red = map(redBuffer[i], min_pt, max_pt, 0, SCREEN_HEIGHT);
            ir = map(irBuffer[i], min_pt, max_pt, 0, SCREEN_HEIGHT);
            display.drawPixel(i, SCREEN_HEIGHT - red, SSD1306_WHITE);
            display.drawPixel(i, SCREEN_HEIGHT - ir, SSD1306_WHITE);
            break;
        }
    }
    display.display();
}

void displayStatus() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.print("TS: " + String(millis() / 1000) + "  ");
    if (WiFi.getMode() == WIFI_AP) {
        display.println("");
        display.println("AP: " + WiFi.softAPSSID());
        display.println("IP: " + WiFi.softAPIP().toString());
    } else {
        display.println("RSSI: " + String(WiFi.RSSI()));
        display.println("IP: " + WiFi.localIP().toString());
    }

    display.println("MQTT: ");
    display.println(MQTT_TOPIC);
    if (mx30102_available) {
        display.println("MX: " + String(irBuffer.last()) + "," + String(redBuffer.last()));
        display.println(String(sample_freq_hz) + "Hz");
    }
    display.display();
}

void displayLog() {
    display.clearDisplay();
    display.setCursor(0, 0);
    display.println(logger.tail());
    display.display();
}

void updateDisplay() {
    switch (current_screen) {
    case SCREEN_CONF:
        displayConfig();
        break;
    case SCREEN_INFO:
        displayStatus();
        break;
    case SCREEN_LOG:
        displayLog();
        break;
    case SCREEN_MAX:
        displayMx30102Chart();
        break;
    case SCREEN_OFF:
        display.clearDisplay();
        display.display();
        break;
    }
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
    logger.setSerial(&Serial);

    // Setup BTN
    btn.begin();
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_UP), []() { btn.pressed(BTN_UP); }, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_DOWN), []() { btn.pressed(BTN_DOWN); }, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_SHARP), []() { btn.pressed(BTN_SHARP); }, FALLING);
    attachInterrupt(digitalPinToInterrupt(PIN_BTN_STAR), []() { btn.pressed(BTN_STAR); }, FALLING);

    ts_last_btn_press = millis();
    IBtnScanTimer.attachInterruptInterval(INTERVAL_BTN_SCAN_MS * 1000, BtnScanTimerHandler);

    // load config
    prefs.begin(APP_NAME);
    // TODO: set wifi config in ap mode
    JsonDocument config;
    deserializeJson(config, prefs.getString("config", "{}"));
    motor_percent = prefs.getInt("motor_percent", 50);
    motor_diroption = prefs.getChar("motor_dir_option", 0);
    motor_interval = prefs.getInt("motor_interval", 500);

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

    ts_last_basic_msg = millis();

    // WiFi setup
    WiFi.mode(WIFI_STA);
    String macAddr = WiFi.macAddress();
    logger.log("\nUsing mac addr: " + macAddr);
    macAddr.replace(":", "");
    String hostname = String(APP_NAME) + "-" + macAddr.substring(macAddr.length() - 4, macAddr.length());
    WiFi.hostname(hostname);

    // Try wifi configs from config in prefs
    // config: {"wifi":[{"ssid":"ssid_str","password": "password_str"}]}
    if (config.containsKey("wifi")) {
        for (int i = 0; i < config["wifi"].size(); i++) {
            String ssid = config["wifi"][i]["ssid"].as<String>();
            String password = config["wifi"][i]["password"].as<String>();
            WiFi.begin(ssid, password);
            logger.log("Connecting to WiFi " + ssid + " with hostname " + hostname);
            display.clearDisplay();
            display.setCursor(0, 0);
            display.println("Connecting to WiFi");
            display.println("AP: " + ssid);
            display.println("Hostname: " + hostname);
            display.println("MAC: " + macAddr);
            display.display();
            for (int i = 0; i < 10; i++) {
                delay(500);
                display.print(".");
                display.display();
                if (WiFi.status() == WL_CONNECTED) {
                    break;
                    logger.log(String("\nUsing ip addr: ") + WiFi.localIP().toString());
                }
            }
            if (WiFi.status() == WL_CONNECTED) {
                break;
            }
        }
    }

    if (WiFi.status() != WL_CONNECTED) {
        WiFi.disconnect();
        display.clearDisplay();
        display.setCursor(0, 0);
        logger.log("WiFi Sta Failed");
        display.println("WiFi Sta Failed");
        display.display();
        delay(2000);
        WiFi.mode(WIFI_MODE_AP);
        WiFi.softAP(hostname, macAddr);
        display.clearDisplay();
        logger.log("WiFi AP Mode: " + hostname + "," + macAddr + "," + WiFi.softAPIP().toString());
    }

    // MQTT broker setup
    // Subscribe to a topic pattern and attach a callback
    MQTT_TOPIC = String("device/") + macAddr;
    logger.log("Subscribing to topic: " + MQTT_TOPIC);
    mqtt.subscribe(MQTT_TOPIC.c_str(), [](const char *topic, const char *payload) {
        logger.log(String("Received message in topic '") + String(topic) + ": " + String(payload));
        JsonDocument doc;
        deserializeJson(doc, payload);
        if (doc.containsKey("wifi")) {
            // Save wifi config to prefs
            JsonDocument wifi_config;
            deserializeJson(wifi_config, prefs.getString("config", "{}"));
            for (int i = 0; i < doc["wifi"].size(); i++) {
                wifi_config["wifi"].add(doc["wifi"][i]);
            }
            String wifi_config_json;
            serializeJson(wifi_config, wifi_config_json);
            logger.log("Saving wifi config: " + wifi_config_json);
        }
        if (doc.containsKey("power")) {
            if (doc["power"].containsKey("voltage")) {
                power_voltage = doc["power"]["voltage"];
            }
        }
        if (doc.containsKey("motor")) {
            if (doc["motor"].containsKey("percent")) {
                motor_percent = doc["motor"]["percent"];
                prefs.putLong("motor_percent", motor_percent);
            }
            if (doc["motor"].containsKey("dir")) {
                motor_diroption = doc["motor"]["diroption"].as<uint8_t>() % 4;
                prefs.putChar("motor_diroption", motor_diroption);
            }
            if (doc["motor"].containsKey("interval")) {
                motor_interval = doc["motor"]["interval"];
                prefs.putLong("motor_interval", motor_interval);
            }
        }
        config_update = true;
    });
    mqtt.begin();
    logger.setMqtt(&mqtt, MQTT_TOPIC);
    // displayStatus();

    PageSysStatus *statusPage = new PageSysStatus(&WiFi, &MQTT_TOPIC);
    menu.addPage(statusPage);
    logger.log(String(menu.getPagesCount()));
    PageLogger *loggerPage = new PageLogger(&logger);
    menu.addPage(loggerPage);
    logger.log(String(menu.getPagesCount()));
    menu.draw(&display);

    // Interrupt sensor switch
    pinMode(PIN_SENSOR, INPUT_PULLUP);
    ts_sensor_last = millis();
    ts_sensor_this = millis();
    sensor_switch_update = false;
    attachInterrupt(digitalPinToInterrupt(PIN_SENSOR), SensorSwitchInterrupt, RISING);

    // Initialize motor
    pinMode(PIN_MOTOR_PWM_1, OUTPUT);
    pinMode(PIN_MOTOR_PWM_2, OUTPUT);
    digitalWrite(PIN_MOTOR_PWM_2, LOW);
    analogWrite(PIN_MOTOR_PWM_1, 127);
    delay(20);
    digitalWrite(PIN_MOTOR_PWM_1, LOW);
    analogWrite(PIN_MOTOR_PWM_2, 127);
    delay(20);
    digitalWrite(PIN_MOTOR_PWM_1, LOW);
    digitalWrite(PIN_MOTOR_PWM_2, LOW);

    // Initialize spo2 sensor
    mx30102_available = particleSensor.begin();
    for (int i = 0; i < MX3010X_MAX_RETRY; i++) {
        if (mx30102_available) {
            break;
        }
        mx30102_available = particleSensor.begin();
        logger.log("MAX30105 is not loaded, retrying.");
        delay(100);
    }
    if (mx30102_available) {
        byte ledBrightness = prefs.getChar("led_brightness", 60); // Options: 0=Off to 255=50mA
        byte sampleAverage = prefs.getChar("sample_average", 4);  // Options: 1, 2, 4, 8, 16, 32
        byte ledMode = prefs.getChar("led_mode", 2);              // Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
        // Real sample rate may be only about 20hz
        byte sampleRate = prefs.getChar("sample_rate", 200); // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
        int pulseWidth = prefs.getInt("pulse_width", 215);   // Options: 69, 118, 215, 411
        int adcRange = prefs.getInt("adc_range", 4096);      // Options: 2048, 4096, 8192, 16384
        particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
        logger.log("Red,IR=" + String(particleSensor.getRed()) + "," + particleSensor.getIR());
        logger.log("MAX30105 is set");
        digitalWrite(PIN_READ_LED, LOW);
    } else {
        logger.log("MAX30105 is not loaded, skipping.");
        mx30102_available = false;
    }

    // Initialize SGP30 sensor
    sgp30_available = sgp.begin();
    if (sgp30_available) {
        ts_last_sgp30_update = millis();
        ts_last_sgp30_baseline = millis();
        // set baseline if available
        uint16_t eco2_base = prefs.getUShort("sgp30_eco2_base", 0);
        uint16_t tvoc_base = prefs.getUShort("sgp30_tvoc_base", 0);
        if (eco2_base > 0 && tvoc_base > 0) {
            sgp.setIAQBaseline(eco2_base, tvoc_base);
            logger.log("Set SGP30 base line: " + String(eco2_base) + "," + String(tvoc_base));
        } else {
            logger.log("SGP30 base line not found");
        }
        logger.log("SGP30 is loaded");
    } else {
        logger.log("SGP30 is not loaded, skipping.");
    }

    // init aht20
    aht20_available = aht.begin();
    if (aht20_available) {
        logger.log("AHT20 is loaded");
    } else {
        logger.log("AHT20 is not loaded, skipping.");
    }

    // init bmp280
    bmp280_available = bmp.begin();
    if (bmp280_available) {
        logger.log("BMP280 is loaded");
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                        Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                        Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                        Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                        Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */
    } else {
        logger.log("BMP280 is not loaded, skipping.");
    }
}

void loop() {
    mqtt.loop();
    if (millis() - ts_last_screen_update > INTERVAL_SCREEN_UPDATE) {
        BtnEvent evnet = btn.pop();
        if (evnet.btn != BTN_NONE) {
            menu.onBtn(evnet);
        }
        menu.draw(&display);
    }

    // Motor update
    writeMotor();

    // Watchdog
    if (millis() - ts_last_watchdog_ok > INTERVAL_WATCHDOG_MILLIS) {
        // TODO: Watchdog task
        ts_last_watchdog_ok = millis();
        JsonDocument doc;
        doc["ts"] = millis();
        doc["watchdog"]["ok"] = true;

        String res;
        serializeJson(doc, res);
        mqtt.publish(MQTT_TOPIC, res);
    }

    // Config update
    if (config_update) {
        config_update = false;
        prefs.putLong("motor_interval", motor_interval);
        prefs.putLong("motor_percent", motor_percent);
        prefs.putChar("motor_diroption", motor_diroption);
        logger.log("Config updated");
    }

    // Publish basic info every 5 seconds
    if (millis() - ts_last_basic_msg > INTERVAL_BASIC_MSG_MILLIS || millis() < ts_last_basic_msg) {
        ts_last_basic_msg = millis();
        JsonDocument doc;
        doc["ts"] = millis();
        if (WiFi.getMode() == WIFI_AP) {
            doc["wifi"]["mode"] = "AP";
            doc["wifi"]["ip"] = String(WiFi.softAPIP().toString());
            doc["wifi"]["mac"] = WiFi.softAPmacAddress();
            doc["wifi"]["hostname"] = WiFi.getHostname();
            doc["wifi"]["status"] = WiFi.status();
            doc["wifi"]["ssid"] = WiFi.softAPSSID();
        } else {
            doc["wifi"]["mode"] = "STA";
            doc["wifi"]["ip"] = String(WiFi.localIP().toString());
            doc["wifi"]["mac"] = WiFi.macAddress();
            doc["wifi"]["hostname"] = WiFi.getHostname();
            doc["wifi"]["status"] = WiFi.status();
            doc["wifi"]["ssid"] = WiFi.SSID();
            doc["wifi"]["rssi"] = WiFi.RSSI();
        }

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

        doc["motor"]["current"] = motor_pwm_1 + motor_pwm_2;
        doc["motor"]["dir"] = motor_pwm_1 > motor_pwm_1;
        doc["motor"]["dir_option"] = motor_diroption;
        doc["motor"]["pwm_1"] = motor_pwm_1;
        doc["motor"]["pwm_2"] = motor_pwm_2;
        doc["motor"]["percent"] = motor_percent;
        doc["motor"]["interval"] = motor_interval;

        doc["power"]["voltage"] = power_voltage;

        doc["sgp30"]["ok"] = sgp30_available;
        doc["sgp30"]["tvoc"] = sgp.TVOC;
        doc["sgp30"]["eco2"] = sgp.eCO2;
        doc["sgp30"]["rawH2"] = sgp.rawH2;
        doc["sgp30"]["rawEthanol"] = sgp.rawEthanol;

        String res;
        serializeJson(doc, res);
        mqtt.publish(MQTT_TOPIC, res);
    }

    if (sensor_switch_update && sensor_switch_count % 2 == 0) {
        sensor_switch_update = false;
        logger.log("stop motor");
        setMotor(0, current_dir);
        ts_last_stop = millis();
    }

    // restart motor
    if (millis() - ts_last_stop > motor_interval && (motor_pwm_1 + motor_pwm_2) <= 0) {
        calcMotor();
    }

    // If we have enough data, run the algorithm
    if (mx30102_available && readSensor() && irBuffer.isFull()) {
        ts_last_btn_press = millis();
        if (current_screen == SCREEN_OFF) {
            current_screen = SCREEN_CONF;
        }
        redBuffer.copyToArray(redArray);
        irBuffer.copyToArray(irArray);

        maxim_heart_rate_and_oxygen_saturation(irArray, BUFFER_LENGTH, redArray, &spo2, &validSPO2, &heartRate, &validHeartRate);
        heartRate = heartRate * sample_freq_hz / FreqS;
        if ((validSPO2 && validHeartRate && sample_freq_hz == TARGET_FREQ) || random(1000) <= 1) {
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
            doc["mx30102"]["freq"] = sample_freq_hz;

            String res;
            serializeJson(doc, res);
            mqtt.publish(MQTT_TOPIC, res);
        }
    }

    // SGP30
    if (sgp30_available && millis() - ts_last_sgp30_update > INTERVAL_SGP30_UPDATE_MILLIS) {
        ts_last_sgp30_update = millis();

        // test aht and bmp
        if (aht20_available) {
            sensors_event_t humidity, temp;
            if (!aht.getEvent(&humidity, &temp)) {
                logger.log("AHT20 reading failed");
            }
            logger.log("AHT20: " + String(temp.temperature) + "C " + String(humidity.relative_humidity) + "%");
            sgp.setHumidity(getAbsoluteHumidity(temp.temperature, humidity.relative_humidity));
        }
        if (bmp280_available) {
            logger.log("BMP280: " + String(bmp.readTemperature()) + "C; " + String(bmp.readPressure()) + "Pa; " +
                       String(bmp.readAltitude()) + "m");
        }

        if (!sgp.IAQmeasure()) {
            logger.log("SGP30 IAQ measure failed");
        }
        if (!sgp.IAQmeasureRaw()) {
            logger.log("SGP30 IAQ measure raw failed");
        }
        logger.log("SGP30: TVOC " + String(sgp.TVOC) + " ppb, eCO2 " + String(sgp.eCO2) + " ppm, raw H2 " + String(sgp.rawH2) +
                   " raw Ethanol " + String(sgp.rawEthanol));

        // get base line
        if (millis() - ts_last_sgp30_baseline > INTERVAL_SGP30_BASELINE_MILLIS) {
            ts_last_sgp30_baseline = millis();
            JsonDocument doc;
            doc["ts"] = millis();
            doc["sgp30"]["tvoc"] = sgp.TVOC;
            doc["sgp30"]["eco2"] = sgp.eCO2;
            doc["sgp30"]["rawH2"] = sgp.rawH2;
            doc["sgp30"]["rawEthanol"] = sgp.rawEthanol;
            uint16_t eco2_base, tvoc_base;
            if (sgp.getIAQBaseline(&eco2_base, &tvoc_base)) {
                doc["sgp30"]["eco2_base"] = eco2_base;
                doc["sgp30"]["tvoc_base"] = tvoc_base;
            }
            // Save base line
            prefs.putUShort("sgp30_eco2_base", eco2_base);
            prefs.putUShort("sgp30_tvoc_base", tvoc_base);
            logger.log("Saved SGP30 base line: " + String(eco2_base) + "," + String(tvoc_base));
            String res;
            serializeJson(doc, res);
            mqtt.publish(MQTT_TOPIC, res);
        }
    }
}