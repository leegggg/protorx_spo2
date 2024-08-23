#include "page_sys_status.hpp"

PageSysStatus::PageSysStatus(WiFiClass * wifi, String *topic) {
    this->wifi = wifi;
    this->topic = topic;
}

void PageSysStatus::draw(Adafruit_SSD1306 *display) {
    display->clearDisplay();
    display->setCursor(0, 0);
    display->print("TS: " + String(millis() / 1000) + "  ");
    if (wifi->getMode() == WIFI_AP) {
        display->println("");
        display->println("AP: " + wifi->softAPSSID());
        display->println("IP: " + wifi->softAPIP().toString());
    } else {
        display->println("RSSI: " + String(wifi->RSSI()));
        display->println("IP: " + wifi->localIP().toString());
    }

    display->println("MQTT: ");
    display->println(*topic);
    // if (mx30102_available) {
    //     display->println("MX: " + String(irBuffer.last()) + "," + String(redBuffer.last()));
    //     display->println(String(sample_freq_hz) + "Hz");
    // }
    display->display();
}

void PageSysStatus::onBtn(BtnEvent event) {
    // do nothing
}

