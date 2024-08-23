#ifndef PAGE_SYS_STATUS_HPP
#define PAGE_SYS_STATUS_HPP

#include "page.hpp"
#include <WiFi.h>

class PageSysStatus : public Page {
    public:
        PageSysStatus(WiFiClass * wifi, String *topic);
        void draw(Adafruit_SSD1306 *display);
        void onBtn(BtnEvent event);
    private:
        WiFiClass *wifi;
        String* topic;
};

#endif // PAGE_SYS_STATUS_HPP