#ifndef MENU_HPP
#define MENU_HPP

#include <Arduino.h>
#include <vector>
#include <Adafruit_SSD1306.h>

#include "page.hpp"
#include "btn.hpp"

class Menu{
    public:
        Menu();
        void addPage(Page* page);
        void draw(Adafruit_SSD1306 *display);
        void onBtn(BtnEvent event);
        void setDefaultPage(uint8_t page);
        size_t getPagesCount();
        void turnOff();
        void turnOn();
    private:
        std::vector<Page*> pages;
        uint8_t currentPage;
        boolean off;
        uint8_t defaultPage = 0;
};

#endif // MENU_HPP

