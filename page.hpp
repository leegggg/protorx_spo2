#ifndef PAGE_HPP
#define PAGE_HPP

#include "btn.hpp"
#include <Adafruit_SSD1306.h>

class Page {
    public:
        virtual void draw(Adafruit_SSD1306 *display);
        virtual void onBtn(BtnEvent event);
};

#endif // PAGE_HPP