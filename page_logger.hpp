#ifndef PAGE_LOGGER_HPP
#define PAGE_LOGGER_HPP

#include "page.hpp"
#include "logger.hpp"

class PageLogger : public Page {
    public:
        PageLogger(Logger* logger);
        void draw(Adafruit_SSD1306 *display);
        void onBtn(BtnEvent event);
    private:
        Logger* logger;
};

#endif // PAGE_LOGGER_HPP