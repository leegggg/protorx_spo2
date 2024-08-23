#include "page_logger.hpp"

PageLogger::PageLogger(Logger *logger) { this->logger = logger; }

void PageLogger::draw(Adafruit_SSD1306 *display) {
    display->clearDisplay();
    display->setTextSize(1);
    display->setTextColor(WHITE);
    display->setCursor(0, 0);
    display->println(String("L:") + String(logger->currentIndex() + 1) + String("/") + String(LOG_BUFFER_SIZE) + String(" ") + logger->get());
    display->display();
}

void PageLogger::onBtn(BtnEvent event) {
    switch (event.btn) {
    case BTN_UP:
        logger->last();
        break;
    case BTN_DOWN:
        logger->next();
        break;
    case BTN_SHARP:
        logger->tail();
        break;
    default:
        break;
    }
}
