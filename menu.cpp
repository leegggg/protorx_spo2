#include "menu.hpp"

Menu::Menu() {
    currentPage = 0;
    pages = std::vector<Page *>();
    off = false;
}

void Menu::addPage(Page *page) { pages.push_back(page); }

void Menu::draw(Adafruit_SSD1306 *display) {
    if (getPagesCount() == 0)
        return;
    if (off) {
        display->clearDisplay();
        display->display();
    } else {
        currentPage = currentPage % getPagesCount();
        pages[currentPage]->draw(display);
    }
}

void Menu::onBtn(BtnEvent event) {
    if (getPagesCount() == 0)
        return;
    if (event.btn == BTN_STAR) {
        off = false;
        currentPage = (currentPage + 1) % getPagesCount();
    } else {
        currentPage = currentPage % getPagesCount();
        pages[currentPage]->onBtn(event);
    }
}

void Menu::setDefaultPage(uint8_t page) { defaultPage = page % getPagesCount(); }

size_t Menu::getPagesCount() { return pages.size(); }

void Menu::turnOff() { off = true; }
void Menu::turnOn() { off = false; }