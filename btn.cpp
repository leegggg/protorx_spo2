#include "btn.hpp"
#include <Arduino.h>

Btn::Btn() {
    this->ts_last_btn = millis();
    this->btn_state = BTN_NONE;
}

void Btn::pressed(BtnState btn) {
    switch (btn){
        case BTN_UP:
            if(millis() - this->ts_last_up < ANTI_BOUNCE_MILLIS) {
                return;
            }
            this->ts_last_up = millis();
            this->ts_last_btn = this->ts_last_up;
            this->btn_state = BTN_UP;
            break;
        case BTN_DOWN:
            if(millis() - this->ts_last_down < ANTI_BOUNCE_MILLIS) {
                return;
            }
            this->ts_last_down = millis();
            this->ts_last_btn = this->ts_last_down;
            this->btn_state = BTN_DOWN;
            break;
        case BTN_SHARP:
            if(millis() - this->ts_last_sharp < ANTI_BOUNCE_MILLIS) {
                return;
            }
            this->ts_last_sharp = millis();
            this->ts_last_btn = this->ts_last_sharp;
            this->btn_state = BTN_SHARP;
            break;
        case BTN_STAR:
            if(millis() - this->ts_last_star < ANTI_BOUNCE_MILLIS) {
                return;
            }
            this->ts_last_star = millis();
            this->ts_last_btn = this->ts_last_star;
            this->btn_state = BTN_STAR;
            break;
        case BTN_NONE:
            this->btn_state = BTN_NONE;
            break;
        case BTN_OTHER:
            this->btn_state = BTN_OTHER;
            this->ts_last_btn = millis();
            break;
    default:
        break;
    }
}

BtnEvent Btn::get() {
    BtnEvent event;
    event.btn = this->btn_state;
    event.ts = this->ts_last_btn;
    return event;
}

BtnEvent Btn::pop() {
    BtnEvent event = this->get();
    this->pressed(BTN_NONE);
    return event;
}

void Btn::begin() {
    pinMode(PIN_BTN_UP, INPUT_PULLUP);
    pinMode(PIN_BTN_DOWN, INPUT_PULLUP);
    pinMode(PIN_BTN_SHARP, INPUT_PULLUP);
    pinMode(PIN_BTN_STAR, INPUT_PULLUP);
    this->pressed(BTN_NONE);
}


