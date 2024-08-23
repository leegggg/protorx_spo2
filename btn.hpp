#ifndef BTN_HPP
#define BTN_HPP

#define PIN_BTN_UP 21
#define PIN_BTN_DOWN 20
#define PIN_BTN_SHARP 3
#define PIN_BTN_STAR 4

#define ANTI_BOUNCE_MILLIS 100

enum BtnState { BTN_UP, BTN_DOWN, BTN_SHARP, BTN_STAR, BTN_OTHER, BTN_NONE };

struct BtnEvent {
    BtnState btn;
    long ts;
};

class Btn {
  public:
    Btn();
    void begin();
    BtnEvent pop();
    BtnEvent get();
    void pressed(BtnState btn);

  private:
    long ts_last_up;
    long ts_last_down;
    long ts_last_sharp;
    long ts_last_star;
    long ts_last_btn;
    BtnState btn_state;
};

#endif // BTN_HPP