#ifndef BUTTONS_HPP
#define BUTTONS_HPP

class PushButton {
protected:
    bool was_pressed;
    InterruptIn *btn;
public:
    PushButton(PinName button);

    void btnInterrupt(void);
    
    bool getPressed(void);

    ~PushButton();
};

extern PushButton button;

#endif /* BUTTONS_HPP */