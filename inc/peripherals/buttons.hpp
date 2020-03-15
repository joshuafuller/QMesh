/*
QMesh
Copyright (C) 2019 Daniel R. Fay

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef BUTTONS_HPP
#define BUTTONS_HPP

extern volatile bool rebooting;

void reboot_system(void);

void button_thread_fn(void);

/**
 * Class that provides useful functionality around the board's
 * pushbuttons.
 */
class PushButton {
protected:
    bool was_pressed;
    InterruptIn *btn;
    Thread btn_thread;
public:
    /**
     * Constructor.
     * @param button The pin controlled by the pushbutton.
     */
    PushButton(PinName button);

    /// Interrupt handler that gets called on a button press.
    void btnInterrupt(void);
    
    /// Determine whether button was pressed between now and 
    /// the last time getPressed() was called.
    bool getPressed(void);

    /// Destructor.
    ~PushButton();
};

//extern PushButton button;

#endif /* BUTTONS_HPP */