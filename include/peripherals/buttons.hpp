/*
QMesh
Copyright (C) 2021 Daniel R. Fay

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

#include "mbed.h"

extern volatile bool rebooting;

void reboot_system();

/**
 * Class that provides useful functionality around the board's
 * pushbuttons.
 */
class PushButton {
private:
    bool was_pressed;
    InterruptIn *btn;
public:
    /**
     * Constructor.
     * @param button The pin controlled by the pushbutton.
     */
    explicit PushButton(PinName button);

        /// Since you can't really share pins, the copy constructor and
    /// copy assignment operators don't make any sense.
    PushButton(const PushButton &old) = delete;
    auto operator= (const PushButton &) -> PushButton & = delete;

    /// Move constructor and move assignment operators do make sense, however.
    auto operator= (PushButton &&) -> PushButton & = default;	
    PushButton(PushButton&& other) = default;

    /**
     * Set the event queue to be used.
     * @param evt_queue reference to the EventQueue to be used.
     */
    void SetQueue(EventQueue &evt_queue);

    /// Interrupt handler that gets called on a button press.
    void btnInterrupt();
    
    /// Determine whether button was pressed between now and 
    /// the last time getPressed() was called.
    auto getPressed() -> bool;

    /// Destructor.
    ~PushButton();
};

//extern PushButton button;

#endif /* BUTTONS_HPP */