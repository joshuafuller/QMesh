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

void reboot_system(void);

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