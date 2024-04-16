#include <JOYSTICK.h>


int get_x_pos(){
    return analogRead(JOYSTICK_X);
}

int get_y_pos(){
    return analogRead(JOYSTICK_Y);
}


