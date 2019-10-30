#include "joystick.h"
#include <QThread>

Joystick::Joystick(QObject *parent) : QObject(parent)
{
    SDL_Init(SDL_INIT_JOYSTICK);
    btn_pressed = false;
    btns = SDL_JoystickNumButtons(myJoystick);
    myJoystick = SDL_JoystickOpen(0);
    if(myJoystick == NULL)
    {
        qDebug() << "No joystick connected";
        exit(1);
    }
    qDebug() << SDL_JoystickNumAxes(myJoystick);
    qDebug() << SDL_JoystickNumButtons(myJoystick);
    //Initiate the publisher and start it, make it of type int
    prevx = 0; prevy = 0; prevz = 0;
}

bool Joystick::pressed() { return btn_pressed; }

int Joystick::getX() { return x; }

int Joystick::getY() { return y; }

int Joystick::getZ() { return z; }

int Joystick::getBtn_value() { return btn_value; }

void Joystick::joystick_update()
{
    if(abs(SDL_JoystickGetAxis(myJoystick,0) - prevx) > 2500)
    {
        x = SDL_JoystickGetAxis(myJoystick,0);
    }
    if(abs(SDL_JoystickGetAxis(myJoystick,1) - prevy) > 2500)
    {
        y = SDL_JoystickGetAxis(myJoystick,1);
    }
    if(abs(SDL_JoystickGetAxis(myJoystick,2) - prevz) > 2500)
    {
        z = SDL_JoystickGetAxis(myJoystick,2);
    }
    for (int i = 0;i<btns;i++)
    {
        if(SDL_JoystickGetButton(myJoystick,i) == 1)
        {
            btn_value = i;
            btn_pressed = true;
            break;
        }
        else
        {
          btn_pressed = false;
        }
    }
}
