#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QObject>
#include "SDL2/SDL.h"
#include "SDL2/SDL_joystick.h"
#include "QDebug"
#include "QThread"
#include <QTimer>

class Joystick : public QObject
{
    Q_OBJECT

private:
    SDL_Joystick *myJoystick;
    int prevx, prevy, prevz, x, y, z, btn_value, btns;
    bool btn_pressed;
public:
    explicit Joystick(QObject *parent = nullptr);

    bool pressed();

    int getX();

    int getY();

    int getZ();

    int getBtn_value();

    void joystick_update();

};

#endif // JOYSTICK_H
