#ifndef JOYSTICK_HANDLER_H
#define JOYSTICK_HANDLER_H

#include "joystick.h"

class Joystick_handler : public QObject
{
    Q_OBJECT

private:
    //ROS Publisher
    float x;
    float y;
    float z;
    int btn_pressed;
    Joystick *myJoystick;
    QTimer * timer;
public:
    explicit Joystick_handler(QObject *parent = nullptr);
public slots:
    void update();
};

#endif // JOYSTICK_HANDLER_H
