#include "joystick_handler.h"

Joystick_handler::Joystick_handler(QObject *parent) : QObject(parent)
{
    myJoystick = new Joystick();
    timer = new QTimer();
    timer->setInterval(100);
    timer->start();
    btn_pressed = -1;

    connect(timer,SIGNAL(timeout()),this,SLOT(update()));
}

void Joystick_handler::update()
{
    myJoystick->joystick_update();
    x = myJoystick->getX();
    y = myJoystick->getY();
    z = myJoystick->getZ();
    x = (x / 32767.0f) * 100;
    y = (y / 32767.0f) * 100;
    z = (z / 32767.0f) * 100;
    // Send x,y,z to PI
    qDebug() << "x := " << x <<endl;
    qDebug() << "y := " << y <<endl;
    qDebug() << "z := " << z <<endl;
    if(myJoystick->pressed())
    {
        btn_pressed = myJoystick->getBtn_value();
        // Do function based on button pressed
        qDebug() << "Button " << btn_pressed << " was pressed\n";
    }
}
