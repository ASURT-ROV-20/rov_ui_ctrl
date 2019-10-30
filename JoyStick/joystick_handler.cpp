#include "joystick_handler.h"

Joystick_handler::Joystick_handler(QObject *parent) : QObject(parent)
{
    ros::NodeHandle n;
    this->moveToThread(&joystick_thread);
    connect(&joystick_thread, &QThread::started, this, &Joystick_handler::joystick_update);
    joystick_thread.start();
    my_publisher = n.advertise<std_msgs::String>("to_be_named_qt", 100);
    myJoystick = new Joystick();
    timer = new QTimer();
    timer->setInterval(100);
    timer->start();
    btn_pressed = -1;
    loop = new ros::Rate(10);
    connect(timer,SIGNAL(timeout()),this,SLOT(update()));
}

void Joystick_handler::joystick_update()
{
    while(ros::ok())
    {
        std_msgs::String msg;
        msg.data = "RRRRRRRRRRRRRR";
        qDebug() << "Publishing...";
        my_publisher.publish(msg);
        qDebug() << "Publishing1...";
        ros::spinOnce();
        qDebug() << "Publishing...2";
        loop->sleep();
    }

}
void Joystick_handler::update()
{
    myJoystick->joystick_update();
    x = myJoystick->getX();
    y = myJoystick->getY();
    z = myJoystick->getZ();
    x = (x / 32767.0f);
    y = (y / 32767.0f);
    z = (z / 32767.0f);
    r = 1;
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
