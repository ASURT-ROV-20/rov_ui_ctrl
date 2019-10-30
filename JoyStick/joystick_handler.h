#ifndef JOYSTICK_HANDLER_H
#define JOYSTICK_HANDLER_H

#include "joystick.h"
#include "ros/ros.h"
#include "geometry_msgs/Quaternion.h"
#include <std_msgs/String.h>

class Joystick_handler : public QObject
{
    Q_OBJECT

private:
    //ROS Publisher
    float x;
    float y;
    float z;
    float r;
    ros::Publisher my_publisher;
    ros::Rate *loop;
    int btn_pressed;
    Joystick *myJoystick;
    QTimer * timer;
    Q_SLOT void joystick_update();
    QThread joystick_thread;
public:
    explicit Joystick_handler(QObject *parent = nullptr);
public slots:
    void update();
};

#endif // JOYSTICK_HANDLER_H
