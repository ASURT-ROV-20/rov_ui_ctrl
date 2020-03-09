#ifndef CONTROLSTATUSPUBLISHER_H
#define CONTROLSTATUSPUBLISHER_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "JoyStick/joystickhandler.h"

class ControlStatusPublisher : public QObject
{
    Q_OBJECT
public:
    explicit ControlStatusPublisher(JoystickHandler* joystick, QObject *parent = nullptr);

public slots:
    void onControlStatusChanged(QString axis, bool enabled);

private:
    JoystickHandler* m_joystick;
    ros::Publisher m_publisher;
};

#endif // CONTROLSTATUSPUBLISHER_H
