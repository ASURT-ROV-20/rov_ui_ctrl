#include "joystickpublisher.h"

JoystickPublisher::JoystickPublisher(Joystick* joystick, QObject *parent) : QObject(parent),
    m_joystick(joystick), prevX(0), prevY(0), prevZ(0)
{
    ros::NodeHandle n;
    m_publisher = n.advertise<geometry_msgs::Quaternion>("rov_velocity", 1000);
    connect(joystick, &Joystick::axisChanged, this, &JoystickPublisher::onAxisChanged);
    startTimer(100);
}

void JoystickPublisher::timerEvent(QTimerEvent *) {
    geometry_msgs::Quaternion msg;
    msg.x = prevX;
    msg.y = prevY;
    msg.z = prevZ;
    m_publisher.publish(msg);
}

void JoystickPublisher::onAxisChanged(float x, float y, float z) {
    prevX = x;
    prevY = y;
    prevZ = z;
}
