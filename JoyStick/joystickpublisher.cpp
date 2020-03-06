#include "joystickpublisher.h"

JoystickPublisher::JoystickPublisher(JoystickHandler* joystick, QObject *parent) : QObject(parent),
    m_joystick(joystick), prevX(0), prevY(0), prevZ(0)
{
    ros::NodeHandle n;
    m_publisher = n.advertise<geometry_msgs::Quaternion>("rov_velocity", 1000);
    connect(joystick, &JoystickHandler::axisChanged, this, &JoystickPublisher::onAxisChanged);
    startTimer(100);
}

void JoystickPublisher::timerEvent(QTimerEvent *) {
    geometry_msgs::Quaternion msg;
    msg.x = -prevX;
    msg.y = prevY;
    msg.z = prevZ;
    msg.w = prevR;
    m_publisher.publish(msg);
}

void JoystickPublisher::onAxisChanged(const AxesValues &axesValues) {
    prevX = axesValues.x;
    prevY = axesValues.y;
    prevZ = axesValues.z;
    prevR = axesValues.r;
    timerEvent(nullptr);
}
