#include "controlstatuspublisher.h"

ControlStatusPublisher::ControlStatusPublisher(JoystickHandler* joystick, QObject *parent):
    QObject(parent), m_joystick(joystick) {

    ros::NodeHandle n;
    m_publisher = n.advertise<std_msgs::String>("control_status", 1000);
    connect(joystick, &JoystickHandler::axisControlStatusChanged, this, &ControlStatusPublisher::onControlStatusChanged);
}

void ControlStatusPublisher::onControlStatusChanged(QString axis, bool enabled) {
    QString jsonStr("{axis: \"");
    jsonStr = jsonStr.append(axis)
            .append("\", enabled: ")
            .append(enabled? "true" : "false")
            .append("}");
    std_msgs::String msg;
    msg.data = jsonStr.toStdString();
    m_publisher.publish(msg);
}
