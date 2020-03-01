#include "cameracontrollerpublisher.h"

CameraControllerPublisher::CameraControllerPublisher(JoystickHandler* joystick, QObject *parent):
    QObject(parent), m_joystick(joystick) {

    ros::NodeHandle n;
    m_publisher = n.advertise<std_msgs::String>("rov_camera_servo", 1000);
    connect(joystick, &JoystickHandler::cameraMoved, this, &CameraControllerPublisher::onCameraMoved);
}

void CameraControllerPublisher::onCameraMoved(QString camId, CameraMovement movement) {
    std_msgs::String msg;
    QString str;
    str = str.append(camId).append(":");
    if (movement == MoveCameraUp) {
        str = str.append("1");
    } else {
        str = str.append("-1");
    }
    msg.data = str.toStdString();
    m_publisher.publish(msg);
}
