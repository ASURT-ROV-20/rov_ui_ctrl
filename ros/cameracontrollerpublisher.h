#ifndef CAMERACONTROLLERPUBLISHER_H
#define CAMERACONTROLLERPUBLISHER_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include "JoyStick/joystickhandler.h"
#include "JoyStick/joystickhandler.h"

class CameraControllerPublisher : public QObject
{
    Q_OBJECT
public:
    explicit CameraControllerPublisher(JoystickHandler* joystick, QObject *parent = nullptr);

public slots:
    void onCameraMoved(QString camId, CameraMovement movement);

private:
    JoystickHandler* m_joystick;
    ros::Publisher m_publisher;
};

#endif // CAMERACONTROLLERPUBLISHER_H
