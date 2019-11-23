#ifndef JOYSTICKPUBLISHER_H
#define JOYSTICKPUBLISHER_H

#include <QObject>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include "joystick.h"

class JoystickPublisher : public QObject
{
    Q_OBJECT
public:
    explicit JoystickPublisher(Joystick* joystick, QObject *parent = nullptr);

signals:

public slots:
    void onAxisChanged(float x, float y, float z, float r);

protected:
    void timerEvent(QTimerEvent *) override;

private:
    Joystick* m_joystick;
    ros::Publisher m_publisher;
    float prevX, prevY, prevZ, prevR;
};

#endif // JOYSTICKPUBLISHER_H
