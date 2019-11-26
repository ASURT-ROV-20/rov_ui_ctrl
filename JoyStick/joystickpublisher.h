#ifndef JOYSTICKPUBLISHER_H
#define JOYSTICKPUBLISHER_H

#include <QObject>
#include <ros/ros.h>
#include <geometry_msgs/Quaternion.h>
#include "joystickhandler.h"

class JoystickPublisher : public QObject
{
    Q_OBJECT
public:
    explicit JoystickPublisher(JoystickHandler* joystick, QObject *parent = nullptr);

signals:

public slots:
    void onAxisChanged(const AxesValues &axesValues);

protected:
    void timerEvent(QTimerEvent *) override;

private:
    JoystickHandler* m_joystick;
    ros::Publisher m_publisher;
    float prevX, prevY, prevZ, prevR;
};

#endif // JOYSTICKPUBLISHER_H
