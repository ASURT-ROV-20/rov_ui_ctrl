#ifndef JOYSTICK_H
#define JOYSTICK_H

#include <QObject>
#include <SDL.h>
#include <SDL2/SDL.h>
#include <SDL2/SDL_joystick.h>
#include <QDebug>
#include <QThread>
#include <QTimer>
#include <QMutex>

#define JOYSTICK_DEAD_ZONE 2500
#define JOYSTICK_CHANGE_INTEVAL 1000
#define JOYSTICK_SCALE 32768
#define JOYSTICK_MIN_AXIS_VALUE -32768 + JOYSTICK_DEAD_ZONE
#define JOYSTICK_MAX_AXIS_VALUE 32768 - JOYSTICK_DEAD_ZONE
#define JOYSTICK_MIN_ZAXIS_VALUE 0 + JOYSTICK_DEAD_ZONE
#define JOYSTICK_MAX_ZAXIS_VALUE 32768 - JOYSTICK_DEAD_ZONE

enum JoystickAxis {AxisX = 0, AxisY = 1, AxisZ = 2, AxisR = 3};
Q_DECLARE_METATYPE(JoystickAxis);

enum JoystickButtonAction {Down, Up};
Q_DECLARE_METATYPE(JoystickButtonAction);

class Joystick : public QObject
{
    Q_OBJECT

public:
    static Joystick* getInstance();
    virtual ~Joystick();
    bool isConnected();
    void shutdown();
    QString getName();
    QString getGuid();

signals:
    void buttonAction(unsigned char btnNo, JoystickButtonAction action);
    void axisChanged(JoystickAxis axis, float value);
    void connected();
    void disconnected();

private slots:
    void run();

private:
    Joystick();
    void close();
    bool init();
    bool openJoystick(int deviceIndex);
    float scaleAxisValue(JoystickAxis axis, float value);

    SDL_Joystick* m_controller;

    bool m_isRunning;
    static Joystick* m_instance;
    QThread *m_thread;
    static QMutex m_instance_lock;
};

#endif // JOYSTICK_H
