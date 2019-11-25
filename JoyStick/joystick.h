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

#define JOYSTICK_DEAD_ZONE 1000

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
    void buttonPressed(int btnNo);
    void buttonUp(int btnNo);
    void axisChanged(float x, float y, float z, float r);
    void connected();
    void disconnected();

private slots:
    void run();

private:
    Joystick();
    void close();
    bool init();
    bool openJoystick(int deviceIndex);

    SDL_Joystick* m_controller;

    bool m_isRunning;
    static Joystick* m_instance;
    QThread *m_thread;
    static QMutex m_instance_lock;
};

#endif // JOYSTICK_H
