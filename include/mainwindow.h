#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QDebug>
#include "JoyStick/joystick.h"
#include "JoyStick/joystickpublisher.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Q_SLOT void handleTimer();
    Q_SLOT void onAxisChanged(float x, float y, float z);
    Q_SLOT void onJoystickConnected();
    Q_SLOT void onJoystickDisconnected();
    Q_SLOT void onButtonPressed(int btnNo);
    Q_SLOT void onButtonUp(int btnNo);

    Ui::MainWindow *ui;
    unsigned int m_time;
    QTimer *m_timer;

    Joystick* m_joystick;
    JoystickPublisher* m_joystickPublisher;
};

#endif // MAINWINDOW_H
