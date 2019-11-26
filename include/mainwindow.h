#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QDebug>
#include "JoyStick/joystick.h"
#include "JoyStick/joystickhandler.h"
#include "JoyStick/joystickpublisher.h"
#include "include/gstreamer.h"
#include "include/camera.h"

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

signals:
    void cameraLayoutChanged();

private:
    Q_SLOT void handleTimer();
    Q_SLOT void onAxisChanged(const AxesValues &values);
    Q_SLOT void onJoystickConnected();
    Q_SLOT void onJoystickDisconnected();
    Q_SLOT void onChangeCameraMode();
    Q_SLOT void onChangeMainCamera();

    void initJoystick();
    void closeJoystick();

    void toggleCamera();

    Ui::MainWindow *ui;
    unsigned int m_time;
    QTimer *m_timer;

    Joystick* m_joystick;
    JoystickHandler* m_joystickHandler;
    JoystickPublisher* m_joystickPublisher;

    QLabel* joystickStatusLbl;

    Gstreamer * gstreamer;

    Camera * camera;

};

#endif // MAINWINDOW_H
