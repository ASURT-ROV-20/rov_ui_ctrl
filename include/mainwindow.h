#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QLabel>
#include <QDebug>
#include "JoyStick/joystick.h"
#include "JoyStick/joystickhandler.h"
#include "JoyStick/joystickpublisher.h"

#include <glib-object.h>
#include <gst/gst.h>
#include <gst/video/video.h>

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
    Q_SLOT void onAxisChanged(const AxesValues &values);
    Q_SLOT void onJoystickConnected();
    Q_SLOT void onJoystickDisconnected();
    Q_SLOT void onChangeCameraMode();

    void initGstreamer();
    void closeGstreamer();
    void initJoystick();
    void closeJoystick();

    Ui::MainWindow *ui;
    unsigned int m_time;
    QTimer *m_timer;

    GstElement* gst_pipline1;
    GstElement* gst_sink1;
    GstElement* gst_pipline2;
    GstElement* gst_sink2;

    Joystick* m_joystick;
    JoystickHandler* m_joystickHandler;
    JoystickPublisher* m_joystickPublisher;

    QLabel* joystickStatusLbl;
};

#endif // MAINWINDOW_H
