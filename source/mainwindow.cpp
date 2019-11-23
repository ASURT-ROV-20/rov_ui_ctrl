#include "include/mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    m_time = 15*60;
    m_timer = new QTimer(this);
    connect(m_timer, &QTimer::timeout, this, &MainWindow::handleTimer);
    m_timer->start(1000);

    m_joystick = Joystick::getInstance();
    connect(m_joystick, &Joystick::axisChanged, this, &MainWindow::onAxisChanged);
    connect(m_joystick, &Joystick::buttonUp, this, &MainWindow::onButtonPressed);
    connect(m_joystick, &Joystick::buttonPressed, this, &MainWindow::onButtonUp);
    connect(m_joystick, &Joystick::connected, this, &MainWindow::onJoystickConnected);
    connect(m_joystick, &Joystick::disconnected, this, &MainWindow::onJoystickDisconnected);
    m_joystickPublisher = new JoystickPublisher(m_joystick, this);

    initGstreamer();
}

void MainWindow::handleTimer() {
    --m_time;
    QString minutes = QString("%1").arg(m_time/60, 2, 10, QChar('0'));
    QString seconds = QString("%1").arg(m_time%60, 2, 10, QChar('0'));
    ui->lblTimer->setText(minutes + ":" + seconds);
}

void MainWindow::onAxisChanged(float x, float y, float z, float r) {
    qDebug() << "Axis Changed: " << x << "," << y << "," << z << ", " << r;
}

void MainWindow::onJoystickConnected() {
    qDebug() << "Joystick Connected";
}

void MainWindow::onJoystickDisconnected() {
    qDebug() << "Joystick Disconnected";
}

void MainWindow::onButtonUp(int btnNo) {
    qDebug() << "Button Up: " << btnNo;
}

void MainWindow::onButtonPressed(int btnNo) {
    qDebug() << "Button Pressed: " << btnNo;
}

void MainWindow::initGstreamer() {
    gst_init(nullptr, nullptr);
    gst_pipline1 = gst_parse_launch("udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegparse ! jpegdec ! videoconvert ! videoscale ! ximagesink  name=mySink", nullptr);
    gst_sink1 = gst_bin_get_by_name((GstBin*)gst_pipline1,"mySink");

    WId xwinid = ui->camera1Wdgt->winId();
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (gst_sink1), (guintptr)xwinid);
    gst_element_set_state (gst_pipline1, GST_STATE_PLAYING);

    gst_pipline2 = gst_parse_launch("udpsrc port=5001 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegparse ! jpegdec ! videoconvert ! videoscale ! ximagesink  name=mySink2", nullptr);
    gst_sink2 = gst_bin_get_by_name((GstBin*)gst_pipline1,"mySink2");

    WId xwinid2 = ui->camera2Wdgt->winId();
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (gst_sink2), (guintptr)xwinid2);
    gst_element_set_state (gst_pipline2, GST_STATE_PLAYING);
}

void MainWindow::closeGstreamer() {
    gst_object_unref(gst_pipline1);
    gst_object_unref(gst_pipline2);
}

MainWindow::~MainWindow()
{
    closeGstreamer();
    if (m_joystick != nullptr) {
        m_joystick->close();
        delete m_joystick;
        m_joystick = nullptr;
    }
    delete ui;
}
