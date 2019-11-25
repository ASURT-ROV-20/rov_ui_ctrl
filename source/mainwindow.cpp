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

    initJoystick();

    initGstreamer();
}

void MainWindow::initJoystick() {
    m_joystick = Joystick::getInstance();
    m_joystickHandler = new JoystickHandler(m_joystick, this);
    connect(m_joystickHandler, &JoystickHandler::axisChanged, this, &MainWindow::onAxisChanged);
    connect(m_joystick, &Joystick::connected, this, &MainWindow::onJoystickConnected);
    connect(m_joystick, &Joystick::disconnected, this, &MainWindow::onJoystickDisconnected);
    connect(m_joystickHandler, &JoystickHandler::changeCameraMode, this, &MainWindow::onChangeCameraMode);
    m_joystickPublisher = new JoystickPublisher(m_joystickHandler, this);

    joystickStatusLbl = new QLabel(this);
    ui->statusbar->addPermanentWidget(joystickStatusLbl);

    if (m_joystick->isConnected()) {
        joystickStatusLbl->setText(QString("Joystick connected ") + m_joystick->getName());
        joystickStatusLbl->setStyleSheet("QLabel {color: blue}");
    } else {
        joystickStatusLbl->setText("Joystick disconnected");
        joystickStatusLbl->setStyleSheet("QLabel {color: red}");
    }
}

void MainWindow::handleTimer() {
    --m_time;
    QString minutes = QString("%1").arg(m_time/60, 2, 10, QChar('0'));
    QString seconds = QString("%1").arg(m_time%60, 2, 10, QChar('0'));
    ui->lblTimer->setText(minutes + ":" + seconds);
}

void MainWindow::onAxisChanged(const AxesValues &values) {
    qDebug() << "Axis Changed: " << values.x << "," << values.y << "," << values.z << ", " << values.r;
}

void MainWindow::onChangeCameraMode() {
    // TODO: Change Camera Display Mode
}

void MainWindow::onJoystickConnected() {
    joystickStatusLbl->setText(QString("Joystick connected ") + m_joystick->getName());
    joystickStatusLbl->setStyleSheet("QLabel {color: blue}");
}

void MainWindow::onJoystickDisconnected() {
    joystickStatusLbl->setText("Joystick disconnected");
    joystickStatusLbl->setStyleSheet("QLabel {color: red}");
}

void MainWindow::initGstreamer() {
    gst_init(nullptr, nullptr);
    gst_pipline1 = gst_parse_launch("udpsrc port=5000 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegparse ! jpegdec ! videoconvert ! videoscale ! ximagesink  name=mySink", nullptr);
    gst_sink1 = gst_bin_get_by_name((GstBin*)gst_pipline1,"mySink");

    WId xwinid = ui->camera1Wdgt->winId();
    gst_video_overlay_set_window_handle (GST_VIDEO_OVERLAY (gst_sink1), (guintptr)xwinid);
    gst_element_set_state (gst_pipline1, GST_STATE_PLAYING);

    gst_pipline2 = gst_parse_launch("udpsrc port=5001 ! application/x-rtp,encoding-name=JPEG,payload=26 ! rtpjpegdepay ! jpegparse ! jpegdec ! videoconvert ! videoscale ! ximagesink  name=mySink2", nullptr);
    gst_sink2 = gst_bin_get_by_name((GstBin*)gst_pipline2,"mySink2");

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
    delete joystickStatusLbl;
    if (m_joystick != nullptr) {
        m_joystick->shutdown();
        delete m_joystick;
        m_joystick = nullptr;
    }
    delete ui;
}
