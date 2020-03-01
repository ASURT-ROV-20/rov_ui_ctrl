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
    initPing();

    gstreamer = new Gstreamer(ui);
    camera = new Camera(this, ui->camera1Wdgt, ui->camera2Wdgt, ui->camera3Wdgt);
}

void MainWindow::initJoystick() {
    m_joystick = Joystick::getInstance();
    m_joystickHandler = new JoystickHandler(m_joystick, this);
    connect(m_joystickHandler, &JoystickHandler::axisChanged, this, &MainWindow::onAxisChanged);
    connect(m_joystick, &Joystick::connected, this, &MainWindow::onJoystickConnected);
    connect(m_joystick, &Joystick::disconnected, this, &MainWindow::onJoystickDisconnected);
    connect(m_joystickHandler, &JoystickHandler::changeCameraMode, this, &MainWindow::onChangeCameraMode);
    connect(m_joystickHandler, &JoystickHandler::changeMainCamera, this, &MainWindow::onChangeMainCamera);
    m_joystickPublisher = new JoystickPublisher(m_joystickHandler, this);
    m_camPublisher = new CameraControllerPublisher(m_joystickHandler, this);

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

void MainWindow::closeJoystick() {
    delete joystickStatusLbl;
    if (m_joystick != nullptr) {
        m_joystick->shutdown();
        delete m_joystick;
        m_joystick = nullptr;
    }
}

void MainWindow::initPing() {
    m_pingSubscriber = new PingTimeoutHelper();
    connect(m_pingSubscriber, &PingTimeoutHelper::pingReceived, this, &MainWindow::onPingReceived);
    connect(m_pingSubscriber, &PingTimeoutHelper::pingTimeout, this, &MainWindow::onPingTimeout);

    pingStatusLbl = new QLabel();
    ui->statusbar->addPermanentWidget(pingStatusLbl);
}

void MainWindow::closePing() {
    delete pingStatusLbl;
    delete m_pingSubscriber;
}

void MainWindow::onPingReceived() {
    QString timeStr = m_pingSubscriber->getSubscriber()->getLastPingTime()->time().toString();
    pingStatusLbl->setText("Ping received at: " + timeStr);
    pingStatusLbl->setStyleSheet("QLabel {color: blue}");
}

void MainWindow::onPingTimeout() {
    if (m_pingSubscriber->getSubscriber()->getLastPingTime() == nullptr) {
        pingStatusLbl->setText("No Ping Received");
    } else {
        QString timeStr = m_pingSubscriber->getSubscriber()->getLastPingTime()->time().toString();
        pingStatusLbl->setText("No Ping Received since: " + timeStr);
    }
    pingStatusLbl->setStyleSheet("QLabel {color: red}");
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
    camera->toggleMode();
}

void MainWindow::onChangeMainCamera() {
    camera->toggleMain();
}

void MainWindow::onJoystickConnected() {
    joystickStatusLbl->setText(QString("Joystick connected ") + m_joystick->getName());
    joystickStatusLbl->setStyleSheet("QLabel {color: blue}");
}

void MainWindow::onJoystickDisconnected() {
    joystickStatusLbl->setText("Joystick disconnected");
    joystickStatusLbl->setStyleSheet("QLabel {color: red}");
}


MainWindow::~MainWindow()
{
    delete gstreamer;
    closeJoystick();
    closePing();
    delete m_camPublisher;
    delete ui;
}
