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
}

void MainWindow::handleTimer() {
    --m_time;
    QString minutes = QString("%1").arg(m_time/60, 2, 10, QChar('0'));
    QString seconds = QString("%1").arg(m_time%60, 2, 10, QChar('0'));
    ui->lblTimer->setText(minutes + ":" + seconds);
}

void MainWindow::onAxisChanged(float x, float y, float z) {
    qDebug() << "Axis Changed: " << x << "," << y << "," << z;
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

MainWindow::~MainWindow()
{
    if (m_joystick != nullptr) {
        m_joystick->close();
        delete m_joystick;
        m_joystick = nullptr;
    }
    delete ui;
}
