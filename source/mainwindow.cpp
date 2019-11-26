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

    gstreamer = new Gstreamer(ui);
    camera = new Camera(this, ui->camera1Wdgt, ui->camera2Wdgt, ui->camera3Wdgt);
    onChangeCameraLayout();
}

void MainWindow::initJoystick() {
    m_joystick = Joystick::getInstance();
    connect(m_joystick, &Joystick::axisChanged, this, &MainWindow::onAxisChanged);
    connect(m_joystick, &Joystick::buttonUp, this, &MainWindow::onButtonPressed);
    connect(m_joystick, &Joystick::buttonPressed, this, &MainWindow::onButtonUp);
    connect(m_joystick, &Joystick::connected, this, &MainWindow::onJoystickConnected);
    connect(m_joystick, &Joystick::disconnected, this, &MainWindow::onJoystickDisconnected);
    m_joystickPublisher = new JoystickPublisher(m_joystick, this);

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

void MainWindow::onAxisChanged(float x, float y, float z, float r) {
    qDebug() << "Axis Changed: " << x << "," << y << "," << z << ", " << r;
}

void MainWindow::onJoystickConnected() {
    joystickStatusLbl->setText(QString("Joystick connected ") + m_joystick->getName());
    joystickStatusLbl->setStyleSheet("QLabel {color: blue}");
}

void MainWindow::onJoystickDisconnected() {
    joystickStatusLbl->setText("Joystick disconnected");
    joystickStatusLbl->setStyleSheet("QLabel {color: red}");
}

void MainWindow::onButtonUp(int btnNo) {
    qDebug() << "Button Up: " << btnNo;
}

void MainWindow::onButtonPressed(int btnNo) {
    qDebug() << "Button Pressed: " << btnNo;
}


void MainWindow::onChangeCameraLayout()
{

    camera->toggleMode();
    camera->toggleMode();
    camera->toggleMode();
    camera->toggleMain();
}


MainWindow::~MainWindow()
{
    delete gstreamer;
    delete joystickStatusLbl;
    if (m_joystick != nullptr) {
        m_joystick->shutdown();
        delete m_joystick;
        m_joystick = nullptr;
    }
    delete ui;
}
