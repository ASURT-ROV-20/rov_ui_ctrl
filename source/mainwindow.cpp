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
}

void MainWindow::handleTimer() {
    --m_time;
    QString minutes = QString("%1").arg(m_time/60, 2, 10, QChar('0'));
    QString seconds = QString("%1").arg(m_time%60, 2, 10, QChar('0'));
    ui->lblTimer->setText(minutes + ":" + seconds);
}

MainWindow::~MainWindow()
{
    delete ui;
}
