#include "include/connectiondialog.h"
#include "include/mainwindow.h"
#include "ui_connectiondialog.h"
#include <QDebug>
#include <QHostAddress>
#include <QNetworkInterface>

ConnectionDialog::ConnectionDialog(int argc, char** argv, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::ConnectionDialog),
    m_argc(argc), m_argv(argv)
{
    ui->setupUi(this);
    ui->lblError->setVisible(false);
    loadIPAddresses();

    rosNode = RosNode::getInstance();
    connect(rosNode, &RosNode::started, this, &ConnectionDialog::onRosConnected);
    connect(rosNode, &RosNode::stopped, this, &ConnectionDialog::onRosStopped);
    connect(rosNode, &RosNode::errorOccured, this, &ConnectionDialog::onRosError);
    connect(rosNode, &RosNode::masterDiconnected, this, &ConnectionDialog::onRosMasterDiconnected);
}

ConnectionDialog::~ConnectionDialog()
{
    rosNode->shutdown();
    delete ui;
}

void ConnectionDialog::on_btnConnect_clicked()
{
    ui->lblError->setVisible(false);
    ui->btnConnect->setEnabled(false);

    if (ui->chkDefaultSettings->isChecked())
        rosNode->init(m_argc, m_argv);
    else {
        QString masterUrl = ui->txtMasterUrl->text();
        QString hostIP = ui->cmbHostIP->currentText();
        if (!masterUrl.startsWith("http")) {
            if (!masterUrl.contains(":"))
                masterUrl += ":11311";
            masterUrl = "http://" + masterUrl;
        }
        rosNode->init(masterUrl.toStdString(), hostIP.toStdString());
//        RosService::initialize(masterUrl.toStdString(), hostIP.toStdString());
    }
}

void ConnectionDialog::loadIPAddresses() {
    const QHostAddress &localhost = QHostAddress(QHostAddress::LocalHost);
    bool defaultAddressChoosed = false;
    int i = 0;

    for (const QHostAddress &address: QNetworkInterface::allAddresses()) {
        if (address.protocol() == QAbstractSocket::IPv4Protocol) {
            ui->cmbHostIP->addItem(address.toString());

            if (!defaultAddressChoosed && address != localhost) {
                ui->cmbHostIP->setCurrentIndex(i);
                defaultAddressChoosed = true;
            }
            ++i;
        }
    }
}

void ConnectionDialog::on_cmbHostIP_currentIndexChanged(int index)
{
    if (!ui->txtMasterUrl->isModified()) {
        ui->txtMasterUrl->setText("http://" + ui->cmbHostIP->itemText(index) + ":11311/");
    }
}

void ConnectionDialog::on_btnCancel_clicked()
{
    close();
}

void ConnectionDialog::onRosConnected() {
    MainWindow* w = new MainWindow();
    w->show();
    close();
}

void ConnectionDialog::onRosError(const RosException &exception) {
    ui->lblError->setText("An Error occured while connecting to ROS");
    showError();
}

void ConnectionDialog::onRosMasterDiconnected() {
    ui->lblError->setText("Ros Master not running, Make sure that roscore is running");
    showError();
}

void ConnectionDialog::showError() {
    ui->btnConnect->setText("Try Again");
    ui->btnConnect->setEnabled(true);
    ui->lblError->setVisible(true);
    ui->cmbHostIP->setEnabled(false);
    ui->txtMasterUrl->setEnabled(false);
    ui->chkDefaultSettings->setEnabled(false);
}

void ConnectionDialog::onRosStopped() {
    qDebug() << "Ros Stopped from Connection Window";
    ui->btnConnect->setText("Connect");
    ui->btnConnect->setEnabled(true);
    ui->lblError->setVisible(false);
    ui->cmbHostIP->setEnabled(true);
    ui->txtMasterUrl->setEnabled(true);
    ui->chkDefaultSettings->setEnabled(true);
}
