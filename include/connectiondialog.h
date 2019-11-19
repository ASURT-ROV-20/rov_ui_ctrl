#ifndef CONNECTIONDIALOG_H
#define CONNECTIONDIALOG_H

#include <QDialog>
#include "include/rosnode.h"
#include "exceptions/rosexception.h"

namespace Ui {
class ConnectionDialog;
}

class ConnectionDialog : public QDialog
{
    Q_OBJECT

public:
    explicit ConnectionDialog(int argc, char** argv, QWidget *parent = nullptr);
    ~ConnectionDialog();

private slots:
    void onRosConnected();
    void onRosError(const RosException &exception);
    void onRosMasterDiconnected();
    void onRosStopped();

    void on_btnConnect_clicked();
    void loadIPAddresses();

    void on_cmbHostIP_currentIndexChanged(int index);

    void on_btnCancel_clicked();

private:
    Ui::ConnectionDialog *ui;
    void showError();

    int m_argc;
    char** m_argv;
    RosNode *rosNode;
};

#endif // CONNECTIONDIALOG_H
