#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QTimer>

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

    Ui::MainWindow *ui;
    unsigned int m_time;
    QTimer *m_timer;
};

#endif // MAINWINDOW_H
