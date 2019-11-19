#include <QApplication>
#include "include/connectiondialog.h"
#include "include/rosnode.h"

int main(int argc, char** argv) {
    QApplication a(argc, argv);
    ConnectionDialog d(argc, argv);
    d.show();
    int r = a.exec();
    RosNode::dispose();
    return r;
}
