#include <QApplication>
#include "JoyStick/joystick_handler.h"
#include "ros/ros.h"
int main(int argc, char** argv) {
    std::map<std::string, std::string> remapping;
    remapping.insert(std::make_pair("__master", "http://Phobia-XPS-13:11311/"));
    remapping.insert(std::make_pair("__hostname", "127.0.0.1"));
    ros::init(remapping, "talker");

    if (!ros::master::check()) {
        qDebug() << "Couldn't connect";
    }

    ros::start();
    ros::Time::init();

    Joystick_handler *mine = new Joystick_handler();
    QApplication a(argc, argv);
    return 0;
}
