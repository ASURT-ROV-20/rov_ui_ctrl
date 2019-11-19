#include "../include/rosnode.h"
#include "QDebug"
#include <QMetaType>

RosNode::RosNode(QObject* parent)
    : QObject(parent), m_thread(nullptr), m_argc(0), m_argv(nullptr) {
    qRegisterMetaType<RosException>("RosException");
}

RosNode* RosNode::getInstance() {
//    RosNode::instanceLock.lock();
    if (instance == nullptr)
        instance = new RosNode();
//    RosNode::instanceLock.unlock();
    return instance;
}

void RosNode::init(std::string const &masterUrl, std::string const &hostIP, std::string const &nodeName) {
    initLock.lock();
    if (isInitialized)
        return;
    isInitialized = true;
    initLock.unlock();

    useArgs = false;
    this->masterUrl = masterUrl;
    this->hostIP = hostIP;
    this->nodeName = nodeName;

    m_thread = new QThread(this);
    moveToThread(m_thread);
    connect(m_thread, &QThread::started, this, &RosNode::run);
    m_thread->start();
}

void RosNode::init(int argc, char **argv, const std::string &nodeName) {
    initLock.lock();
    if (isInitialized)
        return;
    isInitialized = true;
    initLock.unlock();

    useArgs = true;
    this->m_argc = argc;
    this->m_argv = argv;
    this->nodeName = nodeName;

    m_thread = new QThread(this);
    moveToThread(m_thread);
    connect(m_thread, &QThread::started, this, &RosNode::run);
    m_thread->start();
}

void RosNode::run() {
    try {
        if (useArgs)
            ros::init(m_argc, m_argv, nodeName);
        else {
            std::map<std::string, std::string> remappings;
            remappings["__master"] = masterUrl;
            remappings["__hostname"] = hostIP;
            ros::init(remappings, nodeName);
        }
    } catch (...) {
        emit errorOccured(RosException());
    }

    if (!ros::master::check() && ros::ok())
        emit masterDiconnected();
    while (!ros::master::check() && ros::ok())
        QThread::msleep(100);
    if (ros::ok()) {
        ros::start();
        ros::Time::init();
    }

    while (ros::ok()) {
        emit started();
        while (ros::master::check() && ros::ok()) {
            for (int i = 0; i < 100 && ros::ok(); ++i) {
                ros::spinOnce();
                QThread::msleep(10);
            }
        }

        if (!ros::ok())
            break;
        emit masterDiconnected();
        while(!ros::master::check() && ros::ok()) {
            QThread::msleep(100);
        }
    }

    ros::waitForShutdown();
    emit stopped();
    m_thread->exit();
}

void RosNode::shutdown() {
    if (ros::ok())
        ros::shutdown();
}

RosNode::~RosNode() {
    if (ros::ok()) {
        ros::shutdown();
        ros::waitForShutdown();
    }

    if (m_thread != nullptr) {
        if (m_thread->isRunning()) {
            m_thread->exit();
        }
        m_thread->wait(1000);
        delete m_thread;
        m_thread = nullptr;
    }
}

void RosNode::dispose() {
    if (instance != nullptr) {
        instance->shutdown();
        ros::waitForShutdown();
        delete instance;
        instance = nullptr;
    }
}

bool RosNode::isInitialized = false;
RosNode* RosNode::instance = nullptr;
