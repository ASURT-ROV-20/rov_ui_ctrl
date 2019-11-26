#include "pingsubscriber.h"

PingSubscriber::PingSubscriber(QObject *parent)
    : QObject(parent), lastPingTime(nullptr)
{
    // Register RosNodeInfo type, so we can send it as argument in signals
    qRegisterMetaType<PingInfo>();
    ros::NodeHandle n;
    m_subscriber = n.subscribe("/ping", 1000, &PingSubscriber::onPingReceived, this);
}

void PingSubscriber::onPingReceived(const std_msgs::Empty &) {
    m_lock.lock();
    delete lastPingTime;
    lastPingTime = new QDateTime(QDateTime::currentDateTime());
    m_lock.unlock();
    emit pingReceived(PingInfo());
}

QDateTime* PingSubscriber::getLastPingTime() {
    m_lock.lock();
    auto tmp = lastPingTime;
    m_lock.unlock();
    return tmp;
}

PingSubscriber::~PingSubscriber() {
    delete lastPingTime;
}

PingTimeoutHelper::PingTimeoutHelper(int timeoutIntervalMs, QObject* parent):
    QObject(parent), timeoutIntervalMs(timeoutIntervalMs), m_pingSubscriber(new PingSubscriber())
{
    timerId = startTimer(timeoutIntervalMs);
    connect(m_pingSubscriber, &PingSubscriber::pingReceived, this, &PingTimeoutHelper::onPingReceived);
}

PingSubscriber* PingTimeoutHelper::getSubscriber() {
    return m_pingSubscriber;
}

int PingTimeoutHelper::getTimeoutIntervalMs() {
    return timeoutIntervalMs;
}

void PingTimeoutHelper::timerEvent(QTimerEvent *) {
    emit pingTimeout();
}

void PingTimeoutHelper::onPingReceived(const PingInfo &pingInfo) {
    killTimer(timerId);
    startTimer(timeoutIntervalMs);
    emit pingReceived();
}

PingTimeoutHelper::~PingTimeoutHelper() {
    delete m_pingSubscriber;
}
