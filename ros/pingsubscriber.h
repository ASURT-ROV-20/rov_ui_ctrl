#ifndef PINGSUBSCRIBER_H
#define PINGSUBSCRIBER_H

#include <QObject>
#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <QDebug>
#include <QDateTime>
#include <QMutex>
#include <QTimer>

// May needed later to pass info about node sent the ping
struct PingInfo {
    PingInfo() {}
};
// Register RosNodeInfo type, so we can send it as argument in signals
Q_DECLARE_METATYPE(PingInfo);

class PingSubscriber : public QObject
{
    Q_OBJECT
public:
    explicit PingSubscriber(QObject *parent = nullptr);
    QDateTime* getLastPingTime();
    virtual ~PingSubscriber() override;

signals:
    void pingReceived(const PingInfo &pingInfo);

private:
    void onPingReceived(const std_msgs::Empty &msg);

    QDateTime* lastPingTime;
    ros::Subscriber m_subscriber;
    QMutex m_lock;
};

class PingTimeoutHelper : public QObject
{
    Q_OBJECT
public:
    PingTimeoutHelper(int timeoutIntervalMs = 250, QObject* parent = nullptr);
    PingSubscriber* getSubscriber();
    int getTimeoutIntervalMs();
    virtual ~PingTimeoutHelper() override;

protected:
    void timerEvent(QTimerEvent *) override;

signals:
    void pingTimeout();
    void pingReceived();

private slots:
    void onPingReceived(const PingInfo &pingInfo);

private:
    int timerId;
    int timeoutIntervalMs;
    PingSubscriber* m_pingSubscriber;
};

#endif // PINGSUBSCRIBER_H
