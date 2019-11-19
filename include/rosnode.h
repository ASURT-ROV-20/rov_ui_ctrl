#ifndef ROSNODE_H
#define ROSNODE_H

#include <QObject>
#include <QThread>

#include <ros/ros.h>
#include <string>
#include <QMutex>

#include "exceptions/rosexception.h"

class RosNode : public QObject
{
    Q_OBJECT
public:
    static RosNode* getInstance();
    static void dispose();
    void init(int argc, char** argv, std::string const &nodeName = "ROV_UI_CTRL");
    void init(std::string const &masterUrl, std::string const &hostIP, std::string const &nodeName = "ROV_UI_CTRL");
    void shutdown();
    ~RosNode();

signals:
    void started();
    void stopped();
    void errorOccured(const RosException exception);
    void masterDiconnected();

private slots:
    void run();

private:
    RosNode(QObject* parent = nullptr);

    QThread *m_thread;
    static RosNode* instance;
    static bool isInitialized;
    static QMutex instanceLock;
    QMutex initLock;

    std::string nodeName;
    int m_argc;
    char** m_argv;
    std::string masterUrl;
    std::string hostIP;
    // Indicate if node is intialized from command lines arguments or from remappings
    bool useArgs;
};

#endif // ROSNODE_H
