#ifndef NET_CAMERA_WORKER_H
#define NET_CAMERA_WORKER_H

#include <QObject>
#include <QProcess>
#include <QDebug>
#include <QFile>
#include <QtGui>

class net_camera_worker : public QObject
{
    Q_OBJECT
public:
    net_camera_worker();
    ~net_camera_worker();
public slots:
    void process();

signals:
    void finished();

private:
    QProcess *_net_camera_process;
};

#endif // NET_CAMERA_WORKER_H
