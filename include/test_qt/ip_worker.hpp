#ifndef IP_WORKER_H
#define IP_WORKER_H

#include <QObject>
#include <QProcess>
#include <QDebug>
#include <QFile>
#include <QtGui>

class ip_worker : public QObject
{
    Q_OBJECT
public:
    ip_worker();
    ~ip_worker();
public slots:
    void process();

signals:
    void finished(QStringList);

private:
    QProcess *_ip_scan_process;
};

#endif // IP_WORKER_H
