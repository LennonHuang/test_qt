#ifndef LASER_WORKER_H
#define LASER_WORKER_H

#include <QObject>
#include <QProcess>
#include <QDebug>
#include <QFile>
#include <QtGui>

class laser_worker : public QObject
{
    Q_OBJECT
public:
    laser_worker(QString hostname);
    ~laser_worker();
    bool is_processing = false;
public slots:
    void process();

signals:
    void finished();

private:
    QProcess *_laser_process;
    QString _hostname;
};

#endif // LASER_WORKER_H
