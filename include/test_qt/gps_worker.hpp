#ifndef GPS_WORKER_H
#define GPS_WORKER_H

#include <QObject>
#include <QProcess>

class GPS_Worker : public QObject
{
    Q_OBJECT
public:
    GPS_Worker(QString port_name, QString port_rate);
    ~GPS_Worker();
    bool is_processing = false;
public slots:
    void process();

signals:
    void finished();
    void update(QString);
    void error(QString e);
    void success_launch(QString state);

private:
    QProcess *_gps_process;
    QString _port_name;
    QString _port_rate;
};

#endif // WORKER_H
