#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
class Worker : public QObject
{
    Q_OBJECT
public:
    Worker();
    ~Worker();
    bool is_processing = false;
    bool is_led_on = false;
public slots:
    void process();

signals:
    void finished();
    void update_imu(QString);

private:
    QSerialPort *nano_imu_port;
    QStringList scanPort();

};

#endif // WORKER_H
