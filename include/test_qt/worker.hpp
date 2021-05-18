#ifndef WORKER_H
#define WORKER_H

#include <QObject>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
class Worker : public QObject
{
    Q_OBJECT
public:
    Worker(QString port_name, QSerialPort::BaudRate port_rate);
    ~Worker();
    bool is_processing = false;
    bool is_led_on = false;
public slots:
    void process();

signals:
    void finished();
    void update_imu(QString);
    void imu_error(QString e);

private:
    QSerialPort *nano_imu_port;
    QStringList scanPort();

};

#endif // WORKER_H
