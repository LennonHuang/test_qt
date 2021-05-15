#include "../include/test_qt/worker.hpp"
#include <QtGui>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
using namespace Qt;
Worker::Worker(): QObject()
{
}

Worker::~Worker(){
}

void Worker::process(){
    is_processing = true;
    nano_imu_port = new QSerialPort(this);
    nano_imu_port->setPortName("ttyACM0");
    nano_imu_port->setBaudRate(QSerialPort::Baud9600);
    nano_imu_port->open(QIODevice::ReadWrite);
    while (is_processing && nano_imu_port->isOpen()){
        const char*command;
        if(is_led_on){
            command = "11\n";
        }else{
            command = "01\n";
        }
        nano_imu_port->write(command);
        while(!nano_imu_port->canReadLine()){
            nano_imu_port->waitForReadyRead();
        }
        //qDebug()<< nano_imu_port->readLine();
        emit update_imu(QString(nano_imu_port->readLine()));
    }
    qDebug() << "hELLO";
    emit finished();
}

