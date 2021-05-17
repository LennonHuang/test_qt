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
    try {
        if(!nano_imu_port->open(QIODevice::ReadWrite)){
            throw nano_imu_port->error();
        }
    } catch (QSerialPort::SerialPortError e) {
        qDebug() << e;
        QString error;
        if (e == 1){
         error = "Device Not Found";
        }else{
            error = "Other Error";
        }//More Error codes can be seen in Qt Documentation.
        emit imu_error(error);
        emit finished();
        return;
    }
    while (is_processing && nano_imu_port->isOpen()){
        const char*command;
        if(is_led_on){
            command = "01\n";
        }else{
            command = "11\n";
        }
        nano_imu_port->write(command);
        while(!nano_imu_port->canReadLine() && nano_imu_port->error() == 0){
            nano_imu_port->waitForReadyRead();
        }
        QSerialPort::SerialPortError e_duringWork = nano_imu_port->error();
        qDebug() << "checking error...";
        if(e_duringWork != 0){
            qDebug() << e_duringWork;
            qDebug() << "Error Occurred during work.";
            QString error_duringWork;
            error_duringWork = "Device Not Available";
            emit imu_error(error_duringWork);
            break;
        }
        emit update_imu(QString(nano_imu_port->readLine()));
    }
    qDebug() << "Worker has finished work.";
    emit finished();
}

