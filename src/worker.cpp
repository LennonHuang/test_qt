#include "../include/test_qt/worker.hpp"
#include <QtGui>
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
using namespace Qt;
Worker::Worker(QString port_name, QSerialPort::BaudRate port_rate): QObject()
{
    nano_imu_port = new QSerialPort(this);
    nano_imu_port->setPortName(port_name);
    nano_imu_port->setBaudRate(port_rate);
}

Worker::~Worker(){
}

void Worker::process(){
    is_processing = true;
    //try to connect to the serial port.
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
            error = "Other Error. Check whether the port is selected correctly.";
        }//More Error codes can be seen in Qt Documentation.
        //Emit the error for the main GUI to process.
        emit imu_error(error);
        emit finished();
        return;
    }
    //Connect successfully. Worker in a loop to read IMU, check connection error and light on/off LED.
    while (is_processing && nano_imu_port->isOpen()){
        //Command sent to the micro-controller (Arduino nano). The controller receives the command and response.
        //The receiver codes should be uploaded to the micro-controller.
        const char*command;
        if(is_led_on){
            command = "01\n";
        }else{
            command = "11\n";
        }
        nano_imu_port->write(command);
        //Wait till the micro-controller response with "\n" end and error free (code:0).
        while(!nano_imu_port->canReadLine() && nano_imu_port->error() == 0){
            nano_imu_port->waitForReadyRead();
        }
        //Check serial connection error every loop.
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
        //If error free, send the update imu signal to the main GUI.
        emit update_imu(QString(nano_imu_port->readLine()));
    }
    //
    //Break the loop, if any disconnection or error.
    //
    qDebug() << "Worker has finished work.";
    emit finished();
}

