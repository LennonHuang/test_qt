#include "../include/test_qt/gps_worker.hpp"
#include <QtGui>
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
using namespace Qt;
GPS_Worker::GPS_Worker(QString port_name, QString port_rate):QObject(){
    _port_name = port_name;
    _port_rate = port_rate;

    qDebug() << "Launching GPS";
}

GPS_Worker::~GPS_Worker(){
    qDebug() << "Delete gps_worker";
}

void GPS_Worker::process(){
    //Create the QProcess here. Cannot put it in Constructor otherwise it may have thread conflict since .moveToThread() is after the constructor.
    _gps_process = new QProcess();
    _gps_process->start("bash");
    _gps_process->write("rosrun nmea_navsat_driver nmea_serial_driver ");
    _gps_process->write("_port:=/dev/" + _port_name.toUtf8());
    _gps_process->write(" _baud:=" + _port_rate.toUtf8() + "\n");

    is_processing = true;
    //init Check
    _gps_process->waitForReadyRead(2000);
    QString _gps_error = QString(_gps_process->readAllStandardError());
    qDebug() << _gps_error;
    if(!_gps_error.isEmpty() && _gps_error.mid(0,6) != "[WARN]"){
        is_processing = false;
        emit error(_gps_error);
    }else{
        emit success_launch("GPS Launched");
        //TODO: Establish online checking
        while(is_processing){
            //Online Check Connection
        }
    }

    //Break the loop, if any disconnection or error.
    //
    _gps_process->close();;
    qDebug() << "GPS Worker has finished work.";
    emit finished();
}

