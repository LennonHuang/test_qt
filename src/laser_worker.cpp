#include "../include/test_qt/laser_worker.hpp"

laser_worker::laser_worker(QString hostname)
{
    _hostname = hostname;
    qDebug() << "laser worker init";
}

laser_worker::~laser_worker(){
    qDebug() << "laser worker deconstructed";
}

void laser_worker::process(){
    _laser_process = new QProcess();
    _laser_process->start("bash");
    _laser_process->write("roslaunch sick_tim sick_tim571_2050101.launch ip:=" + _hostname.toUtf8() + " \n");//arg ip define in launch file
    _laser_process->waitForReadyRead(3000);
    qDebug() << _laser_process->readAllStandardError();
    is_processing = true;
    qDebug() << _hostname;
    while(is_processing){
    }

    _laser_process->close();
    _laser_process->start("bash");
    _laser_process->write("rosnode kill /sick_tim571_2050101 \n");


    emit finished();
}
