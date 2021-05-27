#include "../include/test_qt/net_camera_worker.hpp"

net_camera_worker::net_camera_worker()
{
    qDebug() << "net camera worker init";
}

net_camera_worker::~net_camera_worker(){
    qDebug() << "net camera worker deconstructed";
}

void net_camera_worker::process(){
    _net_camera_process = new QProcess();
    _net_camera_process->start("bash");


    emit finished();
}
