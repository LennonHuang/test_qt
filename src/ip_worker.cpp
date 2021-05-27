#include "../include/test_qt/ip_worker.hpp"

ip_worker::ip_worker()
{
    qDebug() << "ip worker init";
}

ip_worker::~ip_worker(){
    qDebug() << "ip worker deconstructed";
}

void ip_worker::process(){
    _ip_scan_process = new QProcess();
    _ip_scan_process->start("bash");
    _ip_scan_process->write("nmap 192.168.0.1/24 -oX ip_file.xml \n");
    _ip_scan_process->waitForFinished();
    _ip_scan_process->close();
    qDebug() << "Done IP Scan";
    //Parse XML
    QFile *xmlFile = new QFile("ip_file.xml");
    if(!xmlFile->open(QIODevice::ReadOnly | QIODevice::Text)){
        qDebug() << "cannot open it";
    }else{
        qDebug() << "The file is open";
    }

    QXmlStreamReader *xml_reader = new QXmlStreamReader(xmlFile);
    QStringList scan_result;
    while(!xml_reader->atEnd() && !xml_reader->hasError()){
        if(xml_reader->readNext() == QXmlStreamReader::StartElement && xml_reader->name() == "address"){
            qDebug() << xml_reader->attributes().value("addr");
            scan_result.append(xml_reader->attributes().value("addr").toString());
        }
    }

    emit finished(scan_result);
}
