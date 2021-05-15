/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date February 2011
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui>
#include <QMessageBox>
#include <iostream>
#include "../include/test_qt/main_window.hpp"
#include <QtSerialPort/QSerialPort>
#include <QtSerialPort/QSerialPortInfo>
/*****************************************************************************
** Namespaces
*****************************************************************************/

namespace test_qt {

using namespace Qt;

/*****************************************************************************
** Implementation [MainWindow]
*****************************************************************************/

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
	: QMainWindow(parent)
	, qnode(argc,argv)
{
	ui.setupUi(this); // Calling this incidentally connects all ui's triggers to on_...() callbacks in this class.
    QObject::connect(ui.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt())); // qApp is a global variable for the application

    ReadSettings();
	setWindowIcon(QIcon(":/images/icon.png"));
	ui.tab_manager->setCurrentIndex(0); // ensure the first tab is showing - qt-designer should have this already hardwired, but often loses it (settings?).
    QObject::connect(&qnode, SIGNAL(rosShutdown()), this, SLOT(close()));

	/*********************
	** Logging
	**********************/
	ui.view_logging->setModel(qnode.loggingModel());
    QObject::connect(&qnode, SIGNAL(loggingUpdated()), this, SLOT(updateLoggingView()));

    /*********************
    ** Auto Start
    **********************/
    if ( ui.checkbox_remember_settings->isChecked() ) {
        on_button_connect_clicked(true);
    }
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::showNoMasterMessage() {
	QMessageBox msgBox;
	msgBox.setText("Couldn't find the ros master.");
	msgBox.exec();
    close();
}

/*
 * These triggers whenever the button is clicked, regardless of whether it
 * is already checked or not.
 */

void MainWindow::on_button_connect_clicked(bool check ) {
	if ( ui.checkbox_use_environment->isChecked() ) {
		if ( !qnode.init() ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
		}
	} else {
		if ( ! qnode.init(ui.line_edit_master->text().toStdString(),
				   ui.line_edit_host->text().toStdString()) ) {
			showNoMasterMessage();
		} else {
			ui.button_connect->setEnabled(false);
			ui.line_edit_master->setReadOnly(true);
			ui.line_edit_host->setReadOnly(true);
			ui.line_edit_topic->setReadOnly(true);
		}
	}
}


void MainWindow::on_checkbox_use_environment_stateChanged(int state) {
	bool enabled;
	if ( state == 0 ) {
		enabled = true;
	} else {
		enabled = false;
	}
	ui.line_edit_master->setEnabled(enabled);
	ui.line_edit_host->setEnabled(enabled);
	//ui.line_edit_topic->setEnabled(enabled);
}

/*****************************************************************************
** Implemenation [Slots][manually connected]
*****************************************************************************/

/**
 * This function is signalled by the underlying model. When the model changes,
 * this will drop the cursor down to the last line in the QListview to ensure
 * the user can always see the latest log message.
 */
void MainWindow::updateLoggingView() {
        ui.view_logging->scrollToBottom();
}

/*****************************************************************************
** Implementation [Menu]
*****************************************************************************/

void MainWindow::on_actionAbout_triggered() {
    QMessageBox::about(this, tr("About ..."),tr("<h2>PACKAGE_NAME Test Program 0.10</h2><p>Copyright Yujin Robot</p><p>This package needs an about description.</p>"));
}

/*****************************************************************************
** Implementation [Configuration]
*****************************************************************************/

void MainWindow::ReadSettings() {
    QSettings settings("Qt-Ros Package", "test_qt");
    restoreGeometry(settings.value("geometry").toByteArray());
    restoreState(settings.value("windowState").toByteArray());
    QString master_url = settings.value("master_url",QString("http://192.168.1.2:11311/")).toString();
    QString host_url = settings.value("host_url", QString("192.168.1.3")).toString();
    //QString topic_name = settings.value("topic_name", QString("/chatter")).toString();
    ui.line_edit_master->setText(master_url);
    ui.line_edit_host->setText(host_url);
    //ui.line_edit_topic->setText(topic_name);
    bool remember = settings.value("remember_settings", false).toBool();
    ui.checkbox_remember_settings->setChecked(remember);
    bool checked = settings.value("use_environment_variables", false).toBool();
    ui.checkbox_use_environment->setChecked(checked);
    if ( checked ) {
    	ui.line_edit_master->setEnabled(false);
    	ui.line_edit_host->setEnabled(false);
    	//ui.line_edit_topic->setEnabled(false);
    }
}

void MainWindow::WriteSettings() {
    QSettings settings("Qt-Ros Package", "test_qt");
    settings.setValue("master_url",ui.line_edit_master->text());
    settings.setValue("host_url",ui.line_edit_host->text());
    //settings.setValue("topic_name",ui.line_edit_topic->text());
    settings.setValue("use_environment_variables",QVariant(ui.checkbox_use_environment->isChecked()));
    settings.setValue("geometry", saveGeometry());
    settings.setValue("windowState", saveState());
    settings.setValue("remember_settings",QVariant(ui.checkbox_remember_settings->isChecked()));

}

void MainWindow::closeEvent(QCloseEvent *event)
{
	WriteSettings();
	QMainWindow::closeEvent(event);
}

void MainWindow::window_update_imu(QString imu_data){{
        ui.gyro_label->setText(imu_data);
                                                     }
}

QStringList MainWindow::scanPort(){
    QStringList scan_result;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        scan_result.append(info.portName());
    }
    return scan_result;
}

void MainWindow::on_led_btn_clicked(){
    if(!nano_worker->is_led_on){
        nano_worker->is_led_on = true;
        ui.led_btn->setText("LED on");
    }else{
        nano_worker->is_led_on = false;
        ui.led_btn->setText("LED off");
    }
}

void MainWindow::on_imu_connect_btn_clicked(){
    QThread *imu_thread = new QThread();
    nano_worker = new Worker();
    nano_worker->moveToThread(imu_thread);
    //Connect the worker and thread
    connect( imu_thread, &QThread::started, nano_worker, &Worker::process);
    connect( nano_worker, &Worker::finished, imu_thread, &QThread::quit);
    connect( nano_worker, &Worker::finished, nano_worker, &Worker::deleteLater);
    connect( imu_thread, &QThread::finished, imu_thread, &QThread::deleteLater);
    connect( nano_worker, SIGNAL(update_imu(QString)), this, SLOT(window_update_imu(QString)));
    imu_thread->start();
    ui.imu_connect_btn->setEnabled(false);
    ui.imu_status_label->setText("IMU Connected");

}

void MainWindow::on_imu_disconnect_btn_clicked(){
    nano_worker->is_processing = false;
    emit nano_worker->finished();

    ui.imu_connect_btn->setEnabled(true);
    ui.imu_disconnect_btn->setEnabled(true);
    ui.imu_status_label->setText("IMU Disconnected");
}

void MainWindow::on_imu_refresh_btn_clicked(){
    ui.imu_connect_btn->setEnabled(true);
    ui.imu_disconnect_btn->setEnabled(true);
    ui.imu_status_label->setText("Refreshed");
}
void MainWindow::on_serial_scan_btn_clicked(){
    ui.serial_comboBox->clear();
    QStringList qls = scanPort();
    ui.serial_comboBox->addItems(qls);
}

void MainWindow::on_plugin_test_btn_clicked(){
    if(!plugin_on){
       plg = new QProcess(this);
       plugin_on = true;
       plg->start("bash");
    }
    plg->write("rqt_mypkg\n");
    std::cout << "Processing" << std::endl;

}

}  // namespace test_qt

