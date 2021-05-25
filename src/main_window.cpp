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
#include <QDebug>
#include <QFileDialog>
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


    //Init All RVIZ Elements
    //init the tree widget
    ui.rviz_treeWidget->setEnabled(false);
    //init the fixed frame setting
    QTreeWidgetItem *coordinate = new QTreeWidgetItem(QStringList() << "Frame Name");
    QTreeWidgetItem *fixed_frame = new QTreeWidgetItem(QStringList() << "Fixed Frame");
    fixed_frame_box = new QComboBox();
    fixed_frame_box->addItems(QStringList() << "gps" <<"odom" << "laser_link" << "map");
    fixed_frame_box->setEditable(true);
    ui.rviz_treeWidget->addTopLevelItem(coordinate);
    //Connect the box, the key and value (both of them are QTreeWidgets)
    coordinate->addChild(fixed_frame);
    ui.rviz_treeWidget->setItemWidget(fixed_frame, 1, fixed_frame_box);
    connect(fixed_frame_box,SIGNAL(currentTextChanged(QString)),this,SLOT(slot_fixed_frame_changed(QString)));


    //init tf//
    QTreeWidgetItem *tf = new QTreeWidgetItem(QStringList() << "Display All Frame");
    tf_check_box = new QCheckBox();
    ui.rviz_treeWidget->addTopLevelItem(tf);
    ui.rviz_treeWidget->setItemWidget(tf,1,tf_check_box);
    //conenct checkbox and tf display
    connect(tf_check_box,SIGNAL(stateChanged(int)),this,SLOT(slot_mainwindow_display_tf(int)));

    //init grid//
    QTreeWidgetItem *grid = new QTreeWidgetItem(QStringList() << "Grid");
    //grid children
    QTreeWidgetItem *grid_color = new QTreeWidgetItem(QStringList() << "Color (RGB)");
    QTreeWidgetItem *grid_cell_num = new QTreeWidgetItem(QStringList() << "Cell Number");
    QTreeWidgetItem *grid_cell_size = new QTreeWidgetItem(QStringList() << "Cell Size (m)");
    grid->addChild(grid_color);
    grid->addChild(grid_cell_num);
    grid->addChild(grid_cell_size);
    //grid item widget
    ui.rviz_treeWidget->addTopLevelItem(grid);
    grid_checkbox = new QCheckBox();
    //grid_checkbox->setChecked(false);
    ui.rviz_treeWidget->setItemWidget(grid,1,grid_checkbox);
    //grid children item widget
    grid_cell_num_box = new QSpinBox();
    grid_cell_num_box->setValue(16);
    ui.rviz_treeWidget->setItemWidget(grid_cell_num,1,grid_cell_num_box);
    grid_color_box = new QComboBox();
    grid_color_box->addItem("160;160;160");
    ui.rviz_treeWidget->setItemWidget(grid_color,1,grid_color_box);
    grid_cell_size_box = new QDoubleSpinBox();
    grid_cell_size_box->setValue(0.5);
    ui.rviz_treeWidget->setItemWidget(grid_cell_size,1,grid_cell_size_box);
    grid->setExpanded(true);
    //connect the checkbox on mainwindow to update the grid
    connect(grid_checkbox, SIGNAL(stateChanged(int)),this,SLOT(slot_mainwindow_display_grid(int)));

    //init GPS//
    QTreeWidgetItem *gps = new QTreeWidgetItem(QStringList() << "Display GPS Map");
    gps_check_box = new QCheckBox();
    ui.rviz_treeWidget->addTopLevelItem(gps);
    ui.rviz_treeWidget->setItemWidget(gps,1,gps_check_box);
    //conenct checkbox and tf display
    connect(gps_check_box,SIGNAL(stateChanged(int)),this,SLOT(slot_mainwindow_display_gps(int)));
    //connect MainWindow display
    connect(&qnode, SIGNAL(update_gps(const sensor_msgs::NavSatFix)), this,SLOT(slot_table_display_gps(const sensor_msgs::NavSatFix)));

    //init laser scan//
    QTreeWidgetItem *laser_scan = new QTreeWidgetItem(QStringList() << "Laser");
    scan_check_box = new QCheckBox();
    ui.rviz_treeWidget->addTopLevelItem(laser_scan);
    ui.rviz_treeWidget->setItemWidget(laser_scan,1,scan_check_box);
    //laser property
    QTreeWidgetItem *scan_topic = new QTreeWidgetItem(QStringList() << "Laser Scan Topic");
    scan_topic_name_box = new QComboBox();
    scan_topic_name_box->addItems(QStringList() << "scan");
    laser_scan->addChild(scan_topic);
    ui.rviz_treeWidget->setItemWidget(scan_topic,1,scan_topic_name_box);
    QTreeWidgetItem *laser_point_size = new QTreeWidgetItem(QStringList() << "Point Cloud Size");
    laser_point_size_box = new QDoubleSpinBox();
    laser_point_size_box->setValue(0.02);
    laser_scan->addChild(laser_point_size);
    ui.rviz_treeWidget->setItemWidget(laser_point_size,1,laser_point_size_box);
    laser_scan->setExpanded(true);
    //conenct checkbox and laser display
    connect(scan_check_box,SIGNAL(stateChanged(int)),this,SLOT(slot_mainwindow_display_scan(int)));

    //init camera//
    QTreeWidgetItem *camera = new QTreeWidgetItem(QStringList() << "Camera");
    QCheckBox *camera_check_box = new QCheckBox();
    ui.rviz_treeWidget->addTopLevelItem(camera);
    ui.rviz_treeWidget->setItemWidget(camera,1,camera_check_box);
    //camera propety
    QTreeWidgetItem *camera_topic = new QTreeWidgetItem(QStringList() << "Camera Topic");
    camera_topic_name_box = new QComboBox();
    camera_topic_name_box->addItems(QStringList() << "/camera/rgb/image_raw");
    camera_topic_name_box->setEditable(true);
    camera->addChild(camera_topic);
    ui.rviz_treeWidget->setItemWidget(camera_topic,1,camera_topic_name_box);
    connect(camera_check_box,SIGNAL(stateChanged(int)),this,SLOT(slot_mainwindow_display_camera(int)));
}

MainWindow::~MainWindow() {}

/*****************************************************************************
** Implementation [Slots]
*****************************************************************************/

void MainWindow::slot_mainwindow_display_gps(int state){
    bool enable = state>=1?true:false;
    my_rviz->display_gps(enable);
}

void MainWindow::slot_mainwindow_display_tf(int state){
    bool enable = state>=1?true:false;
    my_rviz->display_tf(enable);
}

void MainWindow::slot_mainwindow_display_camera(int state){
    bool enable = state>=1?true:false;
    my_rviz->display_camera(enable,camera_topic_name_box->currentText());
}

void MainWindow::slot_mainwindow_display_scan(int state){
    bool enable = state>=1?true:false;
    my_rviz->display_scan(enable,scan_topic_name_box->currentText(),laser_point_size_box->text().toDouble());

}

void MainWindow::slot_mainwindow_display_grid(int state){
    bool enable = state>=1?true:false;
    //Construt QColor from RGB value.
    QStringList qli = grid_color_box->currentText().split(";");
    QColor color = QColor(qli[0].toInt(),qli[1].toInt(),qli[2].toInt());

    my_rviz->display_grid(enable,color,grid_cell_num_box->text().toInt(),grid_cell_size_box->text().toDouble());
}

void MainWindow::slot_fixed_frame_changed(QString qs){
    my_rviz->_manger->setFixedFrame(qs);
}

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
            my_rviz = new qrviz(ui.layout_rviz);//Create embedded RVIZ
            ui.rviz_treeWidget->setEnabled(true);
            //Display RVIZ
            grid_checkbox->setChecked(true);
            scan_check_box->setChecked(true);
            my_rviz->_manger->setFixedFrame(fixed_frame_box->currentText());
            slot_mainwindow_display_grid(true);
            slot_mainwindow_display_scan(true);
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
            my_rviz = new qrviz(ui.layout_rviz);//Create embedded RVIZ
            ui.rviz_treeWidget->setEnabled(true);
            //Display RVIZ
            grid_checkbox->setChecked(true);
            scan_check_box->setChecked(true);
            my_rviz->_manger->setFixedFrame(fixed_frame_box->currentText());
            slot_mainwindow_display_grid(true);
            slot_mainwindow_display_scan(true);
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
    QMessageBox::about(this, tr("About ..."),tr("..."));
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
    if(plg != nullptr){
        if(plg->state() == QProcess::ProcessState::Running){
            plg->close();
        }
    }
	QMainWindow::closeEvent(event);
}


//Once the worker emit update_imu, the Main Gui will update the display.
void MainWindow::window_update_imu(QString imu_data){
    QStringList imu_text = imu_data.split(";");
    ui.x_gyro_label->setText(imu_text[0]);
    ui.y_gyro_label->setText(imu_text[1]);
    ui.z_gyro_label->setText(imu_text[2]);
    ui.x_acc_label->setText(imu_text[3]);
    ui.y_acc_label->setText(imu_text[4]);
    ui.z_acc_label->setText(imu_text[5].trimmed());
}


//Control the LED
void MainWindow::on_led_btn_clicked(){
    if(!nano_worker->is_led_on){
        nano_worker->is_led_on = true;
        ui.led_btn->setText("LED on");
    }else{
        nano_worker->is_led_on = false;
        ui.led_btn->setText("LED off");
    }
}

void MainWindow::on_quit_button_clicked(){
    qDebug() << "Quiting MainWindow...";
}

//Try to connect to the imu//
//A new thread is created for not blocking the main GUI. A customized Worker object is added into the new thread.
//Once the QTheard has started, it will trigger Worker's process method.
//During Worker::process(), worker will emit "update_imu" signal (in a loop till exit command given) which triggers main GUI "window_update_imu".
//Worker will also check connection error during process, and if there is any error, signal "imu_error" will be emitted.
//Once the Worker has finished its job, the exit function of thread and the worker will execute.
void MainWindow::on_imu_connect_btn_clicked(){
    QThread *imu_thread = new QThread();
    nano_worker = new Worker(ui.serial_comboBox->currentText(),QSerialPort::Baud9600);
    nano_worker->moveToThread(imu_thread);
    //Connect the worker and thread//
    connect( imu_thread, &QThread::started, nano_worker, &Worker::process);
    connect( nano_worker, &Worker::finished, imu_thread, &QThread::quit);
    connect( nano_worker, &Worker::finished, nano_worker, &Worker::deleteLater);
    connect( imu_thread, &QThread::finished, imu_thread, &QThread::deleteLater);
    //Connect imu update and checking error signals with the main GUI window.
    connect( nano_worker, SIGNAL(update_imu(QString)), this, SLOT(window_update_imu(QString)));
    connect( nano_worker, SIGNAL(imu_error(QString)), this, SLOT(catch_imu_connection_error(QString)));
    //Start the thread which tigeer the worker to work.
    imu_thread->start();
    //Disable the button in case the user presses the connect button again.
    ui.imu_connect_btn->setEnabled(false);
    //Display the status label and indicate which port.
    ui.imu_status_label->setText("IMU Connected at " + ui.serial_comboBox->currentText());
}

//Fail from trying to connect the IMU. Display the error source.
void MainWindow::catch_imu_connection_error(QString error){
    ui.imu_status_label->setText("IMU " + error);
    QMessageBox::warning(this,tr("Connection Error"),tr("Please check the serial port!"),QMessageBox::Ok);
}

//Disconnect the Arduino.
void MainWindow::on_imu_disconnect_btn_clicked(){
    nano_worker->is_processing = false;//Break the worker processing loop.
    emit nano_worker->finished();
    //Update the error on display
    ui.imu_connect_btn->setEnabled(true);
    ui.imu_disconnect_btn->setEnabled(true);
    ui.imu_status_label->setText("IMU Disconnected");
}

//Refresh the Connect button.
void MainWindow::on_imu_refresh_btn_clicked(){
    ui.imu_connect_btn->setEnabled(true);
    ui.imu_disconnect_btn->setEnabled(true);
    ui.imu_status_label->setText("Refreshed");
}

//Scan avilable serial port and return the list.
QStringList MainWindow::scanPort(){
    QStringList scan_result;
    foreach (const QSerialPortInfo &info, QSerialPortInfo::availablePorts()) {
        scan_result.append(info.portName());
    }
    return scan_result;
}

//Display all avaiable ports for IMU
void MainWindow::on_serial_scan_btn_clicked(){
    ui.serial_comboBox->clear();
    QStringList qls = scanPort();
    ui.serial_comboBox->addItems(qls);
}

//Display all avaiable ports for GPS
void MainWindow::on_serial_scan_gps_btn_clicked(){
    ui.serial_gps_serial_ComboBox->clear();
    QStringList qls = scanPort();
    ui.serial_gps_serial_ComboBox->addItems(qls);
}

//This is a button test for Python plugin.
//Nothing but a bash shell.
void MainWindow::on_plugin_test_btn_clicked(){
    if(!plugin_on){
       plg = new QProcess(this);
       plugin_on = true;
       plg->start("bash");
    }
    plg->write("rqt_mypkg\n");
    std::cout << "Processing" << std::endl;
    plg->waitForStarted();

}

void MainWindow::on_gps_connect_btn_clicked(){
    QThread *gps_thread = new QThread();
    gps_worker = new GPS_Worker(ui.serial_gps_serial_ComboBox->currentText(),"9600");
    gps_worker->moveToThread(gps_thread);
    //Connect the worker and thread//
    connect( gps_thread, &QThread::started, gps_worker, &GPS_Worker::process);
    connect( gps_worker, &GPS_Worker::finished, gps_thread, &QThread::quit);
    connect( gps_worker, &GPS_Worker::finished, gps_worker, &GPS_Worker::deleteLater);
    connect( gps_thread, &QThread::finished, gps_thread, &QThread::deleteLater);
    //Connect worker signal with Main GUI
    connect(gps_worker, SIGNAL(success_launch(QString)), this, SLOT(window_gps_status(QString)));
    connect(gps_worker, SIGNAL(error(QString)), this, SLOT(window_gps_status(QString)));

    gps_thread->start();
    ui.gps_connect_btn->setEnabled(false);
    ui.gps_status_label->setText("Trying to connect to GPS...");
}

void MainWindow::on_gps_disconnect_btn_clicked(){
    //Break the loop inside GPS_Worker process()
    gps_worker->is_processing = false;
    ui.gps_connect_btn->setEnabled(true);
    ui.gps_disconnect_btn->setEnabled(false);
    QProcess *node_killer = new QProcess();
    node_killer->start("bash");
    node_killer->write("rosnode kill /nmea_serial_driver \n");
    ui.gps_status_label->setText("Disconnecting GPS");
    node_killer->waitForReadyRead(2000);
    qDebug() << node_killer->readAllStandardOutput();
    node_killer->close();
    ui.gps_status_label->setText("GPS disconnected");
}

void MainWindow::window_gps_status(QString status){
    if(status != "GPS Launched"){
        ui.gps_connect_btn->setEnabled(true);
        ui.gps_disconnect_btn->setEnabled(false);
        ui.gps_status_label->setText("Connect Again: " + status);
    }else{
        ui.gps_status_label->setText(status);
        ui.gps_disconnect_btn->setEnabled(true);
    }
}

void MainWindow::slot_table_display_gps(const sensor_msgs::NavSatFix msg){

    ui.label_altitude_num->setNum(msg.altitude);
    ui.label_latitude_num->setNum(msg.latitude);
    ui.label_longitude_num->setNum(msg.longitude);
    //qDebug("Heard From GPS");

}

void MainWindow::on_camera_scan_btn_clicked(){
    qDebug() << "Number of camera found: " << QCameraInfo::availableCameras().count();
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    foreach (const QCameraInfo &cameraInfo, cameras){
        qDebug() << "Camera Info: " << cameraInfo.deviceName() << cameraInfo.description() << cameraInfo.position();
        ui.camera_selection_box->addItem(cameraInfo.description());
    }
}

void MainWindow::on_camera_connect_btn_clicked(){
    if(!is_camera_connected){
        //Connect the camera
        connectCamera();
    }else{
        camera->stop();
        viewfinder->deleteLater();
        ui.camera_connect_btn->setText("Connect");
        is_camera_connected = false;
    }

}

void MainWindow::connectCamera(){
    QList<QCameraInfo> cameras = QCameraInfo::availableCameras();
    foreach (const QCameraInfo &cameraInfo, cameras){
        if(cameraInfo.description() == ui.camera_selection_box->currentText()){
            camera = new QCamera(cameraInfo);
            viewfinder = new QCameraViewfinder(this);

            camera->setViewfinder(viewfinder);
            ui.cam_layout->addWidget(viewfinder);
            //
            //TODO: Camera Connection Error Check
            //

            is_camera_connected = true;
            ui.camera_connect_btn->setText("Disconnect");

            camera->start();
        }
    }
}

void MainWindow::on_capture_btn_clicked(){
    if(is_camera_connected){
        imageCapture = new QCameraImageCapture(camera);
        camera->setCaptureMode(QCamera::CaptureStillImage);
        camera->searchAndLock();//Lock settings
        QString path = QFileDialog::getExistingDirectoryUrl().path();
        qDebug() << path;
        imageCapture->capture(path);
        camera->unlock();
    }
}

void MainWindow::on_record_btn_clicked(){
    if(is_camera_connected){
        if(!is_video_recording){
            recorder = new QMediaRecorder(camera);
            camera->setCaptureMode(QCamera::CaptureVideo);
            connect(recorder,SIGNAL(durationChanged(qint64)),this,SLOT(update_video_time(qint64)));
            QString path = QFileDialog::getExistingDirectoryUrl().path();
            recorder->setOutputLocation(path);
            recorder->record();
            is_video_recording = true;
            ui.record_btn->setText("Stop Record");

        }else{
            recorder->stop();
            recorder->deleteLater();
            is_video_recording = false;
            ui.record_btn->setText("Start Record");
        }
    }
}

void MainWindow::update_video_time(qint64 t){
    QString time_str = QString::number((t-t%100)/1000.0);
    ui.record_time_label->setText("Recorded: " + time_str + "s");
}

}  // namespace test_qt

