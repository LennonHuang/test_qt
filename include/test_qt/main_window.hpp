/**
 * @file /include/test_qt/main_window.hpp
 *
 * @brief Qt based gui for test_qt.
 *
 * @date November 2010
 **/
#ifndef test_qt_MAIN_WINDOW_H
#define test_qt_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtWidgets/QMainWindow>
#include <QtGui>
#include <QSpinBox>
#include "ui_main_window.h"
#include "ui_select_dialog_window.h"
#include "qnode.hpp"
#include "worker.hpp"
#include "gps_worker.hpp"
#include "qrviz.hpp"
#include "ip_worker.hpp"
#include "laser_worker.hpp"
#include <QtSerialPort/QSerialPort>
#include <QCamera>
#include <QMediaRecorder>
#include <QCameraInfo>
#include <QCameraViewfinder>
#include <QCameraImageCapture>
#include <QUrl>
#include <QMediaPlayer>
#include <QVideoWidget>
#include <QNetworkInterface>
#include <QProcess>

/*****************************************************************************
** Namespace
*****************************************************************************/

namespace test_qt {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

	void ReadSettings(); // Load up qt program settings at startup
	void WriteSettings(); // Save qt program settings when closing
    void scan_ip();

	void closeEvent(QCloseEvent *event); // Overloaded function
	void showNoMasterMessage();
    QStringList scanPort();
    QString bag_path;


public Q_SLOTS:
	/******************************************
	** Auto-connections (connectSlotsByName())
    ** Format: on_UiElement_Signal()
	*******************************************/
    void on_clean_button_clicked();
    void on_quit_button_clicked();
	void on_actionAbout_triggered();
    void on_button_connect_clicked();
	void on_checkbox_use_environment_stateChanged(int state);
    //gps btn
    void on_gps_connect_btn_clicked();
    void on_gps_disconnect_btn_clicked();
    //imu btn
    void on_imu_connect_btn_clicked();
    void on_imu_refresh_btn_clicked();
    void on_imu_disconnect_btn_clicked();
    //Serial Scan
    void on_serial_scan_gps_btn_clicked();
    void on_serial_scan_btn_clicked();
    void on_led_btn_clicked();
    void window_update_imu(QString imu_data);
    void catch_imu_connection_error(QString error);
    //GPS Process
    void window_gps_status(QString status);
    void slot_table_display_gps(const sensor_msgs::NavSatFix);
    void output_gps_process_error();
    void output_gps_process_standard();
    //USB Camera
    void on_camera_scan_btn_clicked();
    void on_camera_connect_btn_clicked();
    void on_capture_btn_clicked();
    void on_record_btn_clicked();
    void update_video_time(qint64 t);
    //IP Camera
    void on_scan_camera_ip_btn_clicked();
    void on_connect_ip_camera_btn_clicked();
    void on_disconnect_ip_camera_btn_clicked();
    void on_camera_checkBox_stateChanged(int state);
    void on_camera_checkBox_2_stateChanged(int state);
    void on_connect_ip_camera_btn_2_clicked();
    void on_disconnect_ip_camera_btn_2_clicked();
    //Video Player
    void on_load_video_btn_clicked();
    //IP Scan
    void update_ip();
    //Lidar
    void on_scan_laser_btn_clicked();
    void on_connect_laser_btn_clicked();
    void on_disconnect_laser_btn_clicked();
    void output_sick_process();
    void output_sick_process_error();

    //RVIZ
    void init_rviz_ui_elements();
    void slot_fixed_frame_changed(QString);
    void slot_mainwindow_display_grid(int state);
    void slot_mainwindow_display_scan(int state);
    void slot_mainwindow_display_camera(int state);
    void slot_mainwindow_display_tf(int state);
    void slot_mainwindow_display_gps(int state);

    //ROS Bag
    void on_start_bag_btn_clicked();
    void on_close_bag_btn_clicked();
    void slot_start_bag();
    void on_load_bag_btn_clicked();

    //Load URDF
    void on_load_coordinate_btn_clicked();

    /******************************************
    ** Manual connections
    *******************************************/
    void updateLoggingView();

private:
    Ui::MainWindowDesign ui;//header file built from .ui file
    Ui::sensor_select_dialog select_sensor_ui;
    bool plugin_on = false;//A flag to avoid re-create a new bash (plugin)
    bool gps_on = false;
    QProcess *plg = nullptr;
	QNode qnode;
    Worker *nano_worker;//Serial Worker in non-GUI thread
    QProcess *gps_process = nullptr;
    QProcess *ip_scan_process=nullptr;
    QProcess *ip_camera_process=nullptr;
    QProcess *ip_camera_process_2 = nullptr;
    QProcess *sick_lidar_process=nullptr;//SICK Laser
    qrviz *my_rviz;

    //RVIZ ui Elements
    QCheckBox *grid_checkbox;
    QCheckBox *scan_check_box;
    QComboBox *fixed_frame_box;
    QCheckBox *tf_check_box;
    QCheckBox *gps_check_box;
    QSpinBox *grid_cell_num_box;
    QComboBox *grid_color_box;
    QComboBox *scan_topic_name_box;
    QDoubleSpinBox *grid_cell_size_box;
    QComboBox *camera_topic_name_box;
    QDoubleSpinBox *laser_point_size_box;

    //QCamera
    QCamera *camera;
    QCameraViewfinder *viewfinder;
    bool is_camera_connected = false;
    bool is_video_recording = false;
    void connectCamera();
    QCameraImageCapture *imageCapture;
    QMediaRecorder *recorder;
    QMediaPlayer *player;

    //ROS bag
    QDialog *select_dial = nullptr;
    QProcess *bag_player = nullptr;
    bool is_bag_playing = false;

    //URDF
    QProcess *urdf_process = nullptr;
    bool is_urdf_loaded = false;
};

}  // namespace test_qt

#endif // test_qt_MAIN_WINDOW_H
