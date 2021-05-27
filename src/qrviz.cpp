#include "../include/test_qt/qrviz.hpp"
#include <QDebug>

//Constructor:
//1. Add the rviz widget to the vertical layoutbox
//2. Initialize rviz manager.
qrviz::qrviz(QVBoxLayout *layout)
{
    _render_panel = new rviz::RenderPanel();
    layout->addWidget(_render_panel);
    _manger = new rviz::VisualizationManager(_render_panel);
    ROS_ASSERT(_manger != NULL);
    _render_panel->initialize(_manger->getSceneManager(),_manger);
    //initialize RVIZ
    _manger->initialize();
    _manger->removeAllDisplays();
    _manger->startUpdate();
}


//Display the RVIZ Grid
void qrviz::display_grid(bool enable, QColor color, int cell_num, double cell_size){
    if (_grid != nullptr){
        delete _grid;
        _grid = nullptr;
    }
    _grid = _manger->createDisplay("rviz/Grid","my_grid",enable);
    _grid->subProp("Color")->setValue(color);
    _grid->subProp("Plane Cell Count")->setValue(cell_num);
    _grid->subProp("Cell Size")->setValue(cell_size);
    ROS_ASSERT(_grid != NULL);
}

//Display tf
void qrviz::display_tf(bool enable){
    if (_tf_display != nullptr){
        delete _tf_display;
        _tf_display = nullptr;
    }
    _tf_display = _manger->createDisplay("rviz/TF","my_tf",enable);
    //TODO:tf property
}

//Display GPS
void qrviz::display_gps(bool enable){
    //Check both the map and frame display
    if (_gps_map_display != nullptr){
        delete _gps_map_display;
        _gps_map_display = nullptr;
    }
    if (_gps_frame_display != nullptr){
        delete _gps_frame_display;
        _gps_frame_display = nullptr;
    }
    //Map display in rviz
    _gps_map_display = _manger->createDisplay("rviz_plugins/AerialMapDisplay","my_map_view",enable);
    _gps_map_display->subProp("Topic")->setValue("/fix");
    _gps_map_display->subProp("Object URI")->setValue("https://tile.openstreetmap.org/{z}/{x}/{y}.png");
    //GPS frame display in rviz
    _gps_frame_display = _manger->createDisplay("rviz/Axes","gps_frame",enable);
    _gps_frame_display->subProp("Reference Frame")->setValue("gps");
    _gps_frame_display->subProp("Length")->setValue(50);
    _gps_frame_display->subProp("Radius")->setValue(1);
}

//Display the RVIZ laser point cloud
void qrviz::display_scan(bool enable,QString topic_name, double point_size){

    if(_scan_display != nullptr){
        delete _scan_display;
        _scan_display = nullptr;
    }
    _scan_display = _manger->createDisplay("rviz/LaserScan","my_laserScan",enable);
    _scan_display->subProp("Topic")->setValue(topic_name);
    _scan_display->subProp("Size (m)")->setValue(point_size);
    ROS_ASSERT(_scan_display != NULL);
}

//Display the RVIZ camera elements
void qrviz::display_camera(bool enable, QString topic_name){
    if(_camera_display != nullptr){
        delete _camera_display;
        _camera_display = nullptr;
    }
    _camera_display = _manger->createDisplay("rviz/Camera","my_camera",enable);
    _camera_display->subProp("Image Topic")->setValue(topic_name);
    ROS_ASSERT(_camera_display != NULL);
}

//Display IP camera (rviz image)
void qrviz::display_ip_camera(bool enable){
    if(_ip_camera_display != nullptr){
        delete _ip_camera_display;
        _ip_camera_display = nullptr;
    }

    _ip_camera_display = _manger->createDisplay("rviz/Image","my_ip_camera",enable);
    _ip_camera_display->subProp("Image Topic")->setValue("/image_raw");
    _ip_camera_display->subProp("Transport Hint")->setValue("compressed");
    ROS_ASSERT(_ip_camera_display != NULL);
}
