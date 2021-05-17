#include "../include/test_qt/qrviz.hpp"
#include <QDebug>

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

void qrviz::display_grid(){
    bool enable = true;
    int cell_count = 16;
    if (_grid != nullptr){
        delete _grid;
        _grid = nullptr;
    }
    _grid = _manger->createDisplay("rviz/Grid","my_grid",enable);
    _grid->subProp("Plane Cell Count")->setValue(cell_count);
    ROS_ASSERT(_grid != NULL);
}

void qrviz::display_scan(){
    bool enable = true;
    if(_scan_display != nullptr){
        delete _scan_display;
        _scan_display = nullptr;
    }
    _manger->setFixedFrame("laser_link");
    _scan_display = _manger->createDisplay("rviz/LaserScan","my_laserScan",enable);
    _scan_display->subProp("Topic")->setValue("/scan");
    ROS_ASSERT(_scan_display != NULL);
}
