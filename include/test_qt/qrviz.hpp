#ifndef QRVIZ_HPP
#define QRVIZ_HPP


#include <QtGui>
#include <ros/ros.h>
#include <rviz/render_panel.h>
#include <rviz/visualization_manager.h>
#include <rviz/display.h>
#include <rviz/tool_manager.h>
#include <rviz/tool.h>
#include <QVBoxLayout>

class qrviz
{
public:
    qrviz(QVBoxLayout *layout);
    void display_grid(bool enable,QColor color, int cell_num,double cell_size);
    void display_scan(bool enable,QString topic_name, double point_size);
    void display_camera(bool enable, QString topic_name);
    void display_tf(bool enable);
    void display_gps(bool enable);
    void display_ip_camera(bool enable);
    rviz::VisualizationManager *_manger;

private:
    rviz::RenderPanel *_render_panel;
    rviz::Display *_grid = nullptr;
    rviz::Display *_scan_display = nullptr;
    rviz::Display *_camera_display = nullptr;
    rviz::Display *_tf_display = nullptr;
    rviz::Display *_gps_frame_display = nullptr;
    rviz::Display *_gps_map_display = nullptr;
    rviz::Display *_ip_camera_display = nullptr;

};

#endif // QRVIZ_HPP
