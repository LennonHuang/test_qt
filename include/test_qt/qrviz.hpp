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
    void display_grid();
    void display_scan();

private:
    rviz::RenderPanel *_render_panel;
    rviz::VisualizationManager *_manger;
    rviz::Display *_grid = nullptr;
    rviz::Display *_scan_display = nullptr;

};

#endif // QRVIZ_HPP
