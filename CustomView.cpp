#include "CustomView.h"
#include "constants.hh"
#include <iostream>
#include <QtWidgets>
#include <math.h>
#include "mouse.h"
#include "Robot.h"
#include "Waypoint.h"

#include <QMainWindow>
#include "qcustomplot.h"
#include "mainwindow.h"


CustomView::CustomView(Robot* _robot): robot_(_robot)
{

}

void CustomView::keyPressEvent(QKeyEvent *event)
{
    if ((event->key()==Qt::Key_Space))
    {
        if (graphicsTimer_->isActive())
        {
            std::cout << "Pause!" << "\n";
            graphicsTimer_->stop();
        }
        else
        {
            std::cout << "Resume!" << "\n";
            graphicsTimer_->start();
        }
    }
}


void CustomView::mousePressEvent(QMouseEvent *event)
{
    int x = event->x();
    int y = event->y();
    std::cout << "Mouse event triggerd at " << x << ", " << y << "\n";
    
    Waypoint* nextWaypoint = new Waypoint(x,y);
    robot_->AddWaypoint(nextWaypoint);
    QGraphicsView::mousePressEvent(event);
}
