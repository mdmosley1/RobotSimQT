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
    // toggle pause state
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
            graphicsTimer_->setSingleShot(false);
            graphicsTimer_->start(1/LOOP_RATE*1000);
        }
    }

    // Step thru one iteration at a time while paused
    if ( event->key()==Qt::Key_S)
    {
        std::cout << "Step!" << "\n";
        graphicsTimer_->setSingleShot(true);
        graphicsTimer_->start(1);
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
