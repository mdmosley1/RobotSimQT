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

    // Speed up or slow down the simulation
    if ( event->key()==Qt::Key_Up &&
         graphicsTimer_->isActive())
    {
        std::cout << "Speeding up!" << "\n";
        graphicsTimer_->setSingleShot(false);
        int dt = graphicsTimer_->interval();
        graphicsTimer_->start(dt*0.75);
    }

    if ( event->key()==Qt::Key_Down &&
         graphicsTimer_->isActive())
    {
        std::cout << "Slowing down!" << "\n";
        graphicsTimer_->setSingleShot(false);
        int dt = graphicsTimer_->interval();
        graphicsTimer_->start(dt*1.50);
    }

    // Quit
    if ( event->key()==Qt::Key_Q)
    {
        std::cout << "User pressed Q so we are qutting!" << "\n";
        exit(0);
    }

    // Reset Kalman Filter
    if ( event->key()==Qt::Key_R)
    {
        std::cout << "Reset kalman filter!" << "\n";
        robot_->ResetKalmanFilter();
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
