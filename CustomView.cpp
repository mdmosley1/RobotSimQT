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
    static double loopDt_ms = 1/LOOP_RATE*1000;
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
            graphicsTimer_->start(loopDt_ms);
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
    if ( event->key()==Qt::Key_Right &&
         graphicsTimer_->isActive())
    {
        std::cout << "Speeding up!" << "\n";
        graphicsTimer_->setSingleShot(false);
        loopDt_ms *= 0.75;
        graphicsTimer_->start(loopDt_ms);
    }

    if ( event->key()==Qt::Key_Left &&
         graphicsTimer_->isActive())
    {
        std::cout << "Slowing down!" << "\n";
        graphicsTimer_->setSingleShot(false);
        loopDt_ms *= 1.50;
        graphicsTimer_->start(loopDt_ms);
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

    // Increase/Decrease measurement noise
    if ( event->key()==Qt::Key_Up)
    {
        std::cout << "Increasing measurement noise!" << "\n";
        robot_->GPS_.IncreaseNoise();
    }
    if ( event->key()==Qt::Key_Down)
    {
        std::cout << "Decreasing measurement noise!" << "\n";
        robot_->GPS_.DecreaseNoise();
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
