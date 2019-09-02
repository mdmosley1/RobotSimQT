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
    std::cout << "keypress!" << "\n";
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


