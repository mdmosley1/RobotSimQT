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


static const int MouseCount = 7;

const double X_BOUND_MIN = 0;
const double Y_BOUND_MIN = 0;

const double X_BOUND_MAX = 1000;
const double Y_BOUND_MAX = 1000;

CustomView::CustomView(QGraphicsScene* scene)
{
    setScene(scene);

    robot = new Robot();
    scene->addItem(robot);
    robot->setPos(0,0);

    // add mice
    for (int i = 0; i < MouseCount; ++i)
    {
        Mouse *mouse = new Mouse;
        mouse->setPos(::sin((i * 6.28) / MouseCount) * 200,
                      ::cos((i * 6.28) / MouseCount) * 200);
        scene->addItem(mouse);
    }

    Waypoint* waypoint = new Waypoint(X_BOUND_MAX/2, Y_BOUND_MAX/2);
    robot->AddWaypoint(waypoint);
}

void CustomView::mousePressEvent(QMouseEvent *event)
{
    int x = event->x();
    int y = event->y();
    std::cout << "Mouse event triggerd at " << x << ", " << y << "\n";
    
    Waypoint* nextWaypoint = new Waypoint(x,y);
    robot->AddWaypoint(nextWaypoint);
    QGraphicsView::mousePressEvent(event);
}


