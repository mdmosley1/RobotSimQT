#include "CustomView.h"
#include "constants.hh"
#include <iostream>
#include <QtWidgets>
#include <math.h>
#include "mouse.h"
#include "Robot.h"
#include "Waypoint.h"


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

//CustomView::CustomView()
//{
    // QGraphicsScene scene;
    // scene.setSceneRect(X_BOUND_MIN, Y_BOUND_MIN, X_BOUND_MAX, Y_BOUND_MAX);

    // scene.setItemIndexMethod(QGraphicsScene::NoIndex);

    // for (int i = 0; i < MouseCount; ++i)
    // {
    //     Mouse *mouse = new Mouse;
    //     mouse->setPos(::sin((i * 6.28) / MouseCount) * 200,
    //                   ::cos((i * 6.28) / MouseCount) * 200);
    //     scene.addItem(mouse);
    // }

    // // Add robot to scene
    // robot = new Robot();
    // robot->setPos(0,0);
    // robot->SetGoal(X_BOUND_MAX/2, Y_BOUND_MAX/2);
    // //robot->SetGoal(0,0);
    // scene.addItem(robot);

    // QGraphicsView view(&scene);
    // view.setRenderHint(QPainter::Antialiasing);
    // view.setBackgroundBrush(QPixmap("images/cheese.jpg"));
    // view.setCacheMode(QGraphicsView::CacheBackground);
    // view.setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    // view.setDragMode(QGraphicsView::ScrollHandDrag);

    // view.setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
    // view.resize(X_BOUND_MAX, Y_BOUND_MAX);
    // view.show();

    // QTimer timer;
    // QObject::connect(&timer, &QTimer::timeout, &scene, &QGraphicsScene::advance);
    // timer.start(1/LOOP_RATE*1000);

    // std::cout << "starting program" << "\n";
    // show();
//}

// void CustomView::mousePressEvent(QMouseEvent *event)
// {
//     std::cout << "Game Mouse event triggerd at " << event->x() << ", " << event->y() << "\n";
//     robot->SetGoal(event->x(),event->y());

//     QGraphicsView::mousePressEvent(event);
// }


