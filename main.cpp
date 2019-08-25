#include <QtWidgets>
#include <math.h>
#include "mouse.h"
#include "Robot.h"
#include <iostream>
#include "constants.hh"
#include "CustomView.h"

#include <QMainWindow>
#include "qcustomplot.h"

#include "mainwindow.h"


static const int MouseCount = 7;

const double X_BOUND_MIN = 0;
const double Y_BOUND_MIN = 0;

const double X_BOUND_MAX = 1000;
const double Y_BOUND_MAX = 1000;

int main(int argc, char **argv)
{
    QApplication app(argc, argv);
    QGraphicsScene scene;
    scene.setSceneRect(X_BOUND_MIN, Y_BOUND_MIN, X_BOUND_MAX, Y_BOUND_MAX);

    scene.setItemIndexMethod(QGraphicsScene::NoIndex);

    //QGraphicsView view(&scene);
    Robot* robot = new Robot();
    CustomView view(robot);
    view.setScene(&scene);
    scene.addItem(robot);
    robot->setPos(0,0);

    // add mice
    for (int i = 0; i < MouseCount; ++i)
    {
        Mouse *mouse = new Mouse;
        mouse->setPos(std::rand() % int(X_BOUND_MAX),
                      std::rand() % int(Y_BOUND_MAX));
        std::cout << "Mouse at position = " << mouse->pos().x()<< "\n";
        scene.addItem(mouse);
    }

    Waypoint* waypoint = new Waypoint(X_BOUND_MAX/2, Y_BOUND_MAX/2);
    robot->AddWaypoint(waypoint);
    
    view.setRenderHint(QPainter::Antialiasing);
    view.setBackgroundBrush(QPixmap("images/cheese.jpg"));

    view.setCacheMode(QGraphicsView::CacheBackground);
    view.setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);
    view.setDragMode(QGraphicsView::ScrollHandDrag);

    view.setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
    view.resize(X_BOUND_MAX, Y_BOUND_MAX);
    view.show();

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, &scene, &QGraphicsScene::advance);
    timer.start(1/LOOP_RATE*1000);

    // TODO create another timer here that represents the rate at
    // which the sensor measurement is made. when timer expires, call Robot::GetMeasurement like this:
    //QTimer timerSensor;
     // QObject::connect(&timerSensor, &QTimer::timeout,
     //                  robot, &Robot::GetMeasurment);     
//     timerSensor.start(1000); // once per second

    std::cout << "starting program" << "\n";

    MainWindow w(robot);
    w.show();
    
    return app.exec();
} 
