#include <QtWidgets>
#include <math.h>
#include "mouse.h"
#include "Robot.h"
#include <iostream>
#include "constants.hh"
#include "CustomView.h"

#include <QMainWindow>
#include "qcustomplot.h"
#include "AprilTag.h"

#include "mainwindow.h"
#include <QGraphicsPolygonItem>
#include "EstimatedPose.h"


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
    robot->AddMembersToScene();
    
    robot->setPos(X_BOUND_MAX/2, Y_BOUND_MAX/2);
    robot->setRotation(0);

    AprilTag* tag1 = new AprilTag(100,100,0);
    AprilTag* tag2 = new AprilTag(900,100,90);
    AprilTag* tag3 = new AprilTag(900,900,180);
    AprilTag* tag4 = new AprilTag(100,900,270);
    scene.addItem(tag1);
    scene.addItem(tag2);
    scene.addItem(tag3);
    scene.addItem(tag4);
    robot->AddAprilTag(tag1);
    robot->AddAprilTag(tag2);
    robot->AddAprilTag(tag3);
    robot->AddAprilTag(tag4);

    // {
    // QPolygonF wall;
    // wall << QPointF(0,0)
    //      << QPointF(X_BOUND_MAX, 0)
    //      << QPointF(X_BOUND_MAX, 50)
    //      << QPointF(0, 50);
    // QGraphicsPolygonItem* wallG = new QGraphicsPolygonItem(wall);
    // wallG->setBrush(QColor(255,0,0,127));
    // scene.addItem(wallG);
    // }

    // add mice
    if (false)
    {
        for (int i = 0; i < MouseCount; ++i)
        {
            Mouse *mouse = new Mouse;
            mouse->setPos(std::rand() % int(X_BOUND_MAX),
                          std::rand() % int(Y_BOUND_MAX));
            std::cout << "Mouse at position = " << mouse->pos().x()<< "\n";
            scene.addItem(mouse);
        }
    }

    view.setRenderHint(QPainter::Antialiasing);
    view.setBackgroundBrush(QPixmap("images/cheese.jpg"));

    view.setCacheMode(QGraphicsView::CacheBackground);
    view.setViewportUpdateMode(QGraphicsView::BoundingRectViewportUpdate);

    view.setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view.setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    view.setFixedSize(X_BOUND_MAX, Y_BOUND_MAX);

    view.setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
    view.show();

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, &scene, &QGraphicsScene::advance);
    timer.start(1/LOOP_RATE*1000);

    QTimer timerSensor;
    QObject::connect(&timerSensor, &QTimer::timeout,
                       robot, &Robot::GetMeasurementAprilTag);
    timerSensor.start(1000); // once per second

    std::cout << "starting program" << "\n";

    MainWindow w(robot);
    w.show();
    
    return app.exec();
} 
