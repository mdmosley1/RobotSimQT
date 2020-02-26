//#include <QtWidgets>
// Changed from QtWidgets to QWidget because I have QT 5 on mac
#include <QWidget>

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
//#include "ConfigurationYaml.h"

static const int MouseCount = 7;

int main(int argc, char **argv)
{
    //ConfigurationYaml configYaml;
    //configYaml.Init("config.yaml");
    
    QApplication app(argc, argv);
    QGraphicsScene scene;
    int x_bound_min_ = 0;
    int x_bound_max_ = 1000;
    int y_bound_min_ = 0;
    int y_bound_max_ = 1000;
    
    scene.setSceneRect(x_bound_min_, y_bound_min_,
                       x_bound_max_, y_bound_max_);

    scene.setItemIndexMethod(QGraphicsScene::NoIndex);

    //QGraphicsView view(&scene);
    Robot* robot = new Robot();
    CustomView view(robot);
    view.setScene(&scene);
    scene.addItem(robot);
    robot->AddMembersToScene();
    
    robot->setPos(x_bound_max_/2,
                  y_bound_max_/2);
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
            mouse->setPos(std::rand() % int(x_bound_max_),
                          std::rand() % int(y_bound_max_));
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
    view.setFixedSize(x_bound_max_, y_bound_max_);

    view.setWindowTitle(QT_TRANSLATE_NOOP(QGraphicsView, "Colliding Mice"));
    view.show();

    // start a timer to display graphics at 30 frames per second
    QTimer graphicsTimer;
    QObject::connect(&graphicsTimer, &QTimer::timeout, &scene, &QGraphicsScene::advance);
    graphicsTimer.start(1/LOOP_RATE*1000);

    // QTimer timerCamera;
    // QObject::connect(&timerCamera, &QTimer::timeout,
    //                    robot, &Robot::GetMeasurementAprilTag);
    // timerCamera.start(1000); // 1 Hz

    // QTimer timerIMU;
    // QObject::connect(&timerIMU, &QTimer::timeout,
    //                  robot, &Robot::GetMeasurementIMU);
    // timerIMU.start(20); // 50 Hz

    std::cout << "starting program" << "\n";

    MainWindow mainWindow(robot);
    mainWindow.show();
    
    return app.exec();
} 
