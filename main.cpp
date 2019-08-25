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
    CustomView view(&scene);
    
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

    MainWindow w;
    w.show();

    std::cout << "starting program" << "\n";
    return app.exec();
}    

    // test out the custom plot
    // QMainWindow window;
// // setup customPlot as central widget of window:
//     QCustomPlot customPlot;
//     window.setCentralWidget(&customPlot);
  
// // create plot (from quadratic plot example):
//     QVector<double> x(101), y(101);
//     for (int i=0; i<101; ++i)
//     {
//         x[i] = i/50.0 - 1;
//         y[i] = x[i]*x[i];
//     }
//     customPlot.addGraph();
//     customPlot.graph(0)->setData(x, y);
//     customPlot.xAxis->setLabel("x");
//     customPlot.yAxis->setLabel("y");
//     customPlot.rescaleAxes();
  
//     window.setGeometry(100, 100, 500, 400);
//     window.show();


   


// int main(int argc, char **argv)
// {
//     QApplication app(argc, argv);

//     QMainWindow window;
  
// // setup customPlot as central widget of window:
//     QCustomPlot customPlot;
//     window.setCentralWidget(&customPlot);
  
// // create plot (from quadratic plot example):
//     QVector<double> x(101), y(101);
//     for (int i=0; i<101; ++i)
//     {
//         x[i] = i/50.0 - 1;
//         y[i] = x[i]*x[i];
//     }
//     customPlot.addGraph();
//     customPlot.graph(0)->setData(x, y);
//     customPlot.xAxis->setLabel("x");
//     customPlot.yAxis->setLabel("y");
//     customPlot.rescaleAxes();
  
//     window.setGeometry(100, 100, 500, 400);
//     window.show();

//     std::cout << "starting program" << "\n";

//     return app.exec();
// }


