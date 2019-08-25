#include <QtWidgets>
#include <math.h>
#include "mouse.h"
#include "Robot.h"
#include <iostream>
#include "constants.hh"
#include "CustomView.h"
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

    // for (int i = 0; i < MouseCount; ++i)
    // {
    //     Mouse *mouse = new Mouse;
    //     mouse->setPos(::sin((i * 6.28) / MouseCount) * 200,
    //                   ::cos((i * 6.28) / MouseCount) * 200);
    //     scene.addItem(mouse);
    // }

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

    std::cout << "starting program" << "\n";

    return app.exec();
}