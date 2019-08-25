#ifndef CUSTOMVIEW_H
#define CUSTOMVIEW_H

#include <QGraphicsView>
#include <QWidget>
#include <QGraphicsScene>
#include "Robot.h"


class CustomView: public QGraphicsView
{
public:
    CustomView(QGraphicsScene* scene);
    Robot * robot;

    void mousePressEvent(QMouseEvent* event) override;
};

#endif // CUSTOMVIEW_H
