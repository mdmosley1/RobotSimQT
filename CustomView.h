#ifndef CUSTOMVIEW_H
#define CUSTOMVIEW_H

#include <QGraphicsView>
#include <QWidget>
#include <QGraphicsScene>
#include "Robot.h"

#include "qcustomplot.h"


class CustomView: public QGraphicsView
{
public:
    CustomView(QGraphicsScene* scene);
    Robot * robot;
    void mousePressEvent(QMouseEvent* event) override;
    QCustomPlot* customPlot;

public slots:
        void realtimeDataSlot();
};

#endif // CUSTOMVIEW_H
