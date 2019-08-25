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
    CustomView(Robot* _robot);
    void mousePressEvent(QMouseEvent* event) override;
    void keyPressEvent(QKeyEvent* event) override;

    Robot * robot_;

public slots:
        void realtimeDataSlot();
};

#endif // CUSTOMVIEW_H
