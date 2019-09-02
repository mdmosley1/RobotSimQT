#include "EstimatedPose.h"
#include <QPainter>

EstimatedPose::EstimatedPose(int _x, int _y, double _theta)
{
    setPos(_x, _y);
    setRotation(_theta);
}

QRectF EstimatedPose::boundingRect() const
{
    int x = 1000;
    return QRectF(-x/2, -x/2, x, x);
}

void EstimatedPose::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // Body
    painter->setBrush(QColor(255,255,255,30));
    painter->setPen(Qt::black);
    int width = 20;
    int length = 40;
    painter->drawRect(-length/2, -width/2, length, width);

    // draw front trapezoid
    static const QPointF points[4] = {
        QPointF(20, -10),
        QPointF(30, -5),
        QPointF(30, +5),
        QPointF(20, 10),
    };
    painter->drawPolygon(points,4);

    painter->setBrush(Qt::black);
    // draw wheels
    int wheelLength = 25;
    int wheelWidth = 5;
    painter->drawRect(-15, -15, wheelLength, wheelWidth);
    painter->drawRect(-15, +10, wheelLength, wheelWidth);
}
