#include "Waypoint.h"
#include <QPainter>

Waypoint::Waypoint(int x, int y)
{
    setPos(x,y);
}

QRectF Waypoint::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust,
                  36 + adjust, 60 + adjust);
}

void Waypoint::paint(QPainter *painter,
                     const QStyleOptionGraphicsItem *option,
                     QWidget *widget)
{    
    painter->setBrush(color_);
    painter->drawEllipse(-10, -10, 10, 10);
}

void Waypoint::SetColor(QColor _color)
{
    color_ = _color;
}
