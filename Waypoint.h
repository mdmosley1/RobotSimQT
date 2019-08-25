#ifndef WAYPOINT_H
#define WAYPOINT_H

#include <QGraphicsItem>

//! [0]
class Waypoint : public QGraphicsItem
{
public:
    Waypoint(int, int);
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;
    void SetColor(QColor _color);

private:
    QColor color_ = QColor(Qt::red);
};

#endif /* WAYPOINT_H */
