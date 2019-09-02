#ifndef APRILTAG_H
#define APRILTAG_H

#include <QGraphicsItem>


class AprilTag : public QGraphicsItem
{
public:
    AprilTag(int _x, int _y, double _theta);
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;
    void SetColor(QColor _color);
private:
    QColor color_ = QColor(Qt::red);
    
};

#endif /* APRILTAG_H */
