#ifndef ESTIMATEDPOSE_H
#define ESTIMATEDPOSE_H

#include <QGraphicsItem>

class EstimatedPose : public QGraphicsItem
{
public:
    EstimatedPose(int _x, int _y, double _theta);
    QRectF boundingRect() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;
};



#endif /* ESTIMATEDPOSE_H */
