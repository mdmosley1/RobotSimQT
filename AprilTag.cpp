#include "AprilTag.h"
#include <QPainter>
#include <iostream>
using namespace std;


void AprilTag::FadeToRed()
{
    int r,g,b;
    color_.getRgb(&r,&g,&b);
    int changeRate = 32;
    r = std::min(r+changeRate, 255);
    g = std::max(g-changeRate, 0);
    SetColor(QColor(r,g,b));
}

void AprilTag::advance(int step)
{
    if (!step)
        return;
    FadeToRed();
}

AprilTag::AprilTag(int x, int y, double theta)
{
    setPos(x,y);
    setRotation(theta);
}

QRectF AprilTag::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust,
                  36 + adjust, 60 + adjust);
}

void AprilTag::paint(QPainter *painter,
                     const QStyleOptionGraphicsItem *option,
                     QWidget *widget)
{
    painter->setBrush(color_);
    double width1 = 10;
    double height1 = 40;
    painter->drawRect(-width1, -height1/2, width1, height1);
    // draw small rect to indicate which direction the tag is facing
    double width2 = 5;
    double height2 = 10;
    painter->drawRect(0, -height2/2, width2, height2);
}

void AprilTag::SetColor(QColor _color)
{
    color_ = _color;    
}
