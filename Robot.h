#ifndef ROBOT_H
#define ROBOT_H

//#include <QGraphicsPixmapItem>
//#include <QObject>
#include <QGraphicsItem>
#include <queue>
#include "Waypoint.h"
//#include <QMediaPlayer>
//#include <QMouseEvent>


struct Point2f
{
    Point2f(double _x, double _y) : x(_x), y(_y) {};
    double x,y;
};

struct Velocity
{
    Velocity(double _lin, double  _ang): linear(_lin), angular(_ang) {};
    double linear, angular;
};

struct State
{
    State(double _x,double _y,double _theta): x(_x), y(_y), theta(_theta) {};
    State(): x(0), y(0), theta(0) {} ;
    double x,y,theta;
};


//class Robot : public QObject, public QGraphicsPixmapItem
class Robot : public QGraphicsItem
{
public:
    Robot();
    void SetGoal(double x, double y);
    void SetGoal(Point2f);
    //void AddWaypoint(double x, double y);
    void AddWaypoint(Waypoint*);
    QRectF boundingRect() const override;
    QPainterPath shape() const override;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
               QWidget *widget) override;
public slots:
    void advance(int step) override;
private:
    void UpdatePosition(Velocity _vel); 
    State GetRobotState();
    void IncreaseLinearVelocity();
    void DecreaseLinearVelocity();
    void IncreaseAngularVelocity();
    void DecreaseAngularVelocity();
    void Brake();
    Velocity GenerateRobotControl(State _state, Point2f _goal);
    
    double theta_ = 0;

    const double linearVelMax_ = 100;
    const double linearVelMin_ = 0;

    const double angularVelMax_ = 0.5;
    const double angularVelMin_ = -0.5;

    const double linearDelta_ = 1;
    const double angularDelta_ = 0.1;

    Point2f goal_ = Point2f(0,0);

    Velocity vel_ = Velocity(0.0, 0.0);
    State state_;
    QColor color;
    qreal mouseEyeDirection;

    std::queue<Point2f> goals_;
};

#endif // ROBOT_H