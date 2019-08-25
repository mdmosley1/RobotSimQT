#ifndef ROBOT_H
#define ROBOT_H

//#include <QGraphicsPixmapItem>
#include <QObject>
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

// using QGraphicsObject instead of qGraphicsItem so that it an emit signals
class Robot : public QGraphicsObject
{
    Q_OBJECT
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
signals:
    void PositionChanged(double xn, double yn);
    void UpdateVelocity(double);
public slots:
    void GetMeasurement();
    void advance(int step) override;
private:
    void AddGoalToCompleted(Waypoint*);
    void UpdatePosition(Velocity _vel); 
    State GetRobotState();
    void IncreaseLinearVelocity();
    void DecreaseLinearVelocity();
    void IncreaseAngularVelocity();
    void DecreaseAngularVelocity();
    void Brake();
    Velocity GenerateRobotControl(State _state, Waypoint* _goal);
    
    double theta_ = 0;

    const double linearVelMax_ = 100;
    const double linearVelMin_ = 0;

    const double angularVelMax_ = 0.5;
    const double angularVelMin_ = -0.5;

    const double linearDelta_ = 1;
    const double angularDelta_ = 0.1;

    Velocity vel_ = Velocity(0.0, 0.0);
    State state_;
    QColor color;
    qreal mouseEyeDirection;

    std::queue<Waypoint*> goals_;
    std::queue<Waypoint*> goalsCompleted_;
    std::queue<QGraphicsRectItem*> trailPoints_;
};

#endif // ROBOT_H
