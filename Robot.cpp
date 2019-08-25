#include "Robot.h"
#include <QGraphicsScene>
#include <QPainter>
#include <cmath>
#include <iostream>
#include "qpainter.h"
#include "constants.hh"
#include "Waypoint.h"


Robot::Robot(): mouseEyeDirection(0), color(std::rand() % 256, std::rand() % 256, std::rand() % 256)
{
}

QRectF Robot::boundingRect() const
{
    qreal adjust = 0.5;
    return QRectF(-18 - adjust, -22 - adjust,
                  36 + adjust, 60 + adjust);
}

QPainterPath Robot::shape() const
{
    QPainterPath path;
    path.addRect(-10, -20, 20, 40);
    return path;
}

void Robot::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // Body
    painter->setBrush(color);
    painter->drawEllipse(-10, -20, 20, 40);

    // Eyes
    painter->setBrush(Qt::white);
    painter->drawEllipse(-10, -17, 8, 8);
    painter->drawEllipse(2, -17, 8, 8);

    // Nose
    painter->setBrush(Qt::black);
    painter->drawEllipse(QRectF(-2, -22, 4, 4));

    // Pupils
    painter->drawEllipse(QRectF(-8.0 + mouseEyeDirection, -17, 4, 4));
    painter->drawEllipse(QRectF(4.0 + mouseEyeDirection, -17, 4, 4));

    // Ears
    painter->setBrush(scene()->collidingItems(this).isEmpty() ? Qt::darkYellow : Qt::red);
    painter->drawEllipse(-17, -12, 16, 16);
    painter->drawEllipse(1, -12, 16, 16);

    // Tail
    QPainterPath path(QPointF(0, 20));
    path.cubicTo(-5, 22, -5, 22, 0, 25);
    path.cubicTo(5, 27, 5, 32, 0, 30);
    path.cubicTo(-5, 32, -5, 42, 0, 35);
    painter->setBrush(Qt::NoBrush);
    painter->drawPath(path);
}

void Robot::IncreaseLinearVelocity()
{
    double newVel = vel_.linear + linearDelta_;
    vel_.linear = std::min(newVel, linearVelMax_);
}

void Robot::DecreaseLinearVelocity()
{
    double newVel = vel_.linear- linearDelta_;
    vel_.linear = std::max(newVel, linearVelMin_);
}

void Robot::IncreaseAngularVelocity()
{
    double newAngVel = vel_.angular + angularDelta_;
    vel_.angular = std::min(newAngVel, angularVelMax_);
}

void Robot::DecreaseAngularVelocity()
{
    double newAngVel = vel_.angular - angularDelta_;
    vel_.angular = std::max(newAngVel, angularVelMin_);
}


void Robot::Brake()
{
    vel_.linear *= 0.1;
}

// void Robot::keyPressEvent(QKeyEvent *event)
// {
//     std::cout << "Keypress event triggerd " << std::endl;
//     // move the robot left and right
//     if (event->key() == Qt::Key_Up)
//         IncreaseLinearVelocity();
//     else if (event->key() == Qt::Key_Down)
//         DecreaseLinearVelocity();
//     else if (event->key() == Qt::Key_Right)
//         DecreaseAngularVelocity();
//     else if (event->key() == Qt::Key_Left)
//         IncreaseAngularVelocity();
//     else if (event->key() == Qt::Key_Space)
//         Brake();            
// }

Velocity Robot::GenerateRobotControl(State _state, Point2f _goal)
{
    double kp = 20;
    double ka = 5;
    double kb = 0;

    double deltaX = _goal.x - _state.x;
    double deltaY = _goal.y - _state.y;

    double rho = std::sqrt(deltaX*deltaX + deltaY*deltaY); // distance btwne goal and state
    double alpha = -_state.theta + std::atan2(deltaY, deltaX); // How much robot needs to turn in order to face waypoint

    // if we collide with current waypoint, then retrieve the next waypoint if there is one
    if (rho < 15)
    {
        if (goals_.empty())
            return Velocity(0,0);
        else 
        {
            Point2f nextGoal = goals_.front();
            SetGoal(nextGoal);
            goals_.pop();
        }
    }
    

    if (alpha > M_PI)
        alpha -= 2 * M_PI;
    else if (alpha < -M_PI)
        alpha += 2 * M_PI;

    double beta = -_state.theta - alpha; // desired theta when robot reaches goal (I think)
    double angularVelocity = ka * alpha + kb * beta;
    double linearVelocity = kp * rho - std::fabs(alpha)*20.0; // if alpha is high, then need to reduce the linear velocity

    double minVelocity = 0.0;
    double maxVelocity = 150.0;
    linearVelocity = std::min(std::max(linearVelocity, minVelocity), maxVelocity);

    double minAngVel = -10;
    double maxAngVel = 10;
    angularVelocity = std::min(std::max(angularVelocity, minAngVel), maxAngVel);

    return Velocity(linearVelocity, angularVelocity);
}

State Robot::GetRobotState()
{
    return State(x(), y(), theta_);
}

void Robot::advance(int step)
{
    if (!step)
        return;
    // measurement = getMeasurement()
    // estimatedState = filter->EstimateState(measurement)
    State state = GetRobotState();
    Velocity velocity = GenerateRobotControl(state, goal_);
    //Velocity velocity(0,0);
    UpdatePosition(velocity);

    // create an item to put into the scene so i can see where the position is represented
    QGraphicsRectItem* trailPoint = new QGraphicsRectItem();
    QPointF point = mapToScene(0, 30);
    std::cout << "point = " << point.x() << ", " << point.y() << "\n";
    trailPoint->setRect(point.x(), point.y(), 2, 2);
    trailPoint->setBrush(Qt::green);
    //add the item to the scene
    scene()->addItem(trailPoint);
}


void Robot::AddWaypoint(Waypoint* _waypt)
{
    std::cout << "Adding waypoint at " << _waypt->x() << ", " << _waypt->y() << "\n";    
    goals_.push(Point2f(_waypt->x(),_waypt->y()));
    scene()->addItem(_waypt);
}

void Robot::SetGoal(Point2f _goal)
{
    goal_ = _goal;
}

void Robot::SetGoal(double x, double y)
{
    goal_.x = x;
    goal_.y = y;
}

void Robot::UpdatePosition(Velocity _vel)
{
    double dt = 1/LOOP_RATE; // time step (seconds)
    double xn = x() + dt*std::cos(theta_)*_vel.linear;
    double yn = y() + dt*std::sin(theta_)*_vel.linear;
    theta_ += dt*_vel.angular;

    // std::cout << "Rotation = " << rotation() << "\n";
    // std::cout << "PositionX = " << x() << "\n";
    // std::cout << "PositionY= " << y() << "\n";
    setPos(xn, yn);
    setRotation(theta_*180/M_PI + 90);
}
