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

void Robot::AddGoalToCompleted(Waypoint* _goal)
{
    goalsCompleted_.push(_goal);
    while (goalsCompleted_.size() > 5)
    {
        auto goal = goalsCompleted_.front();
        goalsCompleted_.pop();
        scene()->removeItem(goal);
    }
}

Velocity Robot::GenerateRobotControl(State _state, Waypoint* _goal)
{
    double kp = 20;
    double ka = 5;
    double kb = 0;

    double deltaX = _goal->x() - _state.x;
    double deltaY = _goal->y() - _state.y;

    double rho = std::sqrt(deltaX*deltaX + deltaY*deltaY); // distance btwne goal and state
    double alpha = -_state.theta + std::atan2(deltaY, deltaX); // How much robot needs to turn in order to face waypoint

    // if we are near current waypoint, then retrieve the next waypoint if there is one
    if (rho < 15)
    {
        // change color of the waypoint
        _goal->SetColor(Qt::green);
        AddGoalToCompleted(_goal);

        goals_.pop();
        if (goals_.empty())
            return Velocity(0,0);
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
    Velocity velocity(0,0);
    if (!goals_.empty())
        velocity = GenerateRobotControl(state, goals_.front());
    //Velocity velocity(0,0);
    UpdatePosition(velocity);

    // create an item to put into the scene so i can see where the position is represented
    QGraphicsRectItem* trailPoint = new QGraphicsRectItem();
    QPointF point = mapToScene(0, 30);
    trailPoint->setRect(point.x(), point.y(), 2, 2);
    trailPoint->setBrush(Qt::green);
    //add the item to the scene
    scene()->addItem(trailPoint);
    trailPoints_.push(trailPoint);
    const size_t NUM_POINTS_TO_KEEP = 100;
    while (trailPoints_.size() > NUM_POINTS_TO_KEEP)
    {
        auto point = trailPoints_.front();
        trailPoints_.pop();
        scene()->removeItem(point);
    }
}

void Robot::GetMeasurement()
{
    std::cout << "A sensor measurment was made!" << "\n";
}


void Robot::AddWaypoint(Waypoint* _waypt)
{
    std::cout << "Adding waypoint at " << _waypt->x() << ", " << _waypt->y() << "\n";    
    goals_.push(_waypt);
    scene()->addItem(_waypt);
}

void Robot::UpdatePosition(Velocity _vel)
{
    double dt = 1/LOOP_RATE; // time step (seconds)
    double xn = x() + dt*std::cos(theta_)*_vel.linear;
    double yn = y() + dt*std::sin(theta_)*_vel.linear;
    theta_ += dt*_vel.angular;

    if (theta_ > M_PI)
        theta_ -= 2*M_PI;
    if (theta_ < -M_PI)
        theta_ += 2*M_PI;

    // std::cout << "Rotation = " << rotation() << "\n";
    // std::cout << "PositionX = " << x() << "\n";
    // std::cout << "PositionY= " << y() << "\n";
    setPos(xn, yn);
    setRotation(theta_*180/M_PI + 90);
    emit PositionChanged(xn, yn);
    emit UpdateVelocity(_vel.linear);
}

// the python measuremnent code for april tags
/*
def get_measurements(self):
        """
        Returns a list of lists of visible landmarks and a fresh boolean that
        checks if it the measurement is ready
        Outputs:
        measurements - a N by 5 list of visible tags or None. The tags are in
            the form in the form (x,y,theta,id,time) with x,y being the 2D
            position of the marker relative to the robot, theta being the
            relative orientation of the marker with respect to the robot, id
            being the identifier from the map, and time being the current time
            stamp. If no tags are seen, the function returns None.
        """
        curr_time = self.__frame_num*self.__dt
        if curr_time - self.last_meas_time < self.__MEAS_RATE or len(self.markers_flipped) == 0:
            return None
        self.last_meas_time = curr_time
        self.__visible_markers = [False for i in range(len(self.markers_flipped))]

        H_WR = self.__H(self.__x_gt)
        # Get measurements to the robot frame
        meas = []

        for i in range(len(self.markers_flipped)):
            H_WT = self.__H(self.markers[i])
            H_RT = np.linalg.solve(H_WR,H_WT)
            x_new = H_RT[0:2,2]
            theta_new = math.atan2(H_RT[1,0], H_RT[0,0])
            marker_view_angle = \
                np.absolute(np.arccos(x_new[0]/(math.sqrt(x_new[0]**2 + x_new[1]**2))))
            if abs(marker_view_angle) < self.__view_half_angle and abs(theta_new) < np.pi/3 and x_new[0] < 2:
                self.__visible_markers[i] = True
                meas_i = np.array([x_new[0],x_new[1], theta_new, self.markers_flipped[i][3], self.last_meas_time])
                meas_i[0:3] = meas_i[0:3] + np.array([np.random.normal(0, self.__image_noise)])
                meas.append(meas_i.tolist())
        if not meas:
            meas = None
        return meas
*/
