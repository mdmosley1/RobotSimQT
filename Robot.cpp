#include "Robot.h"
#include <QGraphicsScene>
#include <QPainter>
#include <cmath>
#include <iostream>
#include "qpainter.h"
#include "constants.hh"
#include "Waypoint.h"
#include <random>
#include <Eigen/Dense>

const double LOOP_RATE_SIM = 30;
const double SIM_TIME_INCREMENT = 1 / LOOP_RATE_SIM;

Robot::Robot(): color_(std::rand() % 256, std::rand() % 256, std::rand() % 256)
{
    cameraFOV_ = 60 * M_PI / 180.0;
    cameraMaxRange_ = 500;
    int widthFOV = std::tan(cameraFOV_)*cameraMaxRange_;
    pointsFOV_ =
        {
            QPointF(30, 0),
            QPointF(cameraMaxRange_, -widthFOV/2),
            QPointF(cameraMaxRange_, widthFOV/2),
        };

    estimatedPose_ = new EstimatedPose(x(), y(), rotation()*M_PI/180);
    noisyPose_ = State(0,0,0);

    timeTextItem_ = new QGraphicsTextItem;
}

void Robot::AddMembersToScene()
{
    scene()->addItem(estimatedPose_);
}

QPainterPath Robot::shape() const
{
    QPainterPath path;
    double thetaFOV = 60 * M_PI / 180.0;  // 120 degrees
    int lengthFOV = 500;
    int widthFOV = std::tan(thetaFOV)*lengthFOV;
    QPolygonF triangle;
    triangle << QPointF(30, 0)
             << QPointF(lengthFOV, -widthFOV/2)
             << QPointF(lengthFOV, widthFOV/2);
    
    path.addPolygon(triangle);
    return path;
}

QRectF Robot::boundingRect() const
{
    //return QRectF(-30, -30, 100, 100);
    int x = 1000;
    return QRectF(-x/2, -x/2, x, x);
}

void Robot::paint(QPainter *painter, const QStyleOptionGraphicsItem *, QWidget *)
{
    // Body
    painter->setBrush(Qt::blue);
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

    // draw transparent triangle representing the FOV
    painter->setBrush(QColor(255,255,0,50));
    painter->drawPolygon(&pointsFOV_[0], 3);
}

double GetRandN(double mean, double var)
{
    std::random_device r;
    std::default_random_engine gen(r());
    std::normal_distribution<double> dist(mean, var);
    return dist(gen);
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
    // gains 
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

    double beta = -_state.theta - alpha; // desired theta when robot reaches goal
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
    return State(x(), y(), rotation() * M_PI/180.0);
}

// void Robot::ChangeColorOfAprilTags()
// {
//     QList<QGraphicsItem *> list = collidingItems() ;
//     foreach(QGraphicsItem * i , list)
//     {
//         AprilTag* tag = dynamic_cast<AprilTag *>(i);
//         if (tag)
//         {
//             // std::cout << "encountered tag at position: " << tag->x()
//             //           << ", " << tag->y()
//             //           << ", " << tag->rotation()
//             //           << "\n";
//             QPointF pt = mapFromScene(tag->pos());
            
//             std::cout << "Offset is " << pt.x() << ", " << pt.y() << "\n";
//             //tag->SetColor(Qt::green);

//             // add gaussian noise to offset based on the angle
            
//         }
//     }
// }

std::tuple<ImuMeasurement*, State*> Robot::GetMeasurements()
{
    State* cameraMeas = GetMeasurementAprilTag();
    ImuMeasurement* imuMeas = GetMeasurementIMU();

    return std::make_tuple(imuMeas, cameraMeas);
}

void Robot::DisplayTime()
{
    scene()->removeItem(timeTextItem_);
    timeTextItem_->setPos(150,20);
    QString timeStr; timeStr.setNum(simTime_);
    timeTextItem_->setPlainText(timeStr);
    scene()->addItem(timeTextItem_);
}

QPointF Robot::GetGPSMeasurement()
{
    //double variance = 15;
    double variance = 10;
    QPointF noise(GetRandN(0,variance), GetRandN(0,variance));
    return this->pos() + noise;
}

void Robot::VisualizeGPSMeasurment(QPointF pos)
{
    //QGraphicsEllipseItem* posGPS = new QGraphicsEllipseItem();
    static QGraphicsEllipseItem posGPS;
    static bool firstRun = true;
    if (firstRun)
    {
        firstRun = false;
        scene()->addItem(&posGPS);
    }
    posGPS.setRect(pos.x(), pos.y(), 5, 5);
    posGPS.setBrush(Qt::red);
}

void Robot::VisualizeEstimatedPosition(QPointF pos)
{
    //QGraphicsEllipseItem* posGPS = new QGraphicsEllipseItem();
    static QGraphicsEllipseItem p;
    static bool firstRun = true;
    if (firstRun)
    {
        firstRun = false;
        scene()->addItem(&p);
    }
    p.setRect(pos.x(), pos.y(), 10, 10);
    p.setBrush(Qt::green);
}


// TrackKalman()
// Input: z - the measurement (x,y)
// Output: xy - the estimated position (x,y)
QPointF TrackKalman(QPointF meas)
{
    using namespace Eigen;
    static double dt = LOOP_RATE_SIM; // sample time. TODO: calculate what this should be based on simulation time
    static bool firstRun = true;

    static Matrix4f A;
    static MatrixXf H(2,4);
    static Matrix4f Q;
    static Matrix2f R;
    static Vector4f xh;
    static Matrix4f P;

    if (firstRun)
    {
        firstRun = false;
        // The model is simple. Just have 1 derivative each for x,y
        A << 1, dt,  0,   0,
            0,  1,  0,   0,
            0,  0,  1,  dt,
            0,  0,  0,   1;
        // The measurement matrix. We measure only x and y position
        H << 1,  0,  0,  0,
            0,  0,  1,  0;

        // State transition noise covariance matrix. Lower values
        // result in smoother estimate
        Q.setIdentity();
        Q *= 0.01;

        // Measurement noise covariance matrix. Higher values result
        // in smoother estimate
        R << 50,  0,
            0, 50;

        xh << 0, 0, 0, 0; // the initial state estimate

        // set the initial estimate of error covariance (start with high error)
        P.setIdentity();
        P *= 100;
    }

    MatrixXf z(2,1);
    z << meas.x(), meas.y();               // the x,y position measurement

    Vector4f xp = A*xh;                  // predicted state
    Matrix4f Pp = A*P*A.transpose() + Q;            // updated covariance
    Vector4f xp = A*xh;                    // predicted state
    Matrix4f Pp = A*P*A.transpose() + Q;   // prediction of the error covariance
    Matrix2f tmp = H*Pp*H.transpose() + R;
    MatrixXf K(4,2);
    K = Pp*H.transpose()*tmp.inverse(); // compute the kalman gain
    K = Pp*H.transpose()*tmp.inverse();    // compute the kalman gain

    xh = xp + K*(z - H*xp);     // combination of prediction and measurement
    P = Pp - K*H*Pp;
    xh = xp + K*(z - H*xp);                // combination of prediction and measurement
    P = Pp - K*H*Pp;                       // estimate the error covariance

    // return the state estimate
    QPointF estimatedPosition(xh[0], xh[2]);
    return estimatedPosition;
}

void Robot::advance(int step)
{
    simTime_ += SIM_TIME_INCREMENT;
    DisplayTime();
    if (!step)
        return;

    // (i) get measurement (add noise to omega set in step iv)
    //ImuMeasurement* imuMeas = nullptr;
    //State* cameraMeas = nullptr;
    //std::tie(imuMeas, cameraMeas) = GetMeasurements();

    // (ii) estimate state using EKF
    // state = kalmanFilter_.EstimateState(imuMeas, cameraMeas);
    QPointF gpsMeas = GetGPSMeasurement();
    VisualizeGPSMeasurment(gpsMeas);

    // Apply kalman filter to gps measurements
    QPointF estimatedPosition = TrackKalman(gpsMeas);
    // visualize the estimated robot position from gps measurements
    // just copy visualize gps measurement function and use different color.
    VisualizeEstimatedPosition(estimatedPosition);

    State state = GetRobotState();
    //State state = GetEstimatedPose();
    //if (cameraMeas != nullptr) SetEstimatedPose(*cameraMeas);

    // (iii) compute velocity from diff drive controller
    Velocity velocity(0,0);
    if (!goals_.empty())
    {
        Waypoint* wayPt = goals_.front();
        velocity = GenerateRobotControl(state, wayPt);
    }
    // (iv) command robot velocity to that computed by diff drive controller (plus process noise)
    CommandRobotVelocity(velocity);

    // update the position of the robot based on the set velocity
    UpdatePosition();

    // Add "bread crumbs" to represent where the robot has been
    ManageRobotTrail();
}

void Robot::ManageRobotTrail()
{
    static std::queue<QGraphicsRectItem*> trailPoints;
    QGraphicsRectItem* trailPoint = new QGraphicsRectItem();
    QPointF point = mapToScene(-30, 0);
    trailPoint->setRect(point.x(), point.y(), 2, 2);
    trailPoint->setBrush(Qt::green);
    //add the item to the scene
    scene()->addItem(trailPoint);
    trailPoints.push(trailPoint);
    const size_t NUM_POINTS_TO_KEEP = 100;
    while (trailPoints.size() > NUM_POINTS_TO_KEEP)
    {
        auto point = trailPoints.front();
        trailPoints.pop();
        scene()->removeItem(point);
    }
}

void Robot::CommandRobotVelocity(Velocity _vel)
{
    // add noise to velocity to model the process noise
    //_vel.linear += GetRandN(0.0, processNoiseLinear_);
    //_vel.angular += GetRandN(0.0, processNoiseAngular_);

    SetVelocity(_vel);
}

void Robot::SetVelocity(Velocity _vel)
{
    vel_ = _vel;
}

void Robot::AddAprilTag(AprilTag* _at)
{
    aprilTags_.push_back(_at);
}

bool Robot::TagIsInFOV(AprilTag* _tag)
{
    QPointF pt = mapFromScene(_tag->pos());
    QPolygonF polyFOV;
    for (auto& pt : pointsFOV_)
    {
        polyFOV << pt;
    }
    // create graphicspolygonitem object to determine if point is contained within it
    return polyFOV.containsPoint(pt, Qt::OddEvenFill);
}

// get robots pose in april tag coordinate frame
// add noise to the pose. 
// * Add more x/y noise if far away.
// * add more heading noise if heaing is very different
// convert that position to scene frame

State* Robot::GetNoisyPose(AprilTag* _tag)
{
    //QPointF ptOffset = mapFromScene(_tag->pos());
    auto robotPos_Tag = mapToItem(_tag, QPointF(0,0));
    // std::cout << "GetNoisePose:: Robot pos Tag = " << robotPos_Tag.x()
    //           << ", " << robotPos_Tag.y() << "\n";

    auto robotPos_Scene = _tag->mapToScene(robotPos_Tag);
    // std::cout << "GetNoisePose:: Robot pos Scene = " << robotPos_Scene.x()
    //           << ", " << robotPos_Scene.y() << "\n";

    // add noise
    double robotX_Noise = robotPos_Scene.x() + (std::rand() % 20)-10;
    double robotY_Noise = robotPos_Scene.y() + (std::rand() % 20)-10;
    double theta = rotation() + (std::rand() % 5)*0.1 - 0.2;

    noisyPose_ = State(robotX_Noise, robotY_Noise, theta);
    
    return &noisyPose_;
}

State Robot::GetEstimatedPose()
{
    double x = estimatedPose_->x();
    double y = estimatedPose_->y();
    double theta = estimatedPose_->rotation() * M_PI/180.0;
    
    return State(x,y,theta);
}

void Robot::SetEstimatedPose(double x, double y, double theta)
{
    estimatedPose_->setPos(x, y);
    estimatedPose_->setRotation(theta);
}

void Robot::SetEstimatedPose(State _pose)
{
    estimatedPose_->setPos(_pose.x, _pose.y);
    estimatedPose_->setRotation(_pose.theta);
}

void Robot::SetEstimatedPose(EstimatedPose* _pose)
{
    estimatedPose_ = _pose;
}


ImuMeasurement* Robot::GetMeasurementIMU()
{
    //if (!imuReady())
         return nullptr;

    // imuMeasurement_.omega = vel_.angular + GetRandN(0.0, imuNoise_);
    // imuMeasurement_.stamp = GetSimTime();
    // return &imuMeasurement_;
}

bool Robot::CameraReady()
{
    static double lastCameraMeasurement = simTime_;
    static const double cameraRefreshTime_ = 1.0;

    if (simTime_ - lastCameraMeasurement> cameraRefreshTime_)
    {
        lastCameraMeasurement = simTime_;
        return true;
    }
    else
    {
        return false;
    }
}

State* Robot::GetMeasurementAprilTag()
{
    // get offset of april tags
     if (!CameraReady())
         return nullptr;

    for (auto tag : aprilTags_)
    {
        // transform april tag position into robot coordinate
        // frame. Then determine if the tag lies in the robot camera's
        // FOV
        if (TagIsInFOV(tag))
        {
            tag->SetColor(Qt::green);
            // For now, just return the position based on the firs tag
            return GetNoisyPose(tag);
            //std::cout << "Noisy rotation = " << pose.theta << "\n";            
        }
    }
    return nullptr;
}
    //    Point2D offset = tag_ - position;

    // This function should
    // * compute the offset between robot and tag,
    // * add some noise to offset in proportion to distance and angle
    // * return the noisy offset and the april tags position
    
    // Another function accepts the noisy offset and april tags position and then computes the robots position

    

    // Bot should infer its own position from those 2 pieces of information

    
    //visionMeasurementReady = true;
    //visionMeasurement_ =  ?;
        // the python measuremnent code for april tags
/*
def get_measurements(self):
        """
        Returns a list of lists of visible landmarks and a fresh boolean that
        checks if the measurement is ready
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
                np.absolute(np.arccos(x_new[0] / (math.sqrt(x_new[0]**2 + x_new[1]**2))) )
            if abs(marker_view_angle) < self.__view_half_angle and abs(theta_new) < np.pi/3 and x_new[0] < 2:
                self.__visible_markers[i] = True
                meas_i = np.array([x_new[0], x_new[1], theta_new, self.markers_flipped[i][3], self.last_meas_time])
                meas_i[0:3] = meas_i[0:3] + np.array([np.random.normal(0, self.__image_noise)])
                meas.append(meas_i.tolist())
        if not meas:
            meas = None
        return meas
*/



void Robot::AddWaypoint(Waypoint* _waypt)
{
    std::cout << "Adding waypoint at " << _waypt->x() << ", " << _waypt->y() << "\n";    
    goals_.push(_waypt);
    scene()->addItem(_waypt);
}

void Robot::UpdatePosition()
{
    double theta = rotation() * M_PI/180.0;
    double dt = 1/LOOP_RATE_SIM; // time step (seconds)
    double xn = x() + dt*std::cos(theta)*vel_.linear;
    double yn = y() + dt*std::sin(theta)*vel_.linear;
    theta += dt*vel_.angular;

    if (theta > M_PI)
        theta -= 2*M_PI;
    if (theta < -M_PI)
        theta += 2*M_PI;

    // std::cout << "Rotation = " << rotation() << "\n";
    // std::cout << "PositionX = " << x() << "\n";
    // std::cout << "PositionY= " << y() << "\n";
    setPos(xn, yn);
    setRotation(theta*180/M_PI);
    emit PositionChanged(xn, yn);
    emit UpdateVelocity(vel_.linear);
}
