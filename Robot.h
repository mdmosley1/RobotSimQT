#ifndef ROBOT_H
#define ROBOT_H

#include <QObject>
#include <QGraphicsItem>
#include <queue>
#include "Waypoint.h"
#include "AprilTag.h"
#include "EstimatedPose.h"
#include "KalmanFilter.h"

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

struct ImuMeasurement
{
    ImuMeasurement(double _o, double _s) : omega(_o), stamp(_s) {};
    ImuMeasurement() : omega(0), stamp(0) {};
    double omega,stamp;
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
    void AddAprilTag(AprilTag* _at);
    void AddMembersToScene();
    void SetEstimatedPose(EstimatedPose*);
    void SetEstimatedPose(State);
    void SetEstimatedPose(double,double,double);
signals:
    void PositionChanged(double xn, double yn);
    void UpdateVelocity(double);
public slots:
    void advance(int step) override;
private:
    void CommandRobotVelocity(Velocity _vel);
    QPointF GetGPSMeasurement();
    State* GetMeasurementAprilTag();
    ImuMeasurement* GetMeasurementIMU();
    std::tuple<ImuMeasurement*, State*> GetMeasurements();
    void AddGoalToCompleted(Waypoint*);
    void Brake();
    bool CameraReady();
    void DisplayTime();
    void DecreaseLinearVelocity();
    void DecreaseAngularVelocity();
    State GetRobotState();
    State GetEstimatedPose();
    State* GetNoisyPose(AprilTag* _tag);
    Velocity GenerateRobotControl(State _state, Waypoint* _goal);
    void IncreaseLinearVelocity();
    void IncreaseAngularVelocity();
    void ManageRobotTrail();
    void SetVelocity(Velocity _vel);
    void UpdatePosition();
    bool TagIsInFOV(AprilTag* _tag);
    void VisualizeGPSMeasurment(QPointF pos);
    void VisualizeEstimatedPosition(QPointF pos);

    const double processNoiseLinear_ = 1.0;
    const double processNoiseAngular_ = 1.0;
    
    const double linearVelMax_ = 100;
    const double linearVelMin_ = 0;

    const double angularVelMax_ = 0.5;
    const double angularVelMin_ = -0.5;

    const double linearDelta_ = 1;
    const double angularDelta_ = 0.1;

    // std deviation of control noise
    const double controlNoiseLinear_ = 0.05;
    const double controlNoiseAngular_ = 0.05;

    const double imuNoise_ = 0.01;

    

    double imuMeas_ = 0.0; // the angular velocity measured by IMU

    Velocity vel_ = Velocity(0.0, 0.0);
    State state_;
    QColor color_;

    std::queue<Waypoint*> goals_;
    std::queue<Waypoint*> goalsCompleted_;
    

    std::vector<AprilTag*> aprilTags_;

    // camera parameters
    double cameraFOV_;
    double cameraMaxRange_;
    std::vector<QPointF> pointsFOV_; // points that determine the FOV triangle

    bool aprilTagReady_ = false;

    EstimatedPose* estimatedPose_ = nullptr;

    State noisyPose_;

    ImuMeasurement imuMeasurement_;

    double simTime_ = 0.0;

    QGraphicsTextItem* timeTextItem_;

    KalmanFilter KalmanFilter_;
};
#endif // ROBOT_H
