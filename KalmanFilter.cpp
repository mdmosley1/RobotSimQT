#include "KalmanFilter.h"
#include <iostream>
#include "yaml-cpp/yaml.h"

using namespace Eigen;

void KalmanFilter::Init()
{
    // Load params from yaml file
    YAML::Node config = YAML::LoadFile("config.yaml");
    const double measStdDevX = config["measStdDevX"].as<double>();
    const double measStdDevY = config["measStdDevY"].as<double>();
    const double processStdDevX = config["processStdDevX"].as<double>();
    const double processStdDevXdot = config["processStdDevXdot"].as<double>();
    const double processStdDevY = config["processStdDevY"].as<double>();
    const double processStdDevYdot = config["processStdDevYdot"].as<double>();

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
    Q(0,0) = pow(processStdDevX,2);
    Q(1,1) = pow(processStdDevXdot,2);
    Q(2,2) = pow(processStdDevY,2);
    Q(3,3) = pow(processStdDevYdot,2);

    // Measurement noise covariance matrix. Higher values result
    // in smoother estimate
    R << pow(measStdDevX,2),  0,
        0, pow(measStdDevY,2);

    xh << 0, 0, 0, 0; // the initial state estimate

    // set the initial estimate of error covariance (start with high error)
    P.setIdentity();
    P *= 100;
}

// PredictTrajectory
// Predict the next n states using the system model
// Inputs:
//   n - number of states to predict into the future
//   skipFactor - skip this many iterations between predictions
std::vector<QPointF> KalmanFilter::PredictTrajectory(int n, int skipFactor)
{
    std::vector<QPointF> trajectory;
    Vector4d x = xh;
    int idx = 0;
    while (trajectory.size() < n)
    {
        idx++;
        x = Predict(x);
        if (idx % skipFactor == 0)
        {
            trajectory.push_back( QPointF(x(0), x(2)) );
        }
    }
    return trajectory;
}

Vector4d KalmanFilter::Predict(Vector4d xh)
{
    return A*xh;
}

// TrackKalman()
// Input: z - the measurement (x,y)
// Output: xy - the estimated position (x,y)
QPointF KalmanFilter::Filter(QPointF meas)
{
    MatrixXd z(2,1);
    z << meas.x(), meas.y();               // the x,y position measurement

    Vector4d xp = Predict(xh);               // predict the next state
    Matrix4d Pp = A*P*A.transpose() + Q;   // prediction of the error covariance
    Matrix2d tmp = H*Pp*H.transpose() + R;
    MatrixXd K(4,2);
    K = Pp*H.transpose()*tmp.inverse();    // compute the kalman gain

    xh = xp + K*(z - H*xp);                // combination of prediction and measurement
    P = Pp - K*H*Pp;                       // estimate the error covariance

    // return the state estimate
    QPointF estimatedPosition(xh[0], xh[2]);
    return estimatedPosition;
}
