#include <QPointF>
#include <Eigen/Dense>
#include "constants.hh" // LOOP_RATE_SIM

class KalmanFilter
{
public:
    void Init();
    QPointF Filter(QPointF meas);
private:
    double dt = SIM_TIME_INCREMENT; // sample time. TODO: calculate what this should be based on simulation time

    Eigen::Matrix4d A;
    Eigen::Matrix<double, 2, 4> H;
    Eigen::Matrix4d Q;
    Eigen::Matrix2d R;
    Eigen::Vector4d xh;
    Eigen::Matrix4d P;
};
