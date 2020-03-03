#include <QPointF>
#ifndef GPSRECEIVER_H
#define GPSRECEIVER_H

class GPSReceiver
{
public:
    GPSReceiver();
    QPointF GetGPSMeasurement(const QPointF) const;
    void IncreaseNoise();
    void DecreaseNoise();
private:
    // variance of measurment noise
    double variance_ = 5.0;
};

#endif /* GPSRECEIVER_H */
