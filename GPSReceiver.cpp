#include "GPSReceiver.h"
#include "helperFunctions.h" // GetRandN

GPSReceiver::GPSReceiver()
{

}
QPointF GPSReceiver::GetGPSMeasurement(const QPointF _pos) const
{
    QPointF noise(GetRandN(0,variance_), GetRandN(0,variance_));
    return _pos + noise;
}

void GPSReceiver::IncreaseNoise()
{
    variance_ *= 1.5;
}
void GPSReceiver::DecreaseNoise()
{
    variance_ /= 1.5;
}
