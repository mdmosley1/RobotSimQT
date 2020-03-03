#ifndef HELPERFUNCTIONS_H
#define HELPERFUNCTIONS_H

#include <random>
double GetRandN(double mean, double var)
{
    std::random_device r;
    std::default_random_engine gen(r());
    std::normal_distribution<double> dist(mean, var);
    return dist(gen);
}

#endif /* HELPERFUNCTIONS_H */
