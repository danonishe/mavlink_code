#pragma once
#include <cmath>


class getCoordinate
{

    #define M_PI 3.1415
public:
    double azimut = 0;
    double dist = 0;
    double x = 0;
    double y = 0;
    void getDistAndAzimut(double lat1, double lon1, double lat2, double lon2);
    void getInertialCoordinates(double dist, double angle, double x_now, double y_now);

};

