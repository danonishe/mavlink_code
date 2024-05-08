#include "getCoordinate.h"

void getCoordinate::getDistAndAzimut(double nlat1, double nlon1, double nlat2, double nlon2)
{
    double rad = 6372795;

    /*double llat1 = 55.123;
    double llong1 = 8.123;
    double llat2 = 55.423;
    double llong2 = 8.223;*/
    double llat1 = nlat1;
    double llong1 = nlon1;
    double llat2 = nlat2;
    double llong2 = nlon2;
    double lat1 = llat1 * M_PI / 180.;
    double lat2 = llat2 * M_PI / 180.;
    double long1 = llong1 * M_PI / 180.;
    double long2 = llong2 * M_PI / 180.;
    double cl1 = std::cos(lat1);
    double cl2 = std::cos(lat2);
    double sl1 = std::sin(lat1);
    double sl2 = std::sin(lat2);
    double delta = long2 - long1;
    double cdelta = std::cos(delta);
    double sdelta = std::sin(delta);
    double y = std::sqrt(std::pow(cl2 * sdelta, 2) + std::pow(cl1 * sl2 - sl1 * cl2 * cdelta, 2));
    double x = sl1 * sl2 + cl1 * cl2 * cdelta;
    double ad = std::atan2(y, x);
    double dist = ad * rad;

    x = (cl1 * sl2) - (sl1 * cl2 * cdelta);
    y = sdelta * cl2;
    double z = std::atan(-y / x) * 180 / M_PI;

    if (x < 0) {
        z += 180;
    }

    double z2 = std::fmod(z + 180., 360.) - 180.;
    double anglerad2 = z2 - (2 * M_PI * std::floor(z2 / (2 * M_PI)));
    double angledeg = anglerad2 * 180 / M_PI;
    double arr[2];
    this->dist = dist;
    this->azimut = angledeg;
    /*std::cout << "Distance >> " << static_cast<int>(dist) << " [meters]" << std::endl;
    std::cout << "Initial bearing >> " << angledeg << " [degrees]" << std::endl;*/
}
double DegToRad(double D)
{
    double M = 3.14 / 180;
    return D * M;
};
void getCoordinate::getInertialCoordinates(double dist, double angle, double x_now, double y_now)
{
    double x = 0;
    double y = 0;
    double ox = 1;
    double oy = 1;
    double angle_const = angle;

    if (90 < angle && angle < 180) {
        angle = angle - 90;
        oy = -1;
    }
    if (180 < angle && angle < 270) {
        angle = angle - 180;
        ox = -1;
        oy = -1;
    }
    if (270 < angle && angle < 360) {
        angle = angle - 270;
        ox = -1;
    }
    double s = sin(DegToRad(angle));
    y = (dist * sin(DegToRad(angle) + y_now)) * oy;
    x = (dist * (cos(DegToRad(angle)) + x_now)) * ox;
    this->x = x;
    this->y = y;
}