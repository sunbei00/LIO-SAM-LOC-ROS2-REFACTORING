//
// Created by root on 11/7/24.
//

#ifndef BUILD_GPSUTILS_H
#define BUILD_GPSUTILS_H
#include <utility>
#include <cmath>

std::pair<double, double> latlon_to_utm(double lat, double lon) {
    const double a = 6378137.0; // WGS84 타원체의 장반경(m)
    const double f = 1 / 298.257223563; // WGS84 타원체의 편평도
    const double k0 = 0.9996; // UTM 스케일 팩터

    double zone_number = std::floor((lon + 180) / 6) + 1;
    double lon_origin = (zone_number - 1) * 6 - 180 + 3;
    double lon_origin_rad = lon_origin * M_PI / 180.0;

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;

    double N = a / std::sqrt(1 - f * (2 - f) * std::sin(lat_rad) * std::sin(lat_rad));
    double T = std::tan(lat_rad) * std::tan(lat_rad);
    double C = f * (2 - f) * std::cos(lat_rad) * std::cos(lat_rad) / (1 - f * (2 - f));
    double A = (lon_rad - lon_origin_rad) * std::cos(lat_rad);

    double M = a * ((1 - f * (2 - f) / 4 - 3 * f * f * (2 - f) * (2 - f) / 64) * lat_rad
                    - (3 * f * (2 - f) / 8 + 3 * f * f * (2 - f) / 32) * std::sin(2 * lat_rad)
                    + (15 * f * f * (2 - f) * (2 - f) / 256) * std::sin(4 * lat_rad));

    double easting = k0 * N * (A + (1 - T + C) * A * A * A / 6);
    double northing = k0 * (M + N * std::tan(lat_rad) * (A * A / 2 + (5 - T + 9 * C + 4 * C * C) * A * A * A * A / 24));

    return {easting, northing};
}

#endif //BUILD_GPSUTILS_H
