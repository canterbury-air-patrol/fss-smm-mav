#pragma once

class Point {
private:
    bool valid{false};
    double latitude{0.0};
    double longitude{0.0};
public:
    Point() {};
    Point(double lat, double lng) : valid(true), latitude(lat), longitude(lng) {};
    double getLatitude() { return latitude; };
    double getLongitude() { return longitude; };
};
