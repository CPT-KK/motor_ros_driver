#include <vector>
#include <math.h>

inline double rad2deg(double rad) {
    return rad * 57.295779513082320;
}

inline float rad2deg(float rad) {
    return rad * 57.295779513082320f;
}

inline double deg2rad(double deg) {
    return deg * 0.017453292519943;
}

inline float deg2rad(float deg) {
    return deg * 0.017453292519943f;
}

inline double wrapToPi(double in) {
    while (abs(in) > M_PI) {
        in = in - 2 * M_PI * in / abs(in);
    }
    return in;
}

inline float wrapToPi(float in) {
    while (abs(in) > M_PI) {
        in = in - 2 * M_PI * in / abs(in);
    }
    return in;
}

inline double wrapTo180(double in) {
    return rad2deg(wrapToPi(in));
}

inline float wrapTo180(float in) {
    return rad2deg(wrapToPi(in));
}

inline double wrapTo2Pi(double in) {
    while (in > 2 * M_PI || in < 0) {
        in = in - 2 * M_PI * in / abs(in);
    }
    return in;
}

inline float wrapTo2Pi(float in) {
    while (in > 2 * M_PI || in < 0) {
        in = in - 2 * M_PI * in / abs(in);
    }
    return in;
}

inline double wrapTo360(double in) {
    return rad2deg(wrapTo2Pi(in));
}

inline float wrapTo360(float in) {
    return rad2deg(wrapTo2Pi(in));
}