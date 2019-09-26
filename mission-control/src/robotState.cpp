#include "robotState.h"

Mat RobotState::motorValues(double tx, double ty, double r) {
    tx = smoothControllerValue(tx);
    ty = smoothControllerValue(ty);
    r = smoothControllerValue(r);

    double _b[1][4] = {{tx, ty, ROTATION_BIAS * r, 0}};
    Mat b = Mat(1, 4, CV_64FC1, &_b);
    transpose(b, b);

    //solve system of equations
    Mat x = Minv * b;

    //normalize solution
    double minVal, maxVal;
    cv::minMaxIdx(abs(x), &minVal, &maxVal);
    if (maxVal > 0) {
        x /= maxVal;
    }
    double bNorm = max(sqrt(tx * tx + r * r), abs(ty));
    x *= bNorm;

    return x;
}

double RobotState::smoothControllerValue(double x) {
    int s = x < 0 ? -1 : 1;
    x = abs(x);
    return s * min(max(2.5925925925 * x * x * x - 3.7407407407 * x * x + 2.348148148148 * x - 0.2, 0.0), 1.0);
}
