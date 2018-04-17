#include <robosub/robosub.h>

using namespace std;
using namespace cv;

//pi / 180
#define DEGREE 0.01745329251994329576923690768488612713443

int main() {
    //motor angles
    //motor 1 = top left
    //motor 2 = top right
    //motor 3 = bottom left
    //motor 4 = bottom right
    //angle is from right counterclockwise
    const double MOTOR_ANGLE = 35.0*DEGREE;
    const double MOTOR_1_ANGLE = 90.0*DEGREE - MOTOR_ANGLE;
    const double MOTOR_2_ANGLE = 90.0*DEGREE + MOTOR_ANGLE;
    const double MOTOR_3_ANGLE = -90.0*DEGREE + MOTOR_ANGLE;
    const double MOTOR_4_ANGLE = -90.0*DEGREE - MOTOR_ANGLE;

    //robot dimensions
    const double ROBOT_LENGTH = 12; //in
    const double ROBOT_WIDTH = 12.85; //in
    const double THETA_BASE = atan2(ROBOT_LENGTH, ROBOT_WIDTH);
    const double MOTOR_TO_CENTER_OF_MASS = sqrt(ROBOT_LENGTH*ROBOT_LENGTH + ROBOT_WIDTH*ROBOT_WIDTH);

    //angle of vector from center of mass to motor
    const double MOTOR_1_TAU = 180.0*DEGREE - THETA_BASE;
    const double MOTOR_2_TAU = THETA_BASE;
    const double MOTOR_3_TAU = 180.0*DEGREE + THETA_BASE;
    const double MOTOR_4_TAU = -THETA_BASE;

    //input variables
    //tx, ty = translation
    //r = rotation
    //input from -1 to 1
    double tx = 0, ty = 0, r = 0.5;

    //create matrix of inputs
    double _M[4][4] = {
        { cos(MOTOR_1_ANGLE), cos(MOTOR_2_ANGLE), cos(MOTOR_3_ANGLE), cos(MOTOR_4_ANGLE) },
        { sin(MOTOR_1_ANGLE), sin(MOTOR_2_ANGLE), sin(MOTOR_3_ANGLE), sin(MOTOR_4_ANGLE) },
        { MOTOR_TO_CENTER_OF_MASS*sin(MOTOR_1_ANGLE - MOTOR_1_TAU), MOTOR_TO_CENTER_OF_MASS*sin(MOTOR_2_ANGLE - MOTOR_2_TAU),
                MOTOR_TO_CENTER_OF_MASS*sin(MOTOR_3_ANGLE - MOTOR_3_TAU), MOTOR_TO_CENTER_OF_MASS*sin(MOTOR_4_ANGLE - MOTOR_4_TAU)},
        { 1, 1, 1, 1 }
    };
    Mat Minv = Mat(4, 4, CV_64FC1, &_M).inv();

    cout << Minv << endl;

    //TODO main loop

    //create vector of results
    double _b[1][4] = { { tx, ty, r, 0 } };
    Mat b = Mat(1, 4, CV_64FC1, &_b);
    transpose(b, b);

    cout << b << endl;

    //solve system of equations
    Mat x = Minv * b;
    cout << x << endl;

    //normalize solution
    double minVal, maxVal;
    cv::minMaxIdx(abs(x), &minVal, &maxVal);
    if (maxVal > 0) {
        x /= maxVal;
    }
    double bNorm = sqrt(tx*tx + ty*ty + r*r);
    x *= bNorm;

    cout << x << endl;
}
