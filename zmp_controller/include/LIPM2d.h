/*
 * LIPM2d.h
 *
 *  Created on: 04/12/2015
 *      Author: teo
 */

#ifndef LIPM2D_H_
#define LIPM2D_H_

#include <math.h>
//General constants
#define PI  3.14159265358979323846
#define G   9.80665 // Gravity [m/s²]
#define TS  0.03 // Time Samples [s]
// #define L   0.8927 // Pendulum Longitude [m]
// #define M   62.589 // Robot mass [kg]
#define e   0.03225 // Height FT sensor // Loli 0.0194

class LIPM2d{
public:
    LIPM2d();
    LIPM2d(float pendulumLongitude, float mass);
    ~LIPM2d();

    float model(float ft,float ref);

    float _x1[2]; // state variable
    float _x2[2]; // model state variable
    float _zmp_error, _angle_error, _zmp_PD; // error between reference ZMP and actual ZMP.
    float y, ang_error_out; // zmp output signal (actual value)
    float pre_y; // zmp previous value
    float dy; // velocity signal (to ankle joints)
    float _zmp_ref, _ang_ref, _zmp_ft; // ZMP reference. Static posture = 0.0
    float _u_ref; // reference pendulum angle. In static position, the corresponding
              // angle to the reference ZMP u_ref = 0.0
    float _u, _beta, _alpha; //Compensated angle
    float _ka, _ba, _chi, _Wn, _ka_const; // variables del muelle y amortiguador añadido.
    float _a, _b, _c; // variables de la funcion zmp_ft=f(zmp_ref). (sacado de exp1)
    float _z[2]; // model state variable


private:
    float _A[2][2];
    float _B[2][1];
    float _C[2];
    float _D;
    float _K[2]; // feedback LQR gain
    float _Kp, _Ki, _Ku; // P, I, U gains
    float _Kd;
    float _T; // sample time


    float _pre_zmp_error; // previous z --> z[k-1]

    float PIDout, Pout, Iout, Dout;
    float _L;
    float _M;

};

#endif /* LIPM2D_H_ */
