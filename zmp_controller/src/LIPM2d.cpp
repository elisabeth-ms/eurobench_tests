/*
 * 2dLIPM.cpp
 *
 *  Created on: 03/12/2015
 *      Author: teo
 */

#include "LIPM2d.h"
#include <iostream>
LIPM2d::LIPM2d()
{

}


LIPM2d::LIPM2d(float pendulumLongitude, float mass){
    _L = pendulumLongitude;
    _M = mass;
    _ang_ref = 0.0; // inintialization of the inputs
    _zmp_ft = 0;
    //_zmp_ref = 0.0;

    _a = 8.45946778e-02;  //10.758;//-0.166340241;	//-10.758;//-10.758;//; -4.966517054; // quadratic error equacion from exp1 (testCap53)
    _b = 9.91381843e-01;//-1.0366;//-1.0366; //1.024; 1.152608355;
    _c = 1.33281740e-05;//0.0865;//-0.0004; -0.002387972;

    _u = 0.0; // inintialization of the outputs
    _u_ref = 0.0;
    y = 0.0;

    _x1[0] = 0.0;  // inintialization of the space state variable
    _x1[1] = 0.0;
    _x2[0] = 0.0;
    _x2[1] = 0.0;

    _z[0] = 0.0;
    _z[1] = 0.0;

    pre_y = 0.0; // others options (not used)
    dy = 0.0;

    PIDout = 0.0; // PID options (not used)
    Pout = 0.0;
    Iout = 0.0;
    Dout = 0.0;

    // initialization damping variables
    _Wn = 0.0; // natural frecuency
    _chi = 1.0; // damping coefficient
    _ka = 0.11; // spring
    _ka_const = 0.10;
    _ba = 38.6844; // shock

    //_ka = 9.9971; // para zmp_ref=0.09
//    _ba = 30.6949; // para chi=0.8 - para zmp_ref=0.09
    //_ba = 51.1582; // para chi = 1 - para zmp_ref=0.09

}

LIPM2d::~LIPM2d(){
}

float LIPM2d::model(float ft, float ref)     /** STATE FEEDBACK WITH DAMPING (ka, ba) **/
{
    // Discrete-time Space State Model evaluation

    _zmp_error =  ref - ft;

    std::cout<<_zmp_error<<std::endl;

    _x1[0] = _x1[1];
    _x2[0] = _x2[1];

/*
    _chi = 1;

    % ZMP_FT = ANG_ref*Ks = ANG_ref * K / Beta = ANG_ref * (-G/L^2) / ((ka-G)./L)
    ANG_ref = -(asin(ZMP_ref/L) * 180/G) ;
    ka = ((ANG_ref*-1*G/L)/ZMP_ref)+G;

    % Beta = (ka-G)./L = Wn^2
    Wn = sqrt((ka-G)/L);

    % Alpha = ba./(m*L) = 2*chi*Wn
    ba = 2*chi*Wn*m*L;

    K = -G/L^2;
    Num = K;

    Alpha = ba/(m*L);
    Beta = (ka-G)/L;
*/

    _ka_const = 9.98+ref*0.01;//0.25 * ref + 9.95;
    _ang_ref = (ref*(-G))/ (_L*(_ka_const-G));
    //_ang_ref = -(asin(ref/L)*180/G); // relacion entre zmp_ref y ang_ref

    _zmp_ft = _a*ref*ref + _b*ref + _c; // equation from exp1 (stationary error)
    std::cout<<"_ang_ref: "<<_ang_ref<<std::endl;
    std::cout<<"zmp ref: "<<ref<<std::endl;
    std::cout<<"_a: "<<_a<<" _b: "<<_b<<" _c: "<<_c<<std::endl;

    std::cout<<"_zmp_ft: "<<_zmp_ft<<std::endl;
    std::cout<<"ft measured: "<<ft<<std::endl;

    if (_ang_ref==0 || _zmp_ft<0.00000000001){ // calculation of ka in steady state
        _ka = 9.98;//9.95;
    }
    else    {
        _ka = ((_zmp_ft*(-1)*G/_L)/_ang_ref)+G;
    }
    std::cout<<"ka: "<<_ka<<std::endl;
    std::cout<<"_ka-G: "<<_ka-G<<std::endl;
    std::cout<<"_L: "<<_L<<std::endl;


    _Wn = sqrt((_ka-G))/_L; // based on the general form of a 2nd order transfer function
    _ba = 2*_chi*_Wn*_M*_L; // calculation of ba related to the previous line and chi = 1 (damping coefficient);

    std::cout<<"_Wn: "<<_Wn<<std::endl;
    std::cout<<"_ba: "<<_ba<<std::endl;


    // Dynamical-LIPM represented in space state
    _A[0][0] = 0.0;
    _A[0][1] = 1.0;
    _A[1][0] = -((_ka-G)/_L);
    _A[1][1] = -(_ba/(_M*_L));

    _B[0][0] = 0.0;
    _B[1][0] = 1.0;

    _C[0] = -G/(_L*_L);
    _C[1] = 0.0;

    _D = 0.0;

    std::cout<<"_A[0][0]: "<<_A[0][0]<<"_A[0][1]: "<<_A[0][1]<<"_A[1][0]: "<<_A[1][0]<<"_A[1][1]: "<<_A[1][1]<<std::endl;
    std::cout<<"_B[0][0]: "<<_B[0][0]<<"_B[1][0]: "<<_B[1][0]<<std::endl;
    std::cout<<"_C[0]: "<<_C[0]<<"_C[1]: "<<_C[1]<<std::endl;
    std::cout<<"_D: "<<_D<<std::endl;


    /* // different options for the control (PD, PI, )
    //_u_ref = _zmp_ref/L; // L = 0.8927 is the pendulum longitude.
    //_u_ref = (_zmp_ref/L) * 180 / G;

    //_zmp_error = _zmp_ref - zmp_real;
    //_angle_error = (_zmp_error/L) * 180 / G;

    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Ki*(_pre_zmp_error + _zmp_error)*_T + _Kp*_zmp_error - _Ku*_u_ref; // basico - LOLI
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Ki*(_pre_zmp_error + _zmp_error)*_T + _Kp*_zmp_error - _Ku*_u_ref; //
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Kp*_zmp_error - _Ku*_u_ref; // sin Ki
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + _Kp*_zmp_error + _Kd*((_zmp_error-_pre_zmp_error)/_T) - _Ku*_u_ref; // sin Ki
    //_u = -_K[0]*_x1[0] -_K[1]*_x2[0] + (_pre_zmp_error + _zmp_error) - _Ku*_u_ref;
    //_zmp_PD = _Kp*_zmp_error + _Kd*((_zmp_error-_pre_zmp_error)/_T);
    //_angle_error =  (_zmp_PD/L) * 180 / G;; // sin Ki
    //_u =  _angle_error -_K[0]*_x1[0] -_K[1]*_x2[0] - _Ku*_u_ref;
*/

    // if(abs(ft-ref)>=0.1*ref){
        _u = _zmp_error; //bucle abierto. nuestra entrada es directamente la refencia de ZMP que queramos.
        std::cout<<_x1[0]<<std::endl;
        y = _C[0]*_x1[0] + _C[1]*_x2[0] + _D*_u;
        //dy = (y - pre_y) / _T; // velocity

        _x1[1] = _A[0][0]*_x1[0] + _A[0][1]*_x2[0] + _B[0][0]*_u;
        _x2[1] = _A[1][0]*_x1[0] + _A[1][1]*_x2[0] + _B[1][0]*_u;

        // std::cout<<_x1[1]<<std::endl;

        //pre_y = y;
        //_pre_zmp_error = _zmp_error;

        ang_error_out = y; // only for a intuitive name fot the output
    // }
    std::cout<<ang_error_out<<std::endl;
    return ang_error_out;

}
