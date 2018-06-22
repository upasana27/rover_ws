//
// File: ikine_6dof.cpp
//
// MATLAB Coder version            : 3.3
// C/C++ source code generated on  : 19-Jun-2018 19:44:14
//

// Include Files
#include "rt_nonfinite.h"
#include "ikine_6dof.h"
#include "asin.h"
#include "asinh.h"
#include "complexTimes.h"
#include "sqrt1.h"
#include "ikine_6dof_rtwutil.h"
#include "ros/ros.h"
#include <sstream>
#include <iostream>
using namespace std;
using namespace ros;
robotic_Arm::spose& send;//create msgs
robotic_Arm::rpose& recieve;//create msgs


// Function Definitions

//
// Give input x,y,z co-ordinate and the roll,pitch and yaw angle
// along with the three link lengths and get the 6 actuators angle of
// rotation
// Arguments    : double X
//                double Y
//                double Z
//                double r
//                double p
//                double y
//                double a1
//                double a2
//                double a3
//                creal_T theta[6]
// Return Type  : void
//
void ikine_6dof(double x, double y, double z, double r, double p, double y,
                double a1, double a2, double a3,)
{
  double theta1;
  double r3;
  double theta2;
  double x;
  double R03[9];
  double b_x[9];
  int p1;
  int p2;
  int p3;
  double absx21;
  double absx31;
  int itmp;
  double c[9];
  creal_T theta5;
  creal_T u;
  double theta6_re;
  double theta6_im;
  creal_T v;
  theta1 = std::atan(Y / X);
  r3 = Z - a1;
  r3 = std::sqrt((X * X + Y * Y) + r3 * r3);
  theta2 = std::atan(Z - a1 / std::sqrt(X * X + Y * Y)) - std::acos(((a2 * a2 +
    r3 * r3) - a3 * a3) / (2.0 * a2 * r3));
  x = std::acos(((a1 * a1 + a2 * a2) - r3 * r3) / (2.0 * a1 * a2));
  R03[0] = std::cos(theta1) * std::cos(theta2 - (3.1415926535897931 - x));
  R03[3] = -std::cos(theta1) * std::sin(theta2 + (3.1415926535897931 - x));
  R03[6] = std::sin(theta1);
  R03[1] = std::sin(theta1) * std::cos(theta2 + (3.1415926535897931 - x));
  R03[4] = -std::sin(theta1) * std::sin(theta2 + (3.1415926535897931 - x));
  R03[7] = -std::cos(theta1);
  R03[2] = std::sin(theta2 + (3.1415926535897931 - x));
  R03[5] = std::cos(theta2 + (3.1415926535897931 - x));
  R03[8] = 0.0;
  memcpy(&b_x[0], &R03[0], 9U * sizeof(double));
  p1 = 0;
  p2 = 3;
  p3 = 6;
  r3 = std::abs(R03[0]);
  absx21 = std::abs(R03[1]);
  absx31 = std::abs(R03[2]);
  if ((absx21 > r3) && (absx21 > absx31)) {
    p1 = 3;
    p2 = 0;
    b_x[0] = R03[1];
    b_x[1] = R03[0];
    b_x[3] = R03[4];
    b_x[4] = R03[3];
    b_x[6] = R03[7];
    b_x[7] = R03[6];
  } else {
    if (absx31 > r3) {
      p1 = 6;
      p3 = 0;
      b_x[0] = R03[2];
      b_x[2] = R03[0];
      b_x[3] = R03[5];
      b_x[5] = R03[3];
      b_x[6] = 0.0;
      b_x[8] = R03[6];
    }
  }

  r3 = b_x[1] / b_x[0];
  b_x[1] /= b_x[0];
  absx21 = b_x[2] / b_x[0];
  b_x[2] /= b_x[0];
  b_x[4] -= r3 * b_x[3];
  b_x[5] -= absx21 * b_x[3];
  b_x[7] -= r3 * b_x[6];
  b_x[8] -= absx21 * b_x[6];
  if (std::abs(b_x[5]) > std::abs(b_x[4])) {
    itmp = p2;
    p2 = p3;
    p3 = itmp;
    b_x[1] = absx21;
    b_x[2] = r3;
    r3 = b_x[4];
    b_x[4] = b_x[5];
    b_x[5] = r3;
    r3 = b_x[7];
    b_x[7] = b_x[8];
    b_x[8] = r3;
  }

  r3 = b_x[5] / b_x[4];
  b_x[5] /= b_x[4];
  b_x[8] -= r3 * b_x[7];
  r3 = (b_x[5] * b_x[1] - b_x[2]) / b_x[8];
  absx21 = -(b_x[1] + b_x[7] * r3) / b_x[4];
  c[p1] = ((1.0 - b_x[3] * absx21) - b_x[6] * r3) / b_x[0];
  c[p1 + 1] = absx21;
  c[p1 + 2] = r3;
  r3 = -b_x[5] / b_x[8];
  absx21 = (1.0 - b_x[7] * r3) / b_x[4];
  c[p2] = -(b_x[3] * absx21 + b_x[6] * r3) / b_x[0];
  c[p2 + 1] = absx21;
  c[p2 + 2] = r3;
  r3 = 1.0 / b_x[8];
  absx21 = -b_x[7] * r3 / b_x[4];
  c[p3] = -(b_x[3] * absx21 + b_x[6] * r3) / b_x[0];
  c[p3 + 1] = absx21;
  c[p3 + 2] = r3;
  b_x[0] = std::cos(p) * std::cos(y);
  b_x[3] = -std::cos(p) * std::sin(y);
  b_x[6] = std::sin(p);
  b_x[1] = std::sin(r) * std::sin(p) * std::cos(y) + (std::cos(r) + std::sin(y));
  b_x[4] = -std::sin(r) * std::sin(p) * std::sin(y) + (std::cos(r) + std::cos(y));
  b_x[7] = -std::sin(r) * std::cos(p);
  b_x[2] = std::cos(r) * std::sin(p) * std::cos(y) + (std::sin(r) + std::sin(y));
  b_x[5] = std::cos(r) * std::sin(p) * std::sin(y) + (std::sin(r) + std::cos(y));
  b_x[8] = std::cos(r) * std::cos(p);
  for (p1 = 0; p1 < 3; p1++) {
    for (p2 = 0; p2 < 3; p2++) {
      R03[p1 + 3 * p2] = 0.0;
      for (p3 = 0; p3 < 3; p3++) {
        R03[p1 + 3 * p2] += c[p1 + 3 * p3] * b_x[p3 + 3 * p2];
      }
    }
  }

  theta5.re = R03[8];
  theta5.im = 0.0;
  b_asin(&theta5);
  if (theta5.im == 0.0) {
    u.re = std::cos(theta5.re);
    u.im = 0.0;
  } else if (theta5.re == 0.0) {
    u.re = std::cosh(theta5.im);
    u.im = 0.0;
  } else {
    u.re = std::cos(theta5.re) * std::cosh(theta5.im);
    u.im = -std::sin(theta5.re) * std::sinh(theta5.im);
  }

  if (u.im == 0.0) {
    theta6_re = R03[5] / u.re;
    theta6_im = 0.0;
  } else if (u.re == 0.0) {
    if (R03[5] == 0.0) {
      theta6_re = 0.0 / u.im;
      theta6_im = 0.0;
    } else {
      theta6_re = 0.0;
      theta6_im = -(R03[5] / u.im);
    }
  } else {
    absx31 = std::abs(u.re);
    r3 = std::abs(u.im);
    if (absx31 > r3) {
      r3 = u.im / u.re;
      absx21 = u.re + r3 * u.im;
      theta6_re = (R03[5] + r3 * 0.0) / absx21;
      theta6_im = (0.0 - r3 * R03[5]) / absx21;
    } else if (r3 == absx31) {
      if (u.re > 0.0) {
        r3 = 0.5;
      } else {
        r3 = -0.5;
      }

      if (u.im > 0.0) {
        absx21 = 0.5;
      } else {
        absx21 = -0.5;
      }

      theta6_re = (R03[5] * r3 + 0.0 * absx21) / absx31;
      theta6_im = (0.0 * r3 - R03[5] * absx21) / absx31;
    } else {
      r3 = u.re / u.im;
      absx21 = u.im + r3 * u.re;
      theta6_re = r3 * R03[5] / absx21;
      theta6_im = (r3 * 0.0 - R03[5]) / absx21;
    }
  }

  if ((theta6_im == 0.0) && (!(std::abs(theta6_re) > 1.0))) {
    theta6_re = std::acos(theta6_re);
    theta6_im = 0.0;
  } else {
    v.re = 1.0 + theta6_re;
    v.im = theta6_im;
    b_sqrt(&v);
    u.re = 1.0 - theta6_re;
    u.im = 0.0 - theta6_im;
    b_sqrt(&u);
    theta6_im = complexTimes(v.re, -v.im, u.re, u.im);
    b_asinh(&theta6_im);
    theta6_re = 2.0 * rt_atan2d_snf(u.re, v.re);
  }

  if (theta5.im == 0.0) {
    u.re = std::cos(theta5.re);
    u.im = 0.0;
  } else if (theta5.re == 0.0) {
    u.re = std::cosh(theta5.im);
    u.im = 0.0;
  } else {
    u.re = std::cos(theta5.re) * std::cosh(theta5.im);
    u.im = -std::sin(theta5.re) * std::sinh(theta5.im);
  }

  if (u.im == 0.0) {
    u.re = R03[7] / u.re;
    u.im = 0.0;
  } else if (u.re == 0.0) {
    if (R03[7] == 0.0) {
      u.re = 0.0 / u.im;
      u.im = 0.0;
    } else {
      u.re = 0.0;
      u.im = -(R03[7] / u.im);
    }
  } else {
    absx31 = std::abs(u.re);
    r3 = std::abs(u.im);
    if (absx31 > r3) {
      r3 = u.im / u.re;
      absx21 = u.re + r3 * u.im;
      u.re = (R03[7] + r3 * 0.0) / absx21;
      u.im = (0.0 - r3 * R03[7]) / absx21;
    } else if (r3 == absx31) {
      if (u.re > 0.0) {
        r3 = 0.5;
      } else {
        r3 = -0.5;
      }

      if (u.im > 0.0) {
        absx21 = 0.5;
      } else {
        absx21 = -0.5;
      }

      u.re = (R03[7] * r3 + 0.0 * absx21) / absx31;
      u.im = (0.0 * r3 - R03[7] * absx21) / absx31;
    } else {
      r3 = u.re / u.im;
      absx21 = u.im + r3 * u.re;
      u.re = r3 * R03[7] / absx21;
      u.im = (r3 * 0.0 - R03[7]) / absx21;
    }
  }

  b_asin(&u);
  theta[0].re = theta1;
  theta[0].im = 0.0;
  theta[1].re = theta2;
  theta[1].im = 0.0;
  theta[2].re = 3.1415926535897931 - x;
  theta[2].im = 0.0;
  theta[3] = u;
  theta[4] = theta5;
  theta[5].re = theta6_re;
  theta[5].im = theta6_im;
  send.servo1=theta[0].re'
  send.servo2=theta[1].re;
  send.servo3=theta[2].re;
  send.servo4=theta[3];
  send.servo5=theta[4];
  send.servo6=theta[5].re;
  
}
void callback(robotic_arm::rpose& test)
{
//receive from test, do we need type-cpnversion?
}
int main(int argc, char **argv)
{
	NodeHandle n;
  Subscriber sub = n.subscribe("arm/recieveposw", 1000, &callback) ;//subscribing from values sent by us
	Publisher pub = n.advertise<std_msgs::Float64>("arm/sendpose", 1000);//publishing values to topic to be subscibed by MCU
  Rate rate(10);
  while (ok())
	{
		
  ikine_6dof(recieve.x1,recieve.x2,recive.x3,rceieve.yaw,recieve.roll,recieve.pitch);//call function
	pub.publish(send);
   spinOnce();
	rate.sleep();
 }
	return 0;
}
