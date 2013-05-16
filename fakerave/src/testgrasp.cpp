/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "fakerave.h"
using namespace fr;

Mat v2c(const vec3& v) {
  return Mat(3,1) << v[0],v[1],v[2];
}

vec3 c2v(const Mat& v) {
  return vec3(v(0), v(1), v(2));
}

quat fk1(const real x[3], Mat* pJ=0) {

  const vec3 a0(1,0,0);
  const vec3 a1(0,1,0);
  const vec3 a2(1,0,0);

  quat q0 = quat::fromAxisAngle(a0, x[0]);
  quat q1 = quat::fromAxisAngle(a1, x[1]);
  quat q2 = quat::fromAxisAngle(a2, x[2]);

  if (pJ) {

    Mat& J = *pJ;
    if (J.rows != 3 || J.cols != 3) { J = Mat(3,3); }

    J.col(0) = 1*v2c(a0);
    J.col(1) = 1*v2c(q0.toMat3() * a1);
    J.col(2) = 1*v2c((q0*q1).toMat3()*a2);

  }

  return q0 * q1 * q2;

}

quat fk2(const real y[2], Mat* pJ=0) {

  const vec3 a0(0,0,1);
  const vec3 a1(0,1,0);

  quat q0 = quat::fromAxisAngle(a0, y[0]);
  quat q1 = quat::fromAxisAngle(a1, y[1]);

  if (pJ) {

    Mat& J = *pJ;
    if (J.rows != 3 || J.cols != 2) { J = Mat(3,2); }

    J.col(0) = 1*v2c(a0);
    J.col(1) = 1*v2c(q0.toMat3() * a1);

  }

  return q0 * q1;

}

//////////////////////////////////////////////////

vec3 zaxis1(const real x[3], Mat* pJ=0) {

  const vec3 z(0,0,1);

  quat q = fk1(x, pJ);

  mat3 R = q.toMat3();


  if (pJ) {

    std::cerr << "R =\n" << R << "\n";

    Mat& J = *pJ;
    for (int c=0; c<3; ++c) {
      vec3 jr = c2v(J.col(c));
      J.col(c) = 1*v2c( vec3::cross(-R*z,jr) );
    }
  }

  return R * z;

}         

//////////////////////////////////////////////////



//////////////////////////////////////////////////

int main(int argc, char** argv) {

  real x[3] = { M_PI/4, M_PI/3, -M_PI/6 };
  Mat J1, Jn1(3,3);
  
  quat q1 = fk1(x, &J1);

  std::cout << "q1 = " << q1 << "\n";
  std::cout << "J1 =\n" << J1 << "\n";

  real h = 1e-3;

  for (int c=0; c<3; ++c) {
    real xc = x[c];
    x[c] = xc-h;
    quat f0 = fk1(x);
    x[c] = xc+h;
    quat f1 = fk1(x);
    x[c] = xc;
    vec3 d = quat::omega(f0,f1)/(2*h);
    for (int r=0; r<3; ++r) {
      Jn1(r,c) = d[r];
    }
  }

  std::cout << "Jn1 =\n" << Jn1 << "\n";
  std::cout << "err =\n" << Jn1-J1 << "\n";

  //////////////////////////////////////////////////

  Mat Jz1, Jnz1(3,3);
  vec3 z1 = zaxis1(x, &Jz1);

  std::cout << "z1 = " << z1 << " (norm=" << z1.norm() << ")\n";
  std::cout << "Jz1 =\n" << Jz1 << "\n";

  for (int c=0; c<3; ++c) {
    real xc = x[c];
    x[c] = xc-h;
    vec3 fz0 = zaxis1(x);
    x[c] = xc+h;
    vec3 fz1 = zaxis1(x);
    x[c] = xc;
    vec3 d = (fz1-fz0)/(2*h);
    for (int r=0; r<3; ++r) {
      Jnz1(r,c) = d[r];
    }
  }

  std::cout << "Jnz1 =\n" << Jnz1 << "\n";
  std::cout << "err =\n" << Jz1 - Jnz1 << "\n";

  return 0;

  //////////////////////////////////////////////////

  real y[2] = { 0.6, -0.4 };
  Mat J2, Jn2(3,2);
  
  quat q2 = fk2(y, &J2);

  std::cout << "q2 = " << q2 << "\n";
  std::cout << "J2 =\n" << J2 << "\n";

  for (int c=0; c<2; ++c) {
    real yc = y[c];
    y[c] = yc-h;
    quat f0 = fk2(y);
    y[c] = yc+h;
    quat f1 = fk2(y);
    y[c] = yc;
    vec3 d = quat::omega(f0,f1)/(2*h);
    for (int r=0; r<3; ++r) {
      Jn2(r,c) = d[r];
    }
  }

  std::cout << "Jn2 =\n" << Jn2 << "\n";
  std::cout << "err =\n" << Jn2-J2 << "\n";


  return 0;

}
