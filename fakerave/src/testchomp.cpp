/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "chomp.h"
#include <assert.h>
#include <iostream>
#include <util/TimeUtil.h>

typedef cv::Mat_<double> Mat;

void regularChol(const Mat& A, Mat& L) {

  int n = A.rows;
  L = Mat::zeros(n,n);
  
  for (int j=0; j<n; ++j) {
    for (int i=j; i<n; ++i) {
      double sum = 0;
      for (int k=0; k<j; ++k) { 
        sum += L(i,k) * L(j,k); 
      }
      if (i == j) {
        L(j,j) = sqrt(A(j,j) - sum);
      } else {
        L(i,j) = ( A(i,j) - sum ) / L(j,j);
      }
    }
  }
 

}


// Ax = b
//   L L^T x = b
// multiply both sides by L^-1
//   L^T x = L^-1 b
// then multiply both sides by L^-T
//  x = L^-T L^-1 b

void regularCholSolve(const Mat& L, Mat& x) {

  assert( L.rows == L.cols );

  int n = L.rows;
  
  assert(x.rows == n);

#if 0

  for (int c=0; c<x.cols; ++c) {

    for (int i=0; i<n; ++i) {
      for (int j=0; j<i; ++j) {
        x(i,c) -= L(i,j)*x(j,c); // here j < i so col < row
      }
      x(i,c) /= L(i,i);
    }

    for (int i=n-1; i>=0; --i) {
      for (int j=i+1; j<n; ++j) {
        x(i,c) -= L(j,i) * x(j,c); // here j > i so col < row
      }
      x(i,c) /= L(i,i);
    }

  }

#else

  for (int i=0; i<n; ++i) {
    for (int j=0; j<i; ++j) {
      x.row(i) -= L(i,j)*x.row(j); // here j < i so col < row
    }
    x.row(i) /= L(i,i);
  }

  for (int i=n-1; i>=0; --i) {
    for (int j=i+1; j<n; ++j) {
      x.row(i) -= L(j,i) * x.row(j); // here j > i so col < row
    }
    x.row(i) /= L(i,i);
  }

#endif

}

int main(int argc, char** argv) { 

  int n = 100;

  if (argc > 1) { 
    int nn = atoi(argv[1]);
    if (nn) { n = nn; }
  }

  Mat A = Mat::zeros(n,n);
  for (int i=0; i<n; ++i) {
    if (i+2 < n) { A(i,i+2) = 1; }
    if (i+1 < n) { A(i,i+1) = -4; }
    A(i,i) = 6;
    if (i > 0) { A(i,i-1) = -4; }
    if (i > 1) { A(i,i-2) = 1; }
  }


  Mat L, Ls;

  Mat coeffs = (Mat(3,1) << 1, -4, 6);
  //Mat coeffs = (Mat(2,1) << -1, 2);

  Mat m1 = Mat::eye(n,n);
  Mat m2 = Mat::eye(n,n);
  Mat m3 = Mat::eye(n,n);
  Mat m4 = Mat::eye(n,n);
  Mat work;

  TimeStamp t0 = TimeStamp::now();

  regularChol(A, L);

  TimeStamp t1 = TimeStamp::now();

  regularCholSolve(L, m1);

  TimeStamp t2 = TimeStamp::now();

  skylineChol(n, coeffs, Ls);

  TimeStamp t3 = TimeStamp::now();
  
  skylineCholSolve(Ls, m2);

  TimeStamp t4 = TimeStamp::now();

  diagMul(coeffs, m4, m3);

  TimeStamp t5 = TimeStamp::now();
  
  Mat b;
  createBVector(n, coeffs, -1.0, 1.0, b);

  double d1 = (t1-t0).toDouble();
  double d2 = (t2-t1).toDouble();
  double d3 = (t3-t2).toDouble();
  double d4 = (t4-t3).toDouble();
  double d5 = (t5-t4).toDouble();

  if (n < 20) { 
    std::cout << "A =\n" << A << "\n";
    std::cout << "A =\n" << m3 << "\n";
    std::cout << "A.inv() =\n" << A.inv() << "\n";
    std::cout << "m1 =\n" << m1 << "\n";
    std::cout << "errm1 = \n" << m1-A.inv() << "\n";
    std::cout << "m2 =\n" << m2 << "\n";
    std::cout << "errm2 = \n" << m2-A.inv() << "\n";
  }

  std::cout << "with n=" << n << ", regular cholesky decomp took " << d1 << "s.\n";
  std::cout << "with n=" << n << ", regular cholesky solve took " << d2 << "s.\n";
  std::cout << "with n=" << n << ", band cholesky decomp took " << d3 << "s. (" << 100*d3/d1 << "% of regular)\n";
  std::cout << "with n=" << n << ", band cholesky solve took " << d4 << "s. (" << 100*d4/d2 << "% of regular)\n";
  std::cout << "with n=" << n << ", band multiply took " << d5 << ".s\n";

  return 0;


}
