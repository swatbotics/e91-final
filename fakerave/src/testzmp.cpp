#include "ZmpPreview.h"
#include <math.h>
#include <iostream>

int main(int argc, char** argv) {

  size_t ctrlFreq = 200;
  double T = 1.0/ctrlFreq;
  double h = 0.5;
  size_t nl = (size_t)round(2.5/T);
  double R = 1e-9;
 
  ZmpPreview preview(T, h, nl, R);


  size_t numPhases = 10;
  size_t phaseTicks = size_t(round(1.0/T));
  size_t totalTicks = phaseTicks * numPhases;

  double sway = 0.08;

  Eigen::ArrayXd zmpref(totalTicks);

  for (size_t i=0; i<numPhases; ++i) {
    double s;
    if (i < 2 || i+2 >= numPhases) {
      s = 0;
    } else if (i % 2) {
      s = -1;
    } else {
      s = 1;
    }
    zmpref.block(i*phaseTicks, 0, phaseTicks, 1).setConstant(s*sway);
  }


  Eigen::Vector3d X(0.0,0.0,0.0);

  double e = 0;
  double p = 0;

  
  Eigen::ArrayXd com(totalTicks);
  Eigen::ArrayXd zmp(totalTicks);

  for (size_t i=0; i<totalTicks; ++i) {
    com(i) = X(0);
    zmp(i) = p;
    p = preview.update(X, e, zmpref.block(i, 0, totalTicks-i, 1));
  }


  
  for (size_t i=0; i<totalTicks; ++i) {
    std::cout << T*i << ", "
	      << zmpref(i) << ", "
	      << com(i) << ", "
	      << zmp(i) << "\n";
  }

  return 0;

}
