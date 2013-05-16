#include "HuboKin.h"
#include "HuboPlus.h"
#include <stdlib.h>
#include <iostream>

#include <mzcommon/TimeUtil.h>
#include <assert.h>

using namespace HK;
using namespace fakerave;

Vector6d randAngles(const Matrix62d& limits) {
  Vector6d angles = Vector6d::Random();
  for (int i=0; i<6; ++i) {
    double lo = limits(i,0);
    double hi = limits(i,1);
    angles[i] = 0.5*(lo+hi) + angles[i]*0.5*(hi-lo);
    assert( angles[i] >= lo && angles[i] <= hi );
  }
  return angles;
}


Matrix62d getLimits(const HuboKin& hkin, int part, int side) {
  if (part == 0) {
    return hkin.kc.getArmLimits(side);
  } else {
    return hkin.kc.getLegLimits(side);
  }
}

Vector6d getOffset(const HuboKin& hkin, int part, int side) {
  if (part == 0) {
    return hkin.kc.getArmOffset(side);
  } else {
    return hkin.kc.getLegOffset(side);
  }
}

Vector6d mirror(const HuboKin& hkin, int part, int side, Vector6d angles) {
  if (side == HuboKin::SIDE_RIGHT) {
    return angles;
  } else {
    if (part == 0) {
      return HuboKin::mirrorAngles(angles, hkin.kc.arm_mirror);
    } else {
      return HuboKin::mirrorAngles(angles, hkin.kc.leg_mirror);
    }
  }
}

void fk(const HuboKin& hkin, int part, 
        Isometry3d& B, const Vector6d& q, int side) {
  if (part == 0) {
    hkin.armFK(B, q, side);
  } else {
    hkin.legFK(B, q, side);
  }
}

void ik(const HuboKin& hkin, int part,
        Vector6d& q, const Isometry3d& B, const Vector6d& qprev, int side) {
  if (part == 0) {
    hkin.armIK(q, B, qprev, side);
  } else {
    hkin.legIK(q, B, qprev, side);
  }
}


void setPart(RealArray& jvalues,
             const IndexArray& jidx,
             const Vector6d& q) {
  
  assert(jidx.size() == 6);
  for (int i=0; i<6; ++i) {
    jvalues[jidx[i]] = q[i];
  }

}

void getPart(const RealArray& jvalues,
             const IndexArray& jidx,
             Vector6d& q) {

  assert(jidx.size() == 6);
  for (int i=0; i<6; ++i) {
    q[i] = jvalues[jidx[i]];
  }

}
             
Transform3 eigen2Xform(const Isometry3d& B) {
  Eigen::Matrix4d me = B.matrix();
  mat4_t<real> mz;
  for (int i=0; i<4; ++i) {
    for (int j=0; j<4; ++j) {
      mz(i,j) = me(i,j);
    }
  }
  Transform3 rval;
  rval.setFromMatrix(mz);
  return rval;
}


struct TrialData {
  std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > qfk;
  std::vector<Isometry3d, Eigen::aligned_allocator<Isometry3d> > transforms;
  std::vector<Vector6d, Eigen::aligned_allocator<Vector6d> > qik;
};

int main(int argc, char** argv) {

  HuboKin hkin;

  int niter = 100;
  bool print = false;

  std::vector<TrialData> tdata(4);

  //////////////////////////////////////////////////////////////////////
  // construct random joint angles

  for (int part=0; part<2; ++part) {
    
    for (int side=0; side<2; ++side) {

      int tidx = 2*part + side;
      int opptidx = 2*part + (1-side);

      tdata[tidx].qfk.resize(niter);      
      tdata[tidx].qik.resize(niter);      
      tdata[tidx].transforms.resize(niter);

      for (int iter=0; iter<niter; ++iter) {

        if (side == 0) {
          Matrix62d limits = getLimits(hkin, part, HuboKin::SIDE_RIGHT);
          tdata[tidx].qfk[iter] = randAngles(limits);
        } else {
          tdata[tidx].qfk[iter] = mirror(hkin, part, side, tdata[opptidx].qfk[iter]);
        }

      }
        
    }

  }

  //////////////////////////////////////////////////////////////////////
  // do forward kinematics

  for (int part=0; part<2; ++part) {
    const char* partname = ( (part == 0) ? "arm" : "leg" );
    TimeStamp start = TimeStamp::now();
    for (int side=0; side<2; ++side) {
      int tidx = 2*part + side;
      for (int iter=0; iter<niter; ++iter) {
        const Vector6d& qfk = tdata[tidx].qfk[iter];
        Isometry3d& B = tdata[tidx].transforms[iter];
        fk(hkin, part, B, qfk, side);
      }
    }
    double elapsed = (TimeStamp::now() - start).toDouble();
    std::cerr << "computed " << 2*niter << " FK's for " << partname << " "
              << "in " << elapsed << " sec "
              << "(" << (elapsed/(2*niter)) << " per)\n";
  }

  //////////////////////////////////////////////////////////////////////
  // do inverse kinematics

  for (int part=0; part<2; ++part) { 

    const char* partname = ( (part == 0) ? "arm" : "leg" );

    TimeStamp start = TimeStamp::now();

    for (int iter=0; iter<niter; ++iter) {

      for (int side=0; side<2; ++side) {

        int tidx = 2*part + side;

        const Isometry3d& B = tdata[tidx].transforms[iter];
        Vector6d& qik = tdata[tidx].qik[iter];
        const Vector6d& qfk = tdata[tidx].qfk[iter];

        ik(hkin, part, qik, B, qfk, side);
    

      }
      
    }

    double elapsed = (TimeStamp::now() - start).toDouble();

    std::cerr << "computed " << 2*niter << " IK's for " << partname << " "
              << "in " << elapsed << " sec "
              << "(" << (elapsed/(2*niter)) << " per)\n";

  }

  //////////////////////////////////////////////////////////////////////
  // verify inverse kinematics


  for (int part=0; part<2; ++part) { 

    const char* partname = ( (part == 0) ? "arm" : "leg" );
    double totalMaxErr = 0;

    for (int iter=0; iter<niter; ++iter) {

      for (int side=0; side<2; ++side) {

        const char* sidename = ( (side == HuboKin::SIDE_RIGHT) ? "right" : " left" );

        int tidx = 2*part + side;

        const Vector6d& qik = tdata[tidx].qik[iter];
        const Vector6d& qfk = tdata[tidx].qfk[iter];

        Vector6d diff = qfk-qik;

        double maxErr = diff.lpNorm<Eigen::Infinity>();

        if (print) {
          const Isometry3d& B = tdata[tidx].transforms[iter];
          Eigen::Vector3d pos = B.matrix().block(0, 3, 3, 1);
          Eigen::Matrix3d R =  B.matrix().block(0, 0, 3, 3);
          Eigen::Quaterniond rot(R);
          std::cout << sidename << " " << partname << ": "
                    << "qfk = " << qfk.transpose() << ", "
                    << "pos = " << pos.transpose() << ", "
                    << "rot = " << rot.vec().transpose() << ", " << rot.w() << ", "
                    << "qik = " << qik.transpose() << ", "
                    << "err = " << maxErr << "\n";
        }

        assert(maxErr < 1e-4);
        totalMaxErr += maxErr;

      }
      
    }

    std::cerr << "avg. max. err for " << partname << " "
              << "is " << (totalMaxErr/(2*niter)) << "\n";

  }

  return 0;

}
