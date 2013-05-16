/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2013 Matt Zucker. All rights reserved. */
/********************************************************/

#include "HuboPlus.h"
#include <assert.h>
#include <mzcommon/TimeUtil.h>
#include <mzcommon/mersenne.h>

using namespace fakerave;
using namespace HK;

template <class Tval, class Talloc>
std::ostream& operator<<(std::ostream& ostr, 
                         const std::vector<Tval, Talloc>& idx) {
  for (size_t i=0; i<idx.size(); ++i) {
    ostr << (i ? " " : "") << idx[i];
  }
  return ostr;
}

int main(int argc, char** argv) {
  
  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " HUBOFILE.xml\n";
    return 1;
  }

  HuboPlus hplus(argv[1]);
  KinBody& kbody = hplus.kbody;
  const JointLookup& jl = hplus.jl;

  fakerave::Transform3Array xforms;
  fakerave::RealArray jvalues(kbody.joints.size(), 0.0);


  const real deg = M_PI / 180;

  jvalues[ jl("RHP") ] = -20 * deg;
  jvalues[ jl("RKP") ] = 40 * deg;
  jvalues[ jl("RAP") ] = -20 * deg;

  jvalues[ jl("LHP") ] = -20 * deg;
  jvalues[ jl("LKP") ] = 40 * deg;
  jvalues[ jl("LAP") ] = -20 * deg;

  jvalues[ jl("RSP") ] = 20 * deg;
  jvalues[ jl("REP") ] = -40 * deg;
  jvalues[ jl("RWP") ] = 20 * deg;

  jvalues[ jl("LSP") ] = 20 * deg;
  jvalues[ jl("LEP") ] = -40 * deg;
  jvalues[ jl("LWP") ] = 20 * deg;

  kbody.transforms(jvalues, xforms);


  RealArray jinit = jvalues;

  for (int manip=0; manip<4; ++manip) {

    const Manipulator& m = kbody.manipulators[manip];
    const IndexArray& jidx = m.jointIndices;

    MatX limits(jidx.size(), 2);


    for (size_t i=0; i<jidx.size(); ++i) {
      size_t ji = jidx[i];
      const Joint& j = kbody.joints[ji];
      real& lo = limits(i,0);
      real& hi = limits(i,1);
      if (j.limits[0] < j.limits[1]) {
        lo = j.limits[0];
        hi = j.limits[1];
      } else {
        lo = -M_PI/2;
        hi = M_PI/2;
      }
    }



    MatX Jt;

    kbody.transforms(jvalues, xforms, &m.activeBodies);
    kbody.manipulatorJacobian(xforms, manip, Jt);

    MatX Jnt;

    kbody.manipulatorNumericalJacobian(manip, jvalues, xforms, Jnt);

    std::cout << "\n**************************************************\n";
    std::cout << "Manipulator " << manip << " named " << m.name << "\n\n";
    //std::cout << "Limits:\n" << limits << "\n\n";
    //std::cout << "Jacobian transpose:\n" << Jt << "\n\n";
    //std::cout << "Numerical jacobian transpose:\n" << Jnt << "\n\n";
    //std::cout << "Jacobian error:\n" << Jnt-Jt << "\n\n";
    std::cout << "Jacobian max error: " << (Jnt-Jt).lpNorm<Eigen::Infinity>() << "\n";

    assert( (Jnt - Jt).lpNorm<Eigen::Infinity>() < 1e-5 );

    {

      // try to bring end effector closer
      Transform3 ik = kbody.manipulatorFK(xforms, manip);
      vec3 v = ik.translation();
      if (manip < 2) { v.z() = -0.85; } else { v.z() += 0.01; }
      ik.setTranslation(v);

      kbody.centerJoints(jidx, jvalues);
      bool ok = kbody.manipulatorIK(manip, ik, jvalues, xforms);
      std::cout << "IK to go to z=" << v.z() << " returned " << ok << "\n";

    }

    // now try to ascertain coverage of workspace
    IndexArray ndiv;
    MatX angles;

    size_t total = 1;
    for (size_t i=0; i<jidx.size(); ++i) {
      ndiv.push_back(i < 3 ? 5 : 5);
      total *= ndiv.back();
    }

    IndexArray cur(jidx.size(), 0);
    bool done = false;
    size_t count = 0;
    MatX jfk(total, jidx.size());
    MatX jik(total, jidx.size());
    Transform3Array fk(total);
    BoolArray ok(total);

    while (!done) {

      // do stuff with the current
      for (size_t i=0; i<jidx.size(); ++i) {
        real u = real(cur[i]+0.5) / real(ndiv[i]);
        jfk(count,i) = limits(i,0) + u*(limits(i,1) - limits(i,0));
      }

      ++count;
      size_t place=0;
      while (place < cur.size()) {
        ++cur[place];
        if (cur[place] >= ndiv[place]) {
          cur[place] = 0;
          ++place;
          if (place >= cur.size()) {
            done = true;
          }
        } else {
          place = -1;
        }
      }

    }

    assert(count == total);
    
    TimeStamp start = TimeStamp::now();

    for (size_t iter=0; iter<total; ++iter) {
      mat2stdvec(jfk.row(iter), jvalues, jidx);
      kbody.transforms(jvalues, xforms, &m.activeBodies);
      fk[iter] = kbody.manipulatorFK(xforms, manip);
    }

    double elapsed = (TimeStamp::now() - start).toDouble();
    std::cout << "Computed " << total << " FK's in " << elapsed << " sec "
              << "(" << (elapsed/total) << " s per)\n";

    
    start = TimeStamp::now();
    size_t numok = 0;
    for (size_t iter=0; iter<total; ++iter) {

      Vector6d qprev = jfk.row(iter);
      for (int i=0; i<6; ++i) { 
        qprev[i] += (mt_genrand_real1()*2-1) * 0.01;
      }
      mat2stdvec(qprev, jvalues, jidx);
      ok[iter] = hplus.manipIK(manip, fk[iter], jvalues, xforms);
      stdvec2mat(jvalues, jik.row(iter), jidx);

      if (ok[iter]) { ++numok; }
    }

    
    elapsed = (TimeStamp::now() - start).toDouble();
    std::cout << "Computed " << total << " IK's in " << elapsed << " sec "
              << "(" << (elapsed/total) << " s per), "
              << real(100*numok)/total << "% success rate.\n";

    start = TimeStamp::now();
    size_t numverified = 0;
    for (size_t iter=0; iter<total; ++iter) {

      if (ok[iter]) {
        real maxerr = (jfk.row(iter) - jik.row(iter)).lpNorm<Eigen::Infinity>();
        if (maxerr < 1e-3) { ++numverified; }
      }

    }

    elapsed = (TimeStamp::now() - start).toDouble();
    std::cout << "Verified " << real(100*numverified)/numok << "%.\n";

    
    

  }


  return 0;

}
