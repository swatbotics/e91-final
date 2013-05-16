#include "HuboPlus.h"
#include "HuboKin.h"
#include <mzcommon/TimeUtil.h>

using namespace fakerave;
using namespace HK;


void baseDofs(IndexArray& dofs) {

  for (size_t i=DOF_POS_X; i<=DOF_ROT_Z; ++i) {
    dofs.push_back(i);
  }

}


int buildDOFs(const HuboPlus& hplus,
              const HuboPlus::IKMode mode[4],
              IndexArray& dofs) {

  baseDofs(dofs);

  const KinBody& kbody = hplus.kbody;
  int cnt = 0;

  for (size_t m=0; m<4; ++m) {
    if (mode[m] == HuboPlus::IK_MODE_SUPPORT ||
        mode[m] == HuboPlus::IK_MODE_WORLD) {
      const IndexArray& jidx = kbody.manipulators[m].jointIndices;
      for (size_t i=0; i<jidx.size(); ++i) {
        dofs.push_back(jidx[i]);
      }
      ++cnt;
    }
  }

  return cnt;

}

void getBigJacobian(const HuboPlus& hplus, 
                    const Transform3Array& xforms,
                    const HuboPlus::IKMode mode[4],
                    const IndexArray& dofs,
                    MatX& Jt,
                    MatX& Jmt) {

  const KinBody& kbody = hplus.kbody;

  int ndofs = dofs.size();

  int nmanips = 0;

  for (size_t m=0; m<4; ++m) {
    if (mode[m] == HuboPlus::IK_MODE_SUPPORT ||
        mode[m] == HuboPlus::IK_MODE_WORLD) {
      ++nmanips;
    }
  }

  if (Jt.rows() != ndofs || Jt.cols() != 6*nmanips) {
    Jt = MatX(ndofs, 6*nmanips);
  }

  int cnt = 0;

  for (int m=0; m<4; ++m) {
    if (mode[m] == HuboPlus::IK_MODE_SUPPORT ||
        mode[m] == HuboPlus::IK_MODE_WORLD) {
      kbody.manipulatorJacobian(xforms, m, dofs, Jmt);
      assert(Jmt.rows() == ndofs);
      assert(Jmt.cols() == 6);
      Jt.block(0, cnt*6, ndofs, 6) = Jmt;
      ++cnt;
    }
  }
  
  assert(cnt == nmanips);

}

void getFKError(const HuboPlus& hplus,
                const Transform3& baseXform,
                const Transform3Array& xforms,
                const HuboPlus::IKMode mode[4],
                const Transform3 manipXforms[4],
                MatX& f) {

  int nmanips = 0;

  for (size_t m=0; m<4; ++m) {
    if (mode[m] == HuboPlus::IK_MODE_SUPPORT ||
        mode[m] == HuboPlus::IK_MODE_WORLD) {
      ++nmanips;
    }
  }

  if (f.rows() != nmanips*6 || f.cols() != 1) { 
    f = MatX(nmanips*6, 1);
  }


  int cnt = 0;

  for (int m=0; m<4; ++m) {

    if (mode[m] == HuboPlus::IK_MODE_SUPPORT ||
        mode[m] == HuboPlus::IK_MODE_WORLD) {
      
      Transform3 desired = baseXform.inverse() * manipXforms[m];
      Transform3 fk = hplus.kbody.manipulatorFK(xforms, m);

      vec3 dp, dq;

      deltaTransform(fk, desired, dp, dq);

      for (int a=0; a<3; ++a) {
        f(6*cnt + 0 + a) = dp[a];
        f(6*cnt + 3 + a) = dq[a];
      }

      ++cnt;

    }

  }

  assert(cnt == nmanips);

}

vec3 getComError(const HuboPlus& hplus,
                 const Transform3& baseXform,
                 const Transform3Array& xforms,
                 const vec3& desiredCOM) {
  
  vec3 actualCOM = baseXform * hplus.kbody.com(xforms);
  return actualCOM - desiredCOM;

}

void applyDelta(const IndexArray& dofs,
                const MatX& delta,
                HuboPlus::KState& state) {

  assert(isVecOfLength(delta, dofs.size()));
  
  // get out a dq and dp
  vec3 dp(0), dq(0);
  
  for (size_t i=0; i<dofs.size(); ++i) {
    size_t di = dofs[i];
    if (di>=DOF_POS_X && di<=DOF_POS_Z) { 
      dp[di-DOF_POS_X] = delta(i); 
    } else if (di>=DOF_ROT_X && di<=DOF_ROT_Z) {
      dq[di-DOF_ROT_X] = delta(i);
    } else {
      assert(di < state.jvalues.size());
      state.jvalues[di] += delta(i);
    }
  }
  
  state.body_pos += dp;
  state.body_rot = quat::fromOmega(dq) * state.body_rot;

}

void testJacobians(const HuboPlus& hplus,
                   const HuboPlus::KState& state,
                   const vec3& desiredCOM,
                   const Transform3 manipXforms[4],
                   const HuboPlus::IKMode mode[4],
                   Transform3Array& xforms) {

  const KinBody& kbody = hplus.kbody;

  kbody.transforms(state.jvalues, xforms);

  IndexArray dofs;
  int nm = buildDOFs(hplus, mode, dofs);
  int nd = dofs.size();

  MatX FT, FmT, GT;

  getBigJacobian(hplus, xforms, mode, dofs, FT, FmT);

  kbody.comJacobian(xforms, dofs, GT);

  MatX delta(dofs.size(), 1);

  delta.setZero();

  // TODO: deal
  HuboPlus::KState dummy;
  real h = 1e-4;
  int d[2] = { -1, 1 };

  MatX f[2], NFT(nd, 6*nm), NGT(nd, 3);
  vec3 g[2];

  
  for (int i=0; i<nd; ++i) {

    for (int a=0; a<2; ++a) {

      dummy = state;

      delta(i) = h * d[a];

      applyDelta(dofs, delta, dummy);
      kbody.transforms(dummy.jvalues, xforms);

      getFKError(hplus, dummy.xform(), xforms, mode, manipXforms, f[a]);
      assert(isVecOfLength(f[a], 6*nm));

      g[a] = getComError(hplus, dummy.xform(), xforms, desiredCOM);


      delta(i) = 0;

    }

    for (int j=0; j<6*nm; ++j) {
      NFT(i,j) = (f[1](j) - f[0](j)) / (2*h);
    }

    for (int a=0; a<3; ++a) {
      NGT(i,a) = (g[1][a] - g[0][a]) / (2*h);
    }

  }

  //std::cout << "FT:\n" << FT << "\n\n";
  //std::cout << "NFT:\n" << NFT << "\n\n";
  //std::cerr << "Error:\n" << (NFT-FT) << "\n\n";
  std::cerr << "F Max error: " << (NFT-FT).lpNorm<Eigen::Infinity>() << "\n\n";

  //std::cout << "GT:\n" << GT << "\n\n";
  //std::cout << "NGT:\n" << NGT << "\n\n";
  //std::cerr << "Error:\n" << (NGT-GT) << "\n\n";
  std::cerr << "G Max error: " << (NGT-GT).lpNorm<Eigen::Infinity>() << "\n\n";


}


bool dumbComIK(const HuboPlus& hplus,
               HuboPlus::KState& state,
               const vec3& desiredCOM,
               const Transform3 manipXforms[4],
               const HuboPlus::IKMode mode[4],
               Transform3Array& xforms,
               real ascl) {



  IndexArray dofs;


  const KinBody& kbody = hplus.kbody;


  //std::cout << "got " << dofs.size() << " dofs:";
  //for (size_t i=0; i<dofs.size(); ++i) { std::cout << " " << dofs[i]; }
  //std::cout << "\n";


  int nmanips = buildDOFs(hplus, mode, dofs);
  int ndofs = dofs.size();
  

  MatX GT, FmT, FT(ndofs, nmanips*6);
  MatX A, P, Q, R, S;

  MatX f(12, 1);
  MatX g(3, 1);
  MatX delta(ndofs,1);

  delta.setZero();


  Eigen::LDLT<MatX> solveA(18);
  Eigen::LDLT<MatX> solveQ(nmanips*6);

  //A = 0.9 * MatX::Identity(ndofs, ndofs);
  //solveA.compute(A);

  real lambda = 0.1;
  bool ok = false;

  size_t iter;

  for (iter=0; iter < 2000; ++iter) {

    kbody.transforms(state.jvalues, xforms);

    getBigJacobian(hplus, xforms, mode, dofs, FT, FmT);
    kbody.comJacobian(xforms, dofs, GT);

    GT.block(3, 0, 3, 3)  *= ascl;
    FT.block(3, 0, 3, 12) *= ascl;

    Transform3 xform = state.xform();
    getFKError(hplus, xform, xforms, mode, manipXforms, f);

    vec3 err = getComError(hplus, xform, xforms, desiredCOM);

    if (err.norm() < hplus.DEFAULT_COM_PTOL) {
      ok = true;
      break;
    }
    
    g << err[0], err[1], err[2];

    //std::cerr << "f.norm(): " << f.norm() << ", g.norm(): " << g.norm() << "\n";

    A = GT * GT.transpose() + lambda * MatX::Identity(ndofs, ndofs);
    solveA.compute(A);

    P = solveA.solve(FT);
    Q = FT.transpose() * P;
    solveQ.compute(Q);
    R = solveQ.solve(P.transpose());

    S = solveA.solve(MatX::Identity(ndofs, ndofs) - FT * R);

    delta = -S*GT*g - R.transpose()*f;

    applyDelta(dofs, delta, state);


  }

  //std::cerr << "ok = " << ok << " after " << iter << " iterations.\n";

  // clip to joint limits
  kbody.clampToLimits(state.jvalues);

  kbody.transforms(state.jvalues, xforms);

  Transform3 xform = state.xform();
  
  //std::cerr << "ok = " << ok << "\n";


  for (int foot=0; ok && foot<2; ++foot) {
    Transform3 desired = xform.inverse() * manipXforms[foot];
    Transform3 fk = kbody.manipulatorFK(xforms, foot);
    vec3 dp, dq;
    deltaTransform(desired, fk, dp, dq);
    if (dp.norm() > 5e-4 || dq.norm() > 1e-5) {
      ok = false;
    }
  }

  //std::cerr << "ok = " << ok << "\n";
    

  return ok;

}

bool smartComIK(const HuboPlus& hplus,
                HuboPlus::KState& state,
                const vec3& desiredCOM,
                const Transform3 manipXforms[4],
                const HuboPlus::IKMode mode[4],
                Transform3Array& xforms,
                real ascl) {

  IndexArray dofs;

  const KinBody& kbody = hplus.kbody;

  baseDofs(dofs);

  MatX GT;

  MatX g(3, 1);
  MatX delta(6, 1);
  delta.setZero();

  MatX fpT, fxT, xp;

  size_t iter;
  for (iter=0; iter < 2000; ++iter) {

    // Try to verify xp for each thing
    for (int m=0; m<4; ++m) {

      if (mode[m] == HuboPlus::IK_MODE_SUPPORT ||
          mode[m] == HuboPlus::IK_MODE_WORLD) {

        kbody.transforms(state.jvalues, xforms);
        kbody.manipulatorJacobian(xforms, m, dofs, fpT);
        kbody.manipulatorJacobian(xforms, m, fxT);

        xp = -fxT.transpose().colPivHouseholderQr().solve(fpT.transpose());


        real h = 1e-3;
        int d[2] = { -1, 1 };
        Vector6d q[2];
        MatX Nxp(6,6);

        HuboPlus::KState dummy;

        for (int i=0; i<6; ++i) {

          for (int a=0; a<2; ++a) {
            
            dummy = state;
            
            delta(i) = h * d[a];

            applyDelta(dofs, delta, dummy);
            kbody.transforms(dummy.jvalues, xforms);

            Transform3 desired = dummy.xform().inverse() * manipXforms[m];

            hplus.manipIK(m, desired, dummy.jvalues, xforms, true);
            
            const IndexArray& jidx = kbody.manipulators[m].jointIndices;
            assert(jidx.size() == 6);

            for (int j=0; j<6; ++j) {
              q[a](j) = dummy.jvalues[jidx[j]];
            }

            delta(i) = 0;
            
          }

          for (int j=0; j<6; ++j) {
            Nxp(j,i) = (q[1](j) - q[0](j))/(2*h);
          }

        }

        std::cout << "xp =\n" << xp << "\n\n";
        std::cout << "Nxp =\n" << Nxp << "\n\n";

      }
      
    }

    exit(0);

  }

  return false;

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
  
  HuboPlus::KState state;

  state.jvalues.resize(kbody.joints.size(), 0.0);

  const real deg = M_PI / 180;

  state.jvalues[ jl("RHP") ] = -20 * deg;
  state.jvalues[ jl("RKP") ] = 40 * deg;
  state.jvalues[ jl("RAP") ] = -20 * deg;

  state.jvalues[ jl("LHP") ] = -20 * deg;
  state.jvalues[ jl("LKP") ] = 40 * deg;
  state.jvalues[ jl("LAP") ] = -20 * deg;

  state.jvalues[ jl("RSP") ] = 20 * deg;
  state.jvalues[ jl("REP") ] = -40 * deg;
  state.jvalues[ jl("RWP") ] = 20 * deg;

  state.jvalues[ jl("LSP") ] = 20 * deg;
  state.jvalues[ jl("LEP") ] = -40 * deg;
  state.jvalues[ jl("LWP") ] = 20 * deg;

  kbody.transforms(state.jvalues, xforms);

  // place the feet on the ground
  Transform3 fk = kbody.manipulatorFK(xforms, HuboPlus::MANIP_L_FOOT);

  state.body_pos.z() -= fk.translation().z();

  std::cout << "body at " << state.body_pos << "\n";
  
  vec3 actualCOM = state.xform() * kbody.com(xforms);

  std::cout << "com at " << actualCOM << "\n";

  Transform3 manipXforms[4];

  HuboPlus::IKMode mode[4] = { 
    HuboPlus::IK_MODE_SUPPORT,
    HuboPlus::IK_MODE_SUPPORT,
    HuboPlus::IK_MODE_FIXED,
    HuboPlus::IK_MODE_FIXED,
  };

  for (int foot=0; foot<2; ++foot) {
    manipXforms[foot] = state.xform() * kbody.manipulatorFK(xforms, foot);
  }

  vec3 desiredCOM = actualCOM + vec3(0.02, 0.04, -0.05);

  testJacobians( hplus, state, desiredCOM, manipXforms, mode, xforms );
  
  HuboPlus::KState newstate = state;
  bool ok;

  real ascl = 1;

  size_t niter = 100;

  TimeStamp start;
  double elapsed;

  start = TimeStamp::now();

  for (size_t i=0; i<niter; ++i) {
    
    newstate = state;

    ok = smartComIK(hplus, newstate, desiredCOM, manipXforms, mode, xforms, ascl);


  }

  elapsed = (TimeStamp::now() - start).toDouble();
  std::cout << "ran " << niter << " trials in " << elapsed << " s (" << (elapsed/niter) << " s/per)\n";
  
  std::cout << "ok = " << ok << "\n";
  std::cout << "newstate.body_rot = " << newstate.body_rot << "\n";

  start = TimeStamp::now();
  
  for (size_t i=0; i<niter; ++i) {
    
    newstate = state;
    ok = dumbComIK(hplus, newstate, desiredCOM, manipXforms, mode, xforms, ascl);

  }

  elapsed = (TimeStamp::now() - start).toDouble();
  std::cout << "ran " << niter << " trials in " << elapsed << " s (" << (elapsed/niter) << " s/per)\n";
    
  std::cout << "ok = " << ok << "\n";
  std::cout << "newstate.body_rot = " << newstate.body_rot << "\n";
  


  return 0;

}
