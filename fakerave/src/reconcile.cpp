#include "HuboPlus.h"
#include "HuboKin.h"

using namespace HK;
using namespace fakerave;

size_t findJoint(const KinBody& kbody, IndexArray& jidx, const char* name) {
  size_t ji = kbody.lookupJoint(name);
  jidx.push_back(ji);
  return ji;
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

int main(int argc, char** argv) {

  std::cout << "loading...\n";
  HuboPlus hplus("../myhubo.kinbody.xml");
  std::cout << "done.\n";

  HuboKin& hkin = hplus.hkin;
  const KinBody& kbody = hplus.kbody;

  RealArray jvalues(kbody.joints.size(), 0.0);
  Transform3Array xforms;
  kbody.transforms(jvalues, xforms);

  const HuboKin::KinConstants& kc = hkin.kc;

  IndexArray jidx;

  size_t pLSP = findJoint(kbody, jidx, "LSP");
  size_t pHPY = findJoint(kbody, jidx, "HPY");
  size_t pLHY = findJoint(kbody, jidx, "LHY");
  size_t pLHR = findJoint(kbody, jidx, "LHR");
  size_t pLHP = findJoint(kbody, jidx, "LHP");
  size_t pLKP = findJoint(kbody, jidx, "LKP");
  size_t pLAP = findJoint(kbody, jidx, "LAP");
  size_t pLAR = findJoint(kbody, jidx, "LAR");

  Vec3Array jpos(kbody.joints.size());

  for (size_t i=0; i<jidx.size(); ++i) {
    jpos[jidx[i]] = kbody.jointAnchor(xforms, jidx[i]);
    std::cout << "pos[" << kbody.joints[jidx[i]].name << "] = " << jpos[jidx[i]] << "\n";
  }

  vec3 fpos = kbody.manipulatorFK(xforms, 0).translation();
  std::cout << "fpos = " << fpos << "\n\n";

  std::cout << "l1 = " << jpos[pLSP].z() - jpos[pHPY].z() << "\n";
  std::cout << "kc.leg_l1 = " << kc.leg_l1 << "\n\n";

  std::cout << "l2 = " << jpos[pLHY].y() << "\n";
  std::cout << "l2 = " << jpos[pLHR].y() << "\n";
  std::cout << "l2 = " << jpos[pLAR].y() << "\n";
  std::cout << "kc.leg_l2 = " << kc.leg_l2 << "\n\n";

  std::cout << "l3 = " << jpos[pHPY].z() - jpos[pLHR].z() << "\n";
  std::cout << "l3 = " << jpos[pHPY].z() - jpos[pLHP].z() << "\n";
  std::cout << "kc.leg_l3 = " << kc.leg_l3 << "\n\n";

  std::cout << "l4 = " << jpos[pLHP].z() - jpos[pLKP].z() << "\n";
  std::cout << "l4 = " << jpos[pLHR].z() - jpos[pLKP].z() << "\n";
  std::cout << "kc.leg_l4 = " << kc.leg_l4 << "\n\n";

  std::cout << "l5 = " << jpos[pLKP].z() - jpos[pLAP].z() << "\n";
  std::cout << "l5 = " << jpos[pLKP].z() - jpos[pLAR].z() << "\n";
  std::cout << "kc.leg_l5 = " << kc.leg_l5 << "\n\n";

  std::cout << "l6 = " << jpos[pLAP].z() - fpos.z() << "\n";
  std::cout << "l6 = " << jpos[pLAR].z() - fpos.z() << "\n";
  std::cout << "kc.leg_l6 = " << kc.leg_l6 << "\n\n";

  size_t lfoot = kbody.lookupBody("Body_LAR");


  real jy = kc.leg_l2;
  real jz = kc.leg_l1 + kc.leg_l3 + kc.leg_l4 + kc.leg_l5 + kc.leg_l6;
  vec3 t0 = xforms[lfoot].transformInv(vec3(0, jy, -jz));

  real s2 = sqrt(2)/2;
  quat rot(0, s2, 0, s2);

  Transform3 relXform(rot, t0);

  std::cout << "relXform = " << relXform << "\n\n";


  // ok, now try running FK thru the dang hkin
  Isometry3d B;
  Vector6d qfk;
  qfk.setZero();

  hkin.legFK(B, qfk, HuboKin::SIDE_LEFT);
  std::cout << "legFK from zero =\n" << B.matrix() << "\n\n";

  B(0, 3) = 0.04;
  B(2, 3) = -0.85;
  std::cout << "input to legIK =\n" << B.matrix() << "\n\n";

  Vector6d qik;
  hkin.legIK(qik, B, qfk, HuboKin::SIDE_LEFT);

  std::cout << "qik = " << qik.transpose() << "\n\n";

  hkin.legFK(B, qik, HuboKin::SIDE_LEFT);
  std::cout << "legFK from qik =\n" << B.matrix() << "\n\n";

  mat2stdvec(qik, jvalues, kbody.manipulators[0].jointIndices);
  kbody.transforms(jvalues, xforms);

  Transform3 BB = kbody.manipulatorFK(xforms, 0);

  std::cout << "my FK from qik =\n" << BB.matrix() << "\n\n";

  std::cout << "my FK from qik (rotation fixed) =\n" << (BB * Transform3(hplus.footRot)).matrix() << "\n\n";
  
}

