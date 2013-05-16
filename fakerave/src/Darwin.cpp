#include "Darwin.h"

using namespace fakerave;

  enum IKMode {
    IK_MODE_FREE,    // you can do whatever you want to these joint angles
    IK_MODE_FIXED,   // joint angles already specified, do not mess with them
    IK_MODE_BODY,    // manipulator specified relative to body
    IK_MODE_WORLD,   // manipulator specified relative to world
    IK_MODE_SUPPORT, // manipulator specfied relative to world, and holding up robot
  };
  

const char* mnames[4] = {
  "leftFootManip",
  "rightFootManip",
  "leftHandManip",
  "rightHandManip",
};

std::string flipy(const std::string& name) {
  size_t idx = name.find('L');
  if (idx != std::string::npos) {
    std::string n = name;
    n[idx] = 'R';
    return n;
  } else {
    return name;
  }
}

Transform3 flipy(const Transform3& t) {

  mat4_t<real> m = mat4_t<real>::identity();
  m(1,1) = -1;

  return Transform3(m * t.matrix() * m);

}

vec3 flipy(const vec3& v) {
  return vec3(v.x(), -v.y(), v.z());
}

void compare(const vec3& l, const vec3& r) {
  assert( (l - r).norm() < 1e-8 );
}

void compare(const Transform3& l, const Transform3& r) {
  vec3 dp, dq;
  deltaTransform(l, r, dp, dq);
  assert( dp.norm() < 1e-8 && dq.norm() < 1e-8 );
}

void ensureMirror(const Joint& jl, const Joint& jr) {

  vec3 al = jl.anchor;
  vec3 ar = jr.anchor;

  compare(al, flipy(ar));
  compare(jl.axis, jr.axis);

  if (!jl.axis.y()) {
    if (jl.limits[0] < jl.limits[1]) {
      assert(jl.limits[0] == -jr.limits[1]);
      assert(jl.limits[1] == -jr.limits[0]);
    }
  }

}

void ensureMirror(const Body& bl, const Body& br) {

  Transform3 xl = bl.xform;
  Transform3 xr = br.xform;

  compare(xl, flipy(xr));

}

class AnchorLookup {
public:
  const KinBody& kbody;
  const Transform3Array& xforms;
  AnchorLookup(const KinBody& k, const Transform3Array& x): kbody(k), xforms(x) {}
  vec3 operator()(const std::string& n) {
    return kbody.jointAnchor(xforms, kbody.lookupJoint(n));
  }
};

Darwin::Darwin(const std::string& filename) 

{
  
   kbody.loadXML(filename);
   Transform3Array xforms;
  RealArray jvalues(kbody.joints.size(), 0.0);

  kbody.transforms(jvalues, xforms);

  defaultFootPos = kbody.manipulatorFK(xforms, 0).translation();
  defaultComPos = kbody.com(xforms);

  footAnkleDist = kbody.manipulators[0].xform.translation().norm();

  std::cerr << "FOOT ANKLE DIST = " << footAnkleDist << "\n";

  std::cerr << "DEFAULT COM POS = " << defaultComPos << "\n";

  std::cerr << "DEFAULT FOOT POS = " << defaultFootPos << "\n";

  for (size_t i=0; i<kbody.joints.size(); ++i) {
    jointOrder.push_back(i);
  }

}


