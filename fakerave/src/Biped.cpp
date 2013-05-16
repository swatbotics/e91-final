#include "Biped.h"
#include <mzcommon/glstuff.h>
#include <mzcommon/TimeUtil.h>

#include <Eigen/Dense>


using namespace fakerave;

const char* Biped::ikModeString(int i) {
  switch (i) {
  case IK_MODE_FREE:    return "free";
  case IK_MODE_FIXED:   return "fixed";
  case IK_MODE_BODY:    return "body";
  case IK_MODE_WORLD:   return "world";
  case IK_MODE_SUPPORT: return "support";
  default: return "[INVALID]";
  }
}

Biped::Biped():
  bl(kbody), jl(kbody), ml(kbody) {

  DEFAULT_COM_ITER = 1000;
  DEFAULT_COM_PTOL = 1e-3;

}


void Biped::render(const Transform3Array& xforms,
                      const Vec4Array* overrideColors) const {
  
  kbody.render(xforms,
               vec4(0.5,0.5,0.5,1), 
               overrideColors);

}

bool Biped::manipIK(size_t mi,
                       const Transform3& desired,
                       RealArray& jvalues,
                       Transform3Array& xforms,
                       bool global) const {
    
    if (global) { initIK(mi, desired, jvalues); }
    return kbody.manipulatorIK(mi, desired, jvalues, xforms);

  

}

void Biped::initIK(size_t mi, const Transform3& desired,
                      		RealArray& jvalues) const {

  kbody.centerJoints(kbody.manipulators[mi].jointIndices, jvalues);
}


bool Biped::stanceIK( KState& state,
                              const Transform3 manipXforms[NUM_MANIPULATORS],
                              const IKMode mode[NUM_MANIPULATORS],
                              const bool shouldInitIK[NUM_MANIPULATORS],
                              Transform3Array& work,
                              bool* ikvalid ) const {

  bool allOK = true;

  kbody.transforms(state.jvalues, work);

  for (int i=0; i<NUM_MANIPULATORS; ++i) {

    Transform3 desired = manipXforms[i];
    bool doIK = false;
    
    switch (mode[i]) {
    case IK_MODE_BODY:
      doIK = true;
      break;
    case IK_MODE_WORLD:
    case IK_MODE_SUPPORT:
      doIK = true;
      desired = state.xform().inverse() * desired;
      break;
    default:
      doIK = false;
      break;
    }

    bool valid = true;

    if (doIK) {

      Transform3 cur = kbody.manipulatorFK(work,i);
      
      if ( (cur.translation() - desired.translation()).norm() > kbody.DEFAULT_PTOL ||
           quat::dist(cur.rotation(), desired.rotation()) > kbody.DEFAULT_QTOL) {

        if (shouldInitIK[i]) { initIK(i, desired, state.jvalues); }
        
        valid = manipIK(i, desired, state.jvalues, work);

      }
      
    }

    if (!valid) { allOK = false; }
    if (ikvalid) { ikvalid[i] = valid; }

  }

  return allOK;

}

#define debug if (0) std::cerr

bool Biped::comIK( KState& state,
                      const vec3& dcom,
                      const Transform3 manipXforms[NUM_MANIPULATORS],
                      const IKMode mode[NUM_MANIPULATORS],
                      const bool globalIK[NUM_MANIPULATORS],
                      Transform3Array& work,
                      real ascl,
                      real fscl,
                      bool* ikvalid ) const {

  bool ok = false;

  MatX gT, gpT, gxT, fxT, fpT, lambda, gxxpT(6, 3), deltap;
  MatX gfT, deltaf;

  const real alpha = 0.5;
  
  IndexArray pdofs;
  for (size_t i=DOF_POS_X; i<=DOF_ROT_Z; ++i) {
    pdofs.push_back(i);
  }

  IndexArray fdofs;
  for (int i=0; i<4; ++i) {
    if (fscl && mode[i] == IK_MODE_FREE) {
      const IndexArray& jidx = kbody.manipulators[i].jointIndices;
      for (size_t j=0; j<jidx.size(); ++j) {
        fdofs.push_back(jidx[j]);
      }
    }
  }

  for (size_t iter=0; iter<DEFAULT_COM_ITER; ++iter) {

    // try doing IK
    ok = stanceIK( state, manipXforms, mode, globalIK, work, ikvalid );


    // compute the COM pos
    kbody.transforms(state.jvalues, work);

    vec3 com = state.xform() * kbody.com(work);

    // get the error
    vec3 comerr = dcom - com;

    if (comerr.norm() < DEFAULT_COM_PTOL) {
      debug << "breaking after " << iter << " iterations\n";
      break; 
    } else {
      ok = false;
    }

        
    if (ascl == 0) {

      state.body_pos += alpha * comerr;

    } else {
      
      // get jacobians ftw
      kbody.comJacobian(work, pdofs, gpT);
      gpT.block(3, 0, 3, 3) *= ascl;
      

      debug << "gpT=" << gpT << "\n\n";

      if (!fdofs.empty()) {
        kbody.comJacobian(work, fdofs, gfT);
        debug << "gfT=" << gfT << "\n\n";
      }

      gxxpT.setZero();

      for (int i=0; i<4; ++i) {

        if (mode[i] == IK_MODE_WORLD || mode[i] == IK_MODE_SUPPORT) {

          const std::string& name = kbody.manipulators[i].name;

          kbody.comJacobian(work, kbody.manipulators[i].jointIndices, gxT);
          kbody.manipulatorJacobian(work, i, pdofs, fpT);
          kbody.manipulatorJacobian(work, i, fxT);
          lambda = fxT.colPivHouseholderQr().solve(gxT);

          fpT.block(3, 0, 3, 6) *= ascl;

          debug << "gxT[" << name << "]=\n" << gxT << "\n\n";
          debug << "fpT[" << name << "]=\n" << fpT << "\n\n";
          debug << "fxT[" << name << "]=\n" << fxT << "\n\n";
          debug << "lambda[" << name << "]=\n" << lambda << "\n\n";
          gxxpT += fpT * lambda;

        }

      }

      gT = gpT - gxxpT;
      Eigen::Vector3d cerr(comerr[0], comerr[1], comerr[2]);
      deltap = alpha * gT * cerr;

      debug << "gxxpT = \n" << gxxpT << "\n\n";
      debug << "gT = \n" << gT << "\n\n";
      debug << "deltap = \n" << deltap.transpose() << "\n\n";


      vec3 dp(deltap(0), deltap(1), deltap(2));
      vec3 dq(deltap(3), deltap(4), deltap(5));

      state.body_pos += dp;
      state.body_rot = quat::fromOmega(-dq) * state.body_rot;


    }

    if (!fdofs.empty()) {
      Eigen::Vector3d cerr(comerr[0], comerr[1], comerr[2]);
      deltaf = fscl * gfT * cerr;
      debug << "deltaf = \n" << deltaf.transpose() << "\n\n";
      for (size_t i=0; i<fdofs.size(); ++i) {
        state.jvalues[fdofs[i]] += deltaf(i);
      }
    }

  }

  return ok;

}
                           

Transform3 Biped::KState::xform() const {
  return Transform3(body_rot, body_pos);
}

void Biped::KState::setXform(const Transform3& t) {
  body_pos = t.translation();
  body_rot = t.rotation();
}


real Biped::nonSupportMass(const IKMode mode[2]) const {

  real totalMass = 0;

  for (size_t bi=0; bi<kbody.bodies.size(); ++bi) {
    
    bool addMass = true;

    for (int f=0; f<2; ++f) {
      if (mode[f] == IK_MODE_SUPPORT) {
	const Manipulator& m = kbody.manipulators[f];
	for (int j=0; j<2; ++j) {
	  size_t ji = m.jointIndices[m.jointIndices.size()-j-1];
	  if (kbody.bodyDependsOnJoint(bi, ji)) {
	    addMass = false;
	  }
	}
      }
    }

    if (addMass) { totalMass += kbody.bodies[bi].mass; }

  }
  
  return totalMass;

}

const real g = 9.8;

void Biped::computeGroundReaction(const vec3& comPos,
				     const vec3& comAccel,
				     const Transform3 footXforms[2],
				     const IKMode mode[2],
				     vec3 forces[2],
				     vec3 torques[2]) const {

  // Get the mass
  real m = nonSupportMass(mode);

  if (mode[0] == IK_MODE_SUPPORT && mode[1] == IK_MODE_SUPPORT) {

    // If we're in dual support

    // Find the displacement between the feet
    vec3 ry = footXforms[0].translation() - footXforms[1].translation();
    assert( fabs(ry.z()) < 1e-4 );

    // Let d be the distance between the feet
    real d = ry.norm();
    ry /= d;
    
    vec3 rz = vec3(0,0,1);
    vec3 rx = vec3::cross(ry,rz);
    assert( fabs( rx.norm() - 1.0 ) < 1e-8 );
    
    mat3 R;
    R.setRow(0, rx);
    R.setRow(1, ry);
    R.setRow(2, rz);
    
    // Set up a transformation from "world" space to the 
    // frame whose y axis points from right angle to left ankle
    // and whose origin is at the right ankle
    Transform3 forceFrame( quat::fromMat3(R), 
			   -R * footXforms[1].translation() );

    assert(comAccel.z() == 0);

    // COM position in the force frame
    vec3 cp = forceFrame * comPos;

    // Grab x and y coord of COM in force frame
    real x = cp.x();
    real c = cp.y();
    real h = cp.z();

    // COM acceleration the force frame
    vec3 ca = forceFrame.rotFwd() * comAccel;
    ca[2] += g;

    vec3 ff[2];

    real ratio = 1-c/d;

    ff[1][0] = m * (ratio*ca[0] + x*ca[1]/d);
    ff[1][1] = m * ratio * ca[1];
    ff[1][2] = m * (ratio*ca[2] + h*ca[1]/d);

    ff[0] = m*ca - ff[1];

    mat3 RT = R.transpose();
    
    for (int f=0; f<2; ++f) {

      mat3 forceToFoot = footXforms[f].rotInv() * RT;

      if (forces) { forces[f] = forceToFoot * ff[f]; }

      if (torques) {
	// Moment arm for each foot
	vec3 r = (f == 0) ? vec3(-x, d-c, -h) : -cp;
	torques[f] = forceToFoot * vec3::cross(r, ff[f]);
      }

    }

  } else if (mode[0] == IK_MODE_SUPPORT || mode[1] == IK_MODE_SUPPORT) {

    // If we're in single support

    int stance = (mode[1] == IK_MODE_SUPPORT ? 1 : 0);
    int swing = 1-stance;

    computeGroundReaction(m, comPos, comAccel, footXforms[stance], 
			  forces ? forces + stance : 0,
			  torques ? torques + stance : 0);

    if (forces) { forces[swing] = vec3(0); }
    if (torques) { torques[swing] = vec3(0); }

  } else {

    // no support
    if (forces) { forces[0] = forces[1] = vec3(0); }
    if (torques) { torques[0] = torques[1] = vec3(0); }
    
  }

    

}

void Biped::computeGroundReaction(const real m,
				     const vec3& comPos,
				     const vec3& comAccel,
				     const Transform3& footXform,
				     vec3* force,
				     vec3* torque) const {

  assert(comAccel.z() == 0);

  // COM in frame of foot
  vec3 cp = footXform.transformInv(comPos);

  // COM accel in frame of foot
  vec3 ca = footXform.rotInv() * comAccel;
  ca[2] += g; // tack on gravity to the Z component

  // F = ma
  vec3 f = m*ca;


  if (force)  { *force = f; }
  if (torque) { 
    // moment arm for torque from foot
    vec3 r = -cp;
    *torque = vec3::cross(r, f); 
  }

}



