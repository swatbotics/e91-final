/*
 * Biped class- parent class of Hubo and Darwin
 */

#ifndef _BIPED_H_
#define _BIPED_H_

#include "fakerave.h"

class Biped {
public:
    
    enum ManipIndex {
      MANIP_L_FOOT,
      MANIP_R_FOOT,
      MANIP_L_HAND,
      MANIP_R_HAND,
      NUM_MANIPULATORS,
    };

    enum IKMode {
      IK_MODE_FREE,
      IK_MODE_FIXED,
      IK_MODE_BODY,
      IK_MODE_WORLD,
      IK_MODE_SUPPORT,
    };
    
    Biped();

    static const char* ikModeString(int i);

    fakerave::quat footRot;

    class KState {
    public:
    
      fakerave::quat	    body_rot;
      fakerave::vec3	    body_pos;

      fakerave::RealArray   jvalues;

      fakerave::Transform3 xform() const;
      void setXform(const fakerave::Transform3& x);

    };

    fakerave::KinBody kbody;
    fakerave::BodyLookup bl;
    fakerave::JointLookup jl;
    fakerave::ManipulatorLookup ml;

    fakerave::vec3 defaultFootPos;
    fakerave::vec3 defaultComPos;
  
    fakerave::real footAnkleDist;

    fakerave::IndexArray jointOrder;

    size_t DEFAULT_COM_ITER;
    fakerave::real DEFAULT_COM_PTOL;
  
    virtual void render(const fakerave::Transform3Array& xforms,
              const fakerave::Vec4Array* overrideColors=0) const;
  
    virtual void initIK(size_t mi,
              const fakerave::Transform3& desired,
              fakerave::RealArray& jvalues) const;

    virtual bool manipIK(size_t mi,
               const fakerave::Transform3& desired,
               fakerave::RealArray& jvalues,
               fakerave::Transform3Array& work,
               bool global=true) const;

    static const bool* allGlobalIK() {
      static const bool g[4] = { true, true, true, true };
      return g;
    }

    static const bool* noGlobalIK() {
      static const bool g[4] = { false, false, false, false };
      return g;
    }
  
    virtual bool stanceIK( KState& state,
                 const fakerave::Transform3 manipXforms[NUM_MANIPULATORS],
                 const IKMode mode[NUM_MANIPULATORS],
                 const bool globalIK[NUM_MANIPULATORS],
                 fakerave::Transform3Array& work,
                 bool* ikvalid=0 ) const;

    virtual bool comIK( KState& state,
              const fakerave::vec3& com,
              const fakerave::Transform3 manipXforms[NUM_MANIPULATORS],
              const IKMode mode[NUM_MANIPULATORS],
              const bool globalIK[NUM_MANIPULATORS],
              fakerave::Transform3Array& work,
              fakerave::real ascl=10,
              fakerave::real fscl=0,
              bool* ikvalid=0 ) const;

    virtual fakerave::real nonSupportMass(const IKMode mode[2]) const;

    virtual void computeGroundReaction(const fakerave::vec3& comPos,
			     const fakerave::vec3& comAccel,
			     const fakerave::Transform3 footXforms[2],
			     const IKMode mode[2],
			     fakerave::vec3 forces[2],
			     fakerave::vec3 torques[2]) const;

    virtual void computeGroundReaction(const fakerave::real mass,
			     const fakerave::vec3& comPos,
			     const fakerave::vec3& comAccel,
			     const fakerave::Transform3& footXform,
			     fakerave::vec3* force,
			     fakerave::vec3* torque) const;


};

#endif

