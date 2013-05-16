#include <mzcommon/MzGlutApp.h>
#include "HuboPlus.h"
#include <assert.h>
#include <mzcommon/mersenne.h>

using namespace fakerave;

class WalkDemo: public MzGlutApp {
public:

  HuboPlus hplus;
  KinBody& kbody;

  HuboPlus::KState state;

  Transform3Array xforms;

  vec3 desiredCom;
  vec3 actualCom;

  Transform3 desired[4];
  
  HuboPlus::IKMode mode[4];

  GLUquadric* quadric;

  enum WhichThing {
    THING_COM=0,
    THING_SWING_FOOT,
  };

  int which;

  WalkDemo(int argc, char** argv):
    MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE),
    hplus(argv[1]),
    kbody(hplus.kbody)

  {

    initWindowSize(640, 480);
    createWindow("Hubo Demo");
    setupBasicLight(vec4f(1,1,1,0));

    which = THING_SWING_FOOT;

    double bz = 0.85;

    camera.aim(vec3f(3, 0, bz),
               vec3f(0, 0, bz),
               vec3f(0, 0, 1));

    camera.setPerspective();

    camera.setRotateType(GlCamera::ROTATE_2_AXIS);

    camera.setHomePosition();

    quadric = gluNewQuadric();

    kbody.compileDisplayLists();

    resetState();

    kbody.transforms(state.jvalues, xforms);

    testJacobians();



    mode[0] = HuboPlus::IK_MODE_SUPPORT;
    mode[1] = HuboPlus::IK_MODE_WORLD;
    mode[2] = HuboPlus::IK_MODE_FREE;
    mode[3] = HuboPlus::IK_MODE_FREE;

    resetAll();

    updateStance();


  }

  void testJacobians() {

    IndexArray mainJoints;
    for (int i=0; i<4; ++i) {
      const Manipulator& m = kbody.manipulators[i];
      for (size_t j=0; j<m.jointIndices.size(); ++j) {
        mainJoints.push_back(m.jointIndices[j]);
      }
    }
    for (size_t i=DOF_POS_X; i<=DOF_ROT_Z; ++i) {
      mainJoints.push_back(i);
    }

    std::cout << "got " << mainJoints.size() << " main joints " << "\n";

    // get the jacobian of the com
    MatX Jt, Jnt;
    kbody.comJacobian(xforms, mainJoints, Jt);
    kbody.comNumericalJacobian(state.jvalues, xforms, mainJoints, Jnt);

    //std::cout << "COM Jt=\n" << Jt << "\n\n";
    //std::cout << "COM Jnt=\n" << Jnt << "\n\n";
    //std::cout << "COM Error=\n" << Jnt-Jt << "\n\n";
    std::cout << "COM Error max = " << (Jnt-Jt).lpNorm<Eigen::Infinity>() << "\n\n";

    const Manipulator& m = kbody.manipulators[0];
    Transform3 fk = kbody.manipulatorFK(xforms, 0);

    kbody.jacobian(xforms, m.effectorIndex, fk.translation(), true, mainJoints, Jt);
    kbody.numericalJacobian(state.jvalues, xforms, m.effectorIndex, fk.translation(), true, mainJoints, Jnt);

    //std::cout << "Manip Jt=\n" << Jt << "\n\n";
    //std::cout << "Manip Jnt=\n" << Jnt << "\n\n";
    //std::cout << "Manip Error=\n" << Jnt-Jt << "\n\n";
    std::cout << "Manip Error max = " << (Jnt-Jt).lpNorm<Eigen::Infinity>() << "\n\n";

  }

  void resetAll() {

    real fy = hplus.defaultFootPos[1];

    desired[0].setTranslation(vec3(0,  fy, 0));
    desired[1].setTranslation(vec3(0, -fy, 0.08));
    desiredCom = vec3(0.0, fy-0.01, 0.56);

    resetState();

  }

  void resetState() {

    state.body_pos = vec3(0, 0, 0.85);
    state.body_rot = quat();

    state.jvalues.resize(kbody.joints.size(), 0.0);

    real deg = M_PI/180;
    const JointLookup& jl = hplus.jl;
    state.jvalues[jl("LSR")] =  15*deg;
    state.jvalues[jl("RSR")] = -15*deg;
    state.jvalues[jl("LSP")] =  20*deg;
    state.jvalues[jl("RSP")] =  20*deg;
    state.jvalues[jl("LEP")] = -40*deg;
    state.jvalues[jl("REP")] = -40*deg;

    kbody.transforms(state.jvalues, xforms);

  }


  void updateStance() {

    bool ok = hplus.comIK( state, desiredCom,
                           desired, mode, HuboPlus::noGlobalIK(), xforms,
                           0, 0 );


    kbody.transforms(state.jvalues, xforms);

    actualCom = state.xform() * kbody.com(xforms);

    std::cout << "ok is " << ok << "\n";
    std::cout << "com at " << actualCom << "\n";
    std::cerr << "comerr = " << desiredCom - actualCom << "\n\n";

  }

  virtual void display() {

    MzGlutApp::display();

    glMatrixMode(GL_MODELVIEW);

    glColor3ub(0, 127, 127);
    glNormal3f(0, 0, 1);
    glBegin(GL_LINES);
    double s = 0.5;
    double n = 10;
    double x = s*n;
    for (int i=-n; i<=n; ++i) {
      glVertex2d(x, i*s);
      glVertex2d(-x, i*s);
      glVertex2d(i*s, x);
      glVertex2d(i*s, -x);
    }
    glEnd();





    glPushMatrix();
    glstuff::mult_transform(state.xform());

    hplus.render(xforms);
    glClear(GL_DEPTH_BUFFER_BIT);

    hplus.kbody.renderSkeleton(xforms, quadric);

    glPopMatrix();

    glColor3ub(255, 0, 255);
    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], actualCom[2]);
    gluSphere(quadric, 0.05, 32, 24);
    glPopMatrix();

    glPushMatrix();
    glTranslated(actualCom[0], actualCom[1], 0.01);
    glColor3ub(127, 0, 127);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glRotated(180, 1, 0, 0);
    gluDisk(quadric, 0, 0.05, 32, 1);
    glPopMatrix();


    glutSwapBuffers();

  }

  void updateThing(int dx, int dy, int dz) {

    vec3 thing;

    if (which == THING_COM) {
      thing = desiredCom;
    } else {
      thing = desired[1].translation();
    } 

    real scl = 0.01;
    thing[0] += scl*dx;
    thing[1] += scl*dy;
    thing[2] += scl*dz;

    if (which == THING_COM) {
      std::cout << "setting desiredCom = " << thing << "\n";
      desiredCom = thing;
    } else {
      std::cout << "setting swing foot = " << thing << "\n";
      desired[1].setTranslation(thing);
    } 

    updateStance();
    glutPostRedisplay();
    
  }

  virtual void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27: // ESC
      exit(0);
    case 'r':
      resetState();
      updateStance();
      glutPostRedisplay();
      break;
    case 'R':
      resetAll();
      updateStance();
      glutPostRedisplay();
      break;
    case '1':
      which = THING_COM;
      updateThing(0,0,0);
      break;
    case '2':
      which = THING_SWING_FOOT;
      updateThing(0,0,0);
      break;
    case 'q':
      updateThing(1,0,0);
      break;
    case 'a':
      updateThing(-1,0,0);
      break;
    case 'w':
      updateThing(0,1,0);
      break;
    case 's':
      updateThing(0,-1,0);
      break;
    case 'e':
      updateThing(0,0,1);
      break;
    case 'd':
      updateThing(0,0,-1);
      break;
    }
  }
  

};

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " HUBOFILE.xml\n";
    return 1;
  }

  WalkDemo demo(argc, argv);

  demo.run();

  return 0;
  

}
