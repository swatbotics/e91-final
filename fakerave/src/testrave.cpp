#include <mzcommon/MzGlutApp.h>
//#include "HuboPlus.h"
#include <src/fakerave.h>
#include <assert.h>
#include <mzcommon/mersenne.h>

using namespace fakerave;

class TestRave: public MzGlutApp {
public:

  KinBody kbody;
  GLUquadric* quadric;

  RealArray jvalues_init;
  RealArray jvalues_goal;

  RealArray jvalues;

  size_t interp_ticks;
  size_t cur_tick;

  size_t cur_joint;

  bool drawSkeleton;

  Transform3Array xforms;

  int which;

  TestRave(int argc, char** argv):
    MzGlutApp(argc, argv, GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB | GLUT_MULTISAMPLE)

  {

    initWindowSize(640, 480);
    createWindow("Test RAVE loading");
    setupBasicLight(vec4f(1,1,1,0));

    double bz = 0.2;

    camera.aim(vec3f(1, 0, bz),
               vec3f(0, 0, bz),
               vec3f(0, 0, 1));

    camera.setPerspective();

    camera.setRotateType(GlCamera::ROTATE_2_AXIS);

    camera.setHomePosition();

    quadric = gluNewQuadric();

    kbody.loadXML(argv[1]);

    jvalues.clear();
    jvalues.resize(kbody.joints.size(), 0.0);

    kbody.transforms(jvalues, xforms);


    // TEST IK
    if (!kbody.manipulators.empty()) {
      Transform3 fk = kbody.manipulatorFK(xforms, 0);
      fk = Transform3(vec3(0.0, 0.0, 0.04)) * fk;
      kbody.centerJoints(kbody.manipulators[0].jointIndices, jvalues);
      bool ok = kbody.manipulatorIK(0, fk, jvalues, xforms);
      kbody.transforms(jvalues, xforms);
      std::cerr << "ok = " << ok << "\n";
    }

    if (kbody.manipulators.size() > 2) {
      Transform3 fk = kbody.manipulatorFK(xforms, 2);
      bool ok = kbody.manipulatorPosIK(2, fk.translation() + vec3(-0.04,-0.03,-0.04), jvalues, xforms);
      kbody.transforms(jvalues, xforms);
      std::cerr << "ok = " << ok << "\n";
    }


    jvalues_init = jvalues;
    jvalues_goal = jvalues;

    interp_ticks = 10;
    cur_tick = interp_ticks;
    cur_joint = 0;

    drawSkeleton = false;

    setTimer(40, 0);

    updateAngles();

    kbody.compileDisplayLists();


  }

  virtual void timer(int value) {
    if (cur_tick < interp_ticks) { 
      ++cur_tick; 
      updateAngles();
      glutPostRedisplay();
    }
    setTimer(40, 0);
  }

  void updateAngles() {
    
    double u = double(cur_tick) / interp_ticks;
    if (u < 0) { u = 0; } else if (u > 1) { u = 1; }

    for (size_t i=0; i<jvalues.size(); ++i) {
      jvalues[i] = jvalues_init[i] + u*(jvalues_goal[i] - jvalues_init[i]);
    }
    
    kbody.transforms(jvalues, xforms);

  }

  void zeroAngles() {

    jvalues_init = jvalues_goal;

    jvalues_goal.clear();
    jvalues_goal.resize(kbody.joints.size(), 0.0);

    cur_tick = 0;
    
  }

  void randomAngles() {

    jvalues_init = jvalues_goal;

    for (size_t i=0; i<kbody.joints.size(); ++i) {
      jvalues_goal[i] = randomAngle(i);
    }

    cur_tick = 0;

  }

  double randomAngle(size_t i) const {
    const Joint& j = kbody.joints[i];
    double lo = j.limits[0];
    double hi = j.limits[1];
    if (!j.haveLimits || lo > hi) {
      lo = -M_PI/2;
      hi = M_PI/2;
    }
    return lo + (hi-lo)*mt_genrand_real1();
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

    glLineWidth(2.0);
    kbody.render(xforms);

    if (drawSkeleton) {
      glClear(GL_DEPTH_BUFFER_BIT);
      kbody.renderSkeleton(xforms, quadric);
    }

    /*
    glLineWidth(3.0);
    for (size_t i=0; i<xforms.size(); ++i) {
      glPushMatrix();
      glstuff::mult_transform(xforms[i]);
      drawAxes(0.05);
      glPopMatrix();
    }
    glLineWidth(2.0);
    */

    glutSwapBuffers();

  }

  void switchCurJoint(int delta) {
    cur_joint = size_t(cur_joint + delta) % kbody.joints.size();
    moveCurJoint(0);
  }
  
  void moveCurJoint(int delta) {
    jvalues_init = jvalues_goal;
    const Joint& j = kbody.joints[cur_joint];
    jvalues_goal[cur_joint] += delta * 10 * M_PI / 180;
    if (j.haveLimits && j.limits[0] < j.limits[1]) {
      jvalues_goal[cur_joint] = std::max(j.limits[0], std::min(jvalues_goal[cur_joint], j.limits[1]));
    }
    std::cout << "joint " << j.name << " at " << jvalues_goal[cur_joint]*180/M_PI << "\n";
    cur_tick = 0;
  }

  virtual void keyboard(unsigned char key, int x, int y) {
    switch (key) {
    case 27: // ESC
      exit(0);
    case 'r':
      randomAngles();
      glutPostRedisplay();
      break;
    case 'z':
      zeroAngles();
      glutPostRedisplay();
      break;
    case 'k':
      drawSkeleton = !drawSkeleton;
      glutPostRedisplay();
      break;
    case '[':
      switchCurJoint(-1);
      break;
    case ']':
      switchCurJoint(1);
      break;
    case '-':
      moveCurJoint(-1);
      glutPostRedisplay();
      break;
    case '+': case '=':
      moveCurJoint(1);
      glutPostRedisplay();
      break;
    default:
      MzGlutApp::keyboard(key, x, y);
      break;
    }
  }
  

};

int main(int argc, char** argv) {

  if (argc < 2) {
    std::cout << "usage: " << argv[0] << " MODEL.xml\n";
    return 1;
  }

  TestRave demo(argc, argv);

  demo.run();

  return 0;
  

}
