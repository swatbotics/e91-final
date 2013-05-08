#include "bipedSearch.h"
#include "pathplanner.h"
#include <stdlib.h>
#include <iostream>
#include <vector>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

#include <math.h>
#include <sstream>
#include <sys/time.h>

typedef vec2u<int> vec2i;

int width=0, height=0;
OccupancyGrid<unsigned char> grid;
GLuint grid_texture;

GLint viewport[4];
GLdouble projection[16];
GLdouble modelview[16];

vec2i init(-1,-1);
float initTheta = 0;

int goalr = 3;
float inflate_h = 1.25;
float inflate_z = 1;

int maxDepth = 1000;
int viewDepth = 30;

vec2i goal(-1,-1);
 
BipedChecker* checker=0;
bipedSearch helper;

biped* searchResult=0;
double planTime = 0;

bool auto_plan = true;
bool show_help = false;

GLUquadric* quadric = 0;

PathPlanner planner;
vector<biped*> bipedTrajectory;

enum MouseAction {
  MouseNone,
  MouseGoal,
  MouseInit,
  MouseTheta,
};

MouseAction mouse_action = MouseNone;

enum ChangeFlags {
  NoChange         = 0x00,
  InitChanged      = 0x01,
  GoalChanged      = 0x02,
  InflationChanged = 0x04,
  DepthChanged     = 0x08,
};

int changes = 0;

double gettimeasdouble() {
  
  struct timeval tp;
  gettimeofday(&tp, NULL);
  return double(tp.tv_sec) + 1e-6 * double(tp.tv_usec);

}

bool valid(const vec2i& p) {
  return ( p.x() >= 0 && 
           p.y() >= 0 && 
           size_t(p.x()) < grid.nx() && 
           size_t(p.y()) < grid.ny() &&
           grid(p.x(), p.y()) );
}

void getInputSegments(){
    SegmentList segments;
    segments.push_back(Line(20, 5, 20, 25)); //TODO: real input
    segments.push_back(Line(20, 25, 40, 25)); 
    segments.push_back(Line(40, 25, 40, 5)); 
    segments.push_back(Line(40, 5, 20, 5)); 
    planner = PathPlanner(segments);
    planner.populateTrajectory();
    StateTrajectory* traj = planner.getPathTrajectory();
    for (int i = 0; i < traj->size(); i++){
        cout << traj->at(i).toString() << endl;
    }
}

//Function to search a trajectory of states
void searchTrajectory(){
    StateTrajectory* stateTraj = planner.getPathTrajectory();
    //RobotSegment curstate = RobotSegment(init.x(), init.y(), initTheta, Line(0, 0, 0, 0));
    RobotSegment curstate = stateTraj->back();
    stateTraj->pop_back();
    while(stateTraj->size() != 0){
    
        RobotSegment goalstate = stateTraj->back();
        checker = new BipedChecker(&grid, goalstate.robot_pos[0],
                                  goalstate.robot_pos[1], inflate_h, inflate_z);
        cout << "Moving from " << curstate.toString() << " to " << goalstate.toString() <<endl ;
        biped* output = helper.search(curstate.robot_pos[0], curstate.robot_pos[1],
                               curstate.theta, goalstate.robot_pos[0],
                               goalstate.robot_pos[1], 3, goalstate.theta,
                               checker, maxDepth, viewDepth); //hardcoded r? TODO: fix
        bipedTrajectory.push_back(output);
        curstate = goalstate;
        stateTraj->pop_back();
    }
}


void updateSearch() { 

  if (!valid(init) || !valid(goal)) { 
    return;
  }

  if (!checker or (changes & (GoalChanged | InflationChanged))) {
    delete checker;
    checker = 0;
    checker = new BipedChecker(&grid, goal.x(), goal.y(), inflate_h, inflate_z);
  }
  
  searchResult = 0;
  helper.clear();

  double start = gettimeasdouble();
  
  bipedTrajectory.clear();
  searchTrajectory();
    
  /*searchResult = helper.search(init.x(), init.y(), initTheta,
                               goal.x(), goal.y(), goalr, 0, // TODO: don't hardcode
                               checker, maxDepth, viewDepth);*/

  planTime = gettimeasdouble() - start;

  changes = NoChange;


}

void draw(const biped* b, bool recurse) { 

  if (!b) { return; }

  if (recurse) { draw(b->pred, recurse); }

  glPushMatrix();
  glTranslated(b->x, b->y, 0);
  glRotated(b->theta*180/M_PI, 0, 0, 1);

  int r1 = (b->ft == RIGHT) ? 255 : 191;
  int g1 = 191;
  int b1 = (b->ft == LEFT) ? 255 : 191;

  int r0 = r1/4;
  int g0 = 0;
  int b0 = b1/4;

  glBegin(GL_QUADS);
  glColor3ub(r1, g1, b1);
  glVertex2f( 2, -1);
  glVertex2f( 2,  1);
  glColor3ub(r0, g0, b0);
  glVertex2f(-2,  1);
  glVertex2f(-2, -1);
  glEnd();
  
  glPopMatrix();
  
}

void drawString(int x, int y, const std::string& str, void* 
                font=GLUT_BITMAP_8_BY_13) {
  
  glRasterPos2i(x, y);

}

void display() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glEnable(GL_TEXTURE_2D);
  glBindTexture(GL_TEXTURE_2D, grid_texture);

  glColor3ub(255,255,255);

  glBegin(GL_QUADS);

  glTexCoord2f(1, 0);
  glVertex2f(grid.nx(), 0);

  glTexCoord2f(0, 0);
  glVertex2f(0, 0);

  glTexCoord2f(0, 1);
  glVertex2f(0, grid.ny());

  glTexCoord2f(1, 1);
  glVertex2f(grid.nx(), grid.ny());

  glEnd();

  glDisable(GL_TEXTURE_2D);

  if (valid(init)) {
    glPushMatrix();

    glColor3ub(63,255,63);
    glTranslated(init.x(), init.y(), 0);
    gluDisk(quadric, 0, goalr, 32, 1);

    glRotated(initTheta*180/M_PI, 0, 0, 1);
    glBegin(GL_TRIANGLES);
    glVertex2f(6, 0);
    glVertex2f(0, 2);
    glVertex2f(0, -2);
    glEnd();

    glPopMatrix();

  }

  if (valid(goal)) {
    glPushMatrix();
    glColor3ub(255,63,63);
    glTranslated(goal.x(), goal.y(), 0);
    gluDisk(quadric, 0, goalr, 32, 1);
    glPopMatrix();
  }
  
  for(int i = 0; i < bipedTrajectory.size(); i++){
    draw(bipedTrajectory[i], true);
  }
  //draw(searchResult, true);

  glMatrixMode(GL_PROJECTION);
  glPushMatrix();
  glLoadIdentity();
  gluOrtho2D(0, width, 0, height);

  std::ostringstream ostr;
  ostr << "Goal pos:     (" << goal.x() << ", " << goal.y() << ")\n"
       << "Init pos:     (" << init.x() << ", " << init.y() << ")\n"
       << "Init theta:   " << initTheta*180/M_PI << "\n"
       << "XY inflation: " << inflate_h << "\n"
       << "Z inflation:  " << inflate_z << "\n"
       << "Max depth:    " << maxDepth << "\n"
       << "View depth:   " << viewDepth << "\n"
       << "Auto-plan:    " << (auto_plan ? "on" : "off") << "\n"
       << "\n"
       << "Plan cost: " << (searchResult ? searchResult->costToCome : 0) << "\n"
       << "Plan time: " << (searchResult ? planTime : 0) << "\n";

  if (show_help) { 
    ostr << "\n\n"
         << " Left-click init pos\n"
         << "Shift+click init theta\n"
         << "Right-click goal pos\n\n"
         << "        1/2 max depth\n"
         << "        3/4 view depth\n"
         << "        -/+ XY inflation\n"
         << "          A auto-plan\n"
         << "      Enter re-plan\n"
         << "        ESC quit\n"
         << "          ? hide help";
  } else {
    ostr << "\nPress ? to toggle help.";
  }
    


  std::string str = ostr.str();
  
  void* font = GLUT_BITMAP_8_BY_13;
  const int tx = 8;
  const int ty = 13;
  const int th = ty+2;
  int maxwidth = 0;
  int linewidth = 0;
  int rh = th;

  for (size_t i=0; i<str.length(); ++i) {
    char c = str[i];
    if (c == '\n') { 
      maxwidth = std::max(maxwidth, linewidth);
      linewidth = 0;
      rh += th;
    } else { 
      linewidth += tx;
    }
  }
  maxwidth = std::max(maxwidth, linewidth);

  int rw = maxwidth + 20;
  rh += 10;

  glColor4ub(191,191,255,225);
  glEnable(GL_BLEND);
  glBegin(GL_QUADS);
  glVertex2f( 0, height);
  glVertex2f(rw, height);
  glVertex2f(rw, height-rh);
  glVertex2f( 0, height-rh);
  glEnd();
  glDisable(GL_BLEND);

  int rx = 10;
  int ry = height-th;

  glColor3ub(0,0,0);
  glRasterPos2i(rx, ry);

  for (size_t i=0; i<str.length(); ++i) {
    char c = str[i];
    if (c == '\n') {
      ry -= th;
      glRasterPos2i(rx, ry);
    } else {
      glutBitmapCharacter(font, str[i]);
    }
  }

  glPopMatrix();

  glutSwapBuffers();

}

void reshape(int w, int h) {

  width = w;
  height = h;

  glViewport(0, 0, width, height);

  float ga = float(grid.nx())/grid.ny();
  float wa = float(w)/h;

  float wh = grid.ny();
  float ww = grid.nx();

  if (ga > wa) {
    wh *= ga/wa;
  } else {
    ww *= wa/ga;
  }

  float cx = 0.5*grid.nx();
  float cy = 0.5*grid.ny();

  glMatrixMode(GL_PROJECTION);
  glLoadIdentity();

  gluOrtho2D(cx - 0.5 * ww, cx + 0.5*ww, 
             cy + 0.5 * wh, cy - 0.5*wh);

  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glGetIntegerv(GL_VIEWPORT, viewport);
  glGetDoublev(GL_PROJECTION_MATRIX, projection);
  glGetDoublev(GL_MODELVIEW_MATRIX, modelview);

}

void keyboard(unsigned char key, int x, int y) {

  key = tolower(key);
  bool plan = false;

  switch (key) {
  case 27: exit(0); break;
  case '?':
    show_help = !show_help;
    break;
  case 'a': 
    auto_plan = !auto_plan;
    break;
  case '\n': case '\r': // ENTER
    plan = true;
    break;
  case '+': case '=':
    inflate_h = std::min(inflate_h + 0.25, 10.0);
    changes |= InflationChanged;
    break;
  case '-':
    inflate_h = std::max(inflate_h - 0.25, 0.5);
    changes |= InflationChanged;
    break;
  case '1': 
    maxDepth = std::max(maxDepth-10, 10);
    changes |= DepthChanged;
    break;
  case '2':
    maxDepth = std::min(maxDepth+10, 2000);
    changes |= DepthChanged;
    break;
  case '3': 
    viewDepth = std::max(viewDepth-1, 4);
    changes |= DepthChanged;
    break;
  case '4':
    viewDepth = std::min(viewDepth+1, 100);
    changes |= DepthChanged;
    break;
  };

  if ((auto_plan || plan) && changes) { 
    updateSearch();
  }

  glutPostRedisplay();

}

vec2i unproject(int x, int y) {

  GLdouble ox, oy, oz;
  gluUnProject(x, height-y-1, 0, modelview, projection, viewport, &ox, &oy, &oz);

  return vec2i(ox+0.25, oy+0.25);

}

void motion(int x, int y) {

  vec2i mpos = unproject(x, y);

  if (mouse_action == MouseGoal && valid(mpos) && mpos != goal) {
    
    goal = mpos; 
    changes |= GoalChanged;
    
  } else if (mouse_action == MouseInit && valid(mpos) && mpos != init) {

    init = mpos;
    vec2i diff = goal - init;
    initTheta = atan2(diff.y(), diff.x());
    changes |= InitChanged;

  } else if (mouse_action == MouseTheta && valid(init)) {

    vec2i diff = mpos - init;
    initTheta = atan2(diff.y(), diff.x());
    changes |= InitChanged;

  }

  if (changes) { searchResult = 0; }

  glutPostRedisplay();
  
}

void mouse(int button, int state, int x, int y) {


  if (mouse_action == MouseNone && state == GLUT_DOWN) {
    
    if (button == GLUT_LEFT_BUTTON) {
      int mod = glutGetModifiers();
      if (mod == 0) {
        mouse_action = MouseInit;
      } else if (mod == GLUT_ACTIVE_SHIFT) {
        mouse_action = MouseTheta;
      }
    } else if (button == GLUT_RIGHT_BUTTON) {
      mouse_action = MouseGoal;
    }
    
  }

  if (mouse_action != MouseNone) { 

    motion(x,y); 

    if (state == GLUT_UP) { 

      mouse_action = MouseNone;

      if (auto_plan && changes) { 
        updateSearch();
        glutPostRedisplay();
      }
      
    }

  }

  
}


void usage(int code) {
  std::ostream& ostr = code ? std::cerr : std::cout;
  ostr << "usage: bipedDemo map.png\n";
  exit(code);
}

int main(int argc, char** argv) {
  
  getInputSegments(); //initialize the path planner

  glutInit(&argc, argv);
  glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGB);
  glutInitWindowSize(640, 480);

  if (argc < 2) { usage(1); }

  grid.load(argv[1]);
  if (grid.empty()) { 
    std::cerr << "Error loading grid: " << argv[1] << "\n";
    exit(1);
  }

  glutCreateWindow("Biped plan demo");
  glutDisplayFunc(display);
  glutReshapeFunc(reshape);
  glutKeyboardFunc(keyboard);
  glutMouseFunc(mouse);
  glutMotionFunc(motion);
  
  glClearColor(1,1,1,1);

  glGenTextures(1, &grid_texture);
  glBindTexture(GL_TEXTURE_2D, grid_texture);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
  glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

  std::vector<unsigned char> rgbbuf(grid.nx() * grid.ny() * 4);
  int offs = 0;
  for (size_t y=0; y<grid.ny(); ++y) {
    for (size_t x=0; x<grid.nx(); ++x) {
      rgbbuf[offs++] = grid(x,y);
      rgbbuf[offs++] = grid(x,y);
      rgbbuf[offs++] = grid(x,y);
      rgbbuf[offs++] = 255;
    }
  }
    
  glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA,
               grid.nx(), grid.ny(), 0, GL_RGBA,
               GL_UNSIGNED_BYTE, &(rgbbuf[0]));

  quadric = gluNewQuadric();

  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  searchTrajectory();
  glutMainLoop();

  return 0;

}
