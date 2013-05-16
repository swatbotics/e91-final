#include "bipedSearch.h"
#include <stdlib.h>
#include <ctime>

void parseArgs(int argc, char** argv,
	       std::string& inputfile,
	       float &initx, float& inity, float& initTheta,
	       float &goalx, float& goaly,
	       float &goalr, int& maxDepth, int &viewDepth, float& inflate_h, float& inflate_z);

int main(int argc, char** argv){
  std::string inputfile;
  float initx, inity, initTheta, goalx, goaly, goalr, inflate_h, inflate_z;
  int maxDepth, viewDepth;
  parseArgs(argc, argv, inputfile, initx, inity, initTheta, goalx, goaly, goalr, maxDepth, viewDepth, inflate_h, inflate_z);

  // Load grid
  OccupancyGrid<unsigned char> grid;
  grid.load(inputfile);
  if (grid.empty()) {
    std::cerr << "error loading grid!\n";
    return 1;
  }

  bipedSearch helper;	
  time_t t0; time(&t0);
  BipedChecker* checker = new BipedChecker(&grid, goalx, goaly, inflate_h, inflate_z);
  time_t t1; time(&t1);
  
  std::string heurSVG = "Heuristic.svg";
  helper.makeSVG(NULL,heurSVG,(checker->getHeur()),goalx,goaly,goalr,false);
  cout<<"Map making time is: "<<difftime(t1,t0)<<" seconds"<<endl;
  time_t t2; time(&t2);
  
  biped* goal = helper.search(initx, inity, initTheta, goalx, goaly, goalr, 0, checker, maxDepth, viewDepth); 
  time_t t3; time(&t3);

  if (goal) {
    cout << "total cost is " << goal->costToCome << "\n";
  }
  cout<<"search time is: "<<difftime(t3,t2)<<" seconds"<<endl;
  
  if (goal==NULL){
    cout<<"Search failed after "<<helper.getCount()<<" expansions"<<endl;
    std::string filename = "DebugImage.svg";
    helper.makeSVG(NULL,filename,grid,goalx,goaly,goalr,false);
    return -1;
  } else {
    cout<<"Search successful after "<<helper.getCount()<<" expansions"<<endl;
    std::string filename = "DebugImage.svg";
    helper.makeSVG(goal,filename,grid,goalx,goaly,goalr,true);
  }
  return 0;
}

void parseArgs(int argc, char** argv,
	       std::string& inputfile,
	       float& initx, float& inity, float& initTheta,
	       float& goalx, float& goaly, float& goalr,
	       int& maxDepth, int& viewDepth, float& inflate_h, float& inflate_z){
  
  if (argc!=12) {
    std::cerr << "usage: main filename.png "
	      << "initx, inity, initTheta, goalx, goaly, goalr, maxDepth, viewDepth, inflate_h, inflate_z"<< endl;
    exit(1);
  }
  
  // Filename
  inputfile = argv[1];
  
  // All the floats
  char* endptr = 0;
  
  endptr = 0;
  initx = strtod(argv[2], &endptr);
  if (initx < 0 || !endptr || *endptr) {
    std::cerr << "error parsing initx!\n";
    exit(1);
  }
  
  endptr = 0;
  inity = strtod(argv[3], &endptr);
  if (inity < 0 || !endptr || *endptr) {
    std::cerr << "error parsing inity!\n";
    exit(1);
  }
  
  endptr = 0;
  initTheta = strtod(argv[4], &endptr);
  if (initTheta < 0 || !endptr || *endptr) {
    std::cerr << "error parsing initTheta!\n";
    exit(1);
  }
  
  endptr = 0;
  goalx = strtod(argv[5], &endptr);
  if (goalx < 0 || !endptr || *endptr) {
    std::cerr << "error parsing goalx!\n";
    exit(1);
  }
  
  endptr = 0;
  goaly = strtod(argv[6], &endptr);
  if (goaly < 0 || !endptr || *endptr) {
    std::cerr << "error parsing goaly!\n";
    exit(1);
  }
  
  endptr = 0;
  goalr = strtod(argv[7], &endptr);
  if (goalr < 0 || !endptr || *endptr) {
    std::cerr << "error parsing goalr!\n";
    exit(1);
  }
  
  maxDepth = atoi(argv[8]);
  if (maxDepth<0){
    std::cerr<<"error parsing maxDepth!\n";
    exit(1);
  }
  
  viewDepth = atoi(argv[9]);
  if (viewDepth<0){
    std::cerr<<"error parsing viewDepth!\n";
    exit(1);
  }
  
  endptr = 0;
  inflate_h = strtod(argv[10], &endptr);
  if (inflate_h < 0 || !endptr || *endptr) {
    std::cerr << "error parsing inflate!\n";
    exit(1);
  }
  
  endptr = 0;
  inflate_z = strtod(argv[11], &endptr);
  if (inflate_z < 0 || !endptr || *endptr) {
    std::cerr << "error parsing inflate!\n";
    exit(1);
  }
  
}
