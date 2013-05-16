#include "OccupancyGrid.h"
#include "GridSearch.h"
#include "lab3.h"
#include <stdlib.h>


//////////////////////////////////////////////////////////////////////
// declare some helper functions (defined below): 

void parseArgs(int argc, char** argv, 
               std::string& inputfile,
               float& radius,
               vec2u& init,
               vec2u& goal,
               bool& useheuristic,
		bool& truncate,
		float& inflate,
		bool& expand);

std::string getOutputName(const std::string& inputfile, bool useheuristic);

//////////////////////////////////////////////////////////////////////
// our main function: you shouldn't need to modify this!

int main(int argc, char** argv) {

  std::string inputfile;
  float radius;
  vec2u init, goal;
  bool useheuristic;
  bool truncate;
  float inflate;
  bool expand;
  
  parseArgs(argc, argv, inputfile, radius, init, goal, useheuristic, truncate, inflate, expand);

  std::string baseName = getOutputName(inputfile, useheuristic);

  //////////////////////////////////////////////////
  // create an occupancy grid and load the input file

  OccupancyGrid grid;

  grid.load(inputfile);

  if (grid.empty()) {
    std::cerr << "error loading grid!\n";
    return 1;
  }

  std::cout << "loaded from " << inputfile << ", "
            << "map is " << grid.nx() << "x" << grid.ny() << "\n";

  //////////////////////////////////////////////////
  // expand obstacles appropriately

  if (radius > 0) {
    OccupancyGrid tmp;
    expandObstacles(grid, radius, tmp, expand);
    grid = tmp;
  }

  //////////////////////////////////////////////////
  // now run the search

  GridSearchData helper;

  bool success = search(grid, init, goal, helper, useheuristic, truncate, inflate);

  std::cout << "search returned " 
            << (success ? "success" : "failure") << " " 
            << "after " << helper.expansionCount() << " expansions.\n";

  if (success) {
    std::cout << "path cost is " << helper.lookup(goal).costToCome << "\n";
  }

  if (grid.nx() <= 128 && grid.ny() <= 128) {

    std::string svgfile = baseName + ".svg";
    helper.makeDebugSVG(svgfile, grid, goal);
    std::cout << "wrote " << svgfile << "\n";

  } else {

    std::cout << "disabled SVG output because image too large!\n";

  }

  std::string pngfile = baseName + ".png";
  helper.makeDebugPNG(pngfile, grid, goal);
  std::cout << "wrote " << pngfile << "\n\n";

  return 0;

}


//////////////////////////////////////////////////////////////////////
// parse the command line arguments:

void parseArgs(int argc, char** argv, 
               std::string& inputfile,
               float& radius,
               vec2u& init, 
               vec2u& goal,
               bool& useheuristic,
		bool& truncate,
		float& inflate,
		bool& expand) {

  if (argc != 11) {
    std::cerr << "usage: search filename.png "
              << "radius initx inity goalx goaly useheuristic truncate inflation expand\n";
    exit(1);
  }

  // input file
  inputfile = argv[1];

  // radius
  char* endptr = 0;

  radius = strtod(argv[2], &endptr);
  if (radius < 0 || !endptr || *endptr) {
    std::cerr << "error parsing radius!\n";
    exit(1);
  }

  // init & goal
  int coords[4];

  for (int i=0; i<4; ++i) {
    coords[i] = strtol(argv[3+i], &endptr, 10);
    if (coords[i] < 0 || !endptr || *endptr) {
      std::cerr << "error parsing coordinate " << i+1 << "\n";
      exit(1);
    }
  }

  init = vec2u(coords[0], coords[1]);
  goal = vec2u(coords[2], coords[3]);

  // heuristic
  std::string arg = argv[7];

  if (arg == "0" || arg == "false") {
    useheuristic = false;
  } else if (arg == "1" || arg == "true") {
    useheuristic = true;
  } else {
    std::cerr << "error parsing heuristic argument\n";
    exit(1);
  }

  // truncation
  arg = argv[8];

  if (arg == "0" || arg == "false") {
    truncate = false;
  } else if (arg == "1" || arg == "true") {
    truncate = true;
  } else {
    std::cerr << "error parsing truncation argument\n";
    exit(1);
  }

  // inflation
  endptr = 0;

  inflate = strtod(argv[9], &endptr);
  if (inflate < 1 || !endptr || *endptr) {
    std::cerr << "error parsing inflation coefficient!\n";
    exit(1);
  }

  // expansion to allow passing through near wall
  arg = argv[10];

  if (arg == "0" || arg == "false") {
    expand = false;
  } else if (arg == "1" || arg == "true") {
    expand = true;
  } else {
    std::cerr << "error parsing expansion argument\n";
    exit(1);
  }


}

//////////////////////////////////////////////////////////////////////
// get the output name for the given input file 

std::string getOutputName(const std::string& filename, bool useheuristic) {

  // find last slash
  size_t slashpos = filename.rfind('/');
  if (slashpos == std::string::npos) { 
    slashpos = 0; 
  } else {
    ++slashpos;
  }

  // first dot after last slash
  size_t dotpos = filename.find('.', slashpos);
  if (dotpos == std::string::npos) { dotpos = filename.length(); }

  std::string baseName = filename.substr(slashpos, dotpos-slashpos);

  if (useheuristic) {
    baseName += "_astar";
  } else {
    baseName += "_dijkstras";
  }

  return "debug_" + baseName;

}
