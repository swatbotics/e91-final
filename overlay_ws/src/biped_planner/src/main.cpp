#include "OccupancyGrid.h"
#include "dubinSearch.h"
#include "Checker.h"
#include "lab3.h"
#include <stdio.h>
#include <stdlib.h>

///////////////////////////////
// Fuction Definition

void parseArgs(int argc, char** argv,
                std::string& inputfile,
                float &radius,
                float &initx, float& inity, float& initTheta,
                float &goalx, float& goaly,
                float &goalr, float &goalTheta, int& depth,
				bool& heuristic, float& inflate);

//////////////////////////////
// Main

int main(int argc, char** argv){

    std::string inputfile;
    float radius;
    float initx, inity, initTheta, goalx, goaly, goalr, goalTheta;
    int depth;
	bool heuristic;
	float inflate;
	
	goalTheta = 0;
	
    parseArgs(argc, argv, inputfile, radius, initx, inity, initTheta, goalx, goaly, goalr, goalTheta, depth, heuristic, inflate);

    // Load grid
    OccupancyGrid grid;
    grid.load(inputfile);
    
    if (grid.empty()) {
        std::cerr << "error loading grid!\n";
        return 1;
    }

    // expand obstacles
    if (radius > 0) {
        OccupancyGrid tmp;
        expandObstacles(grid, radius, tmp, false);
        grid = tmp;
    }

    // Run search
    DubinSearch helper;
    GridChecker* checker = new GridChecker(&grid);
    Dubin* goal = helper.search(initx, inity, initTheta, goalx, goaly, goalr, goalTheta, checker, depth, heuristic, inflate);
    if (goal==NULL){
        cout<<"Search failed"<<endl;
        std::string filename = "DebugImage.svg";
        helper.makeSVG(NULL,filename,grid,goalx,goaly,goalr);

        return -1;
    }
    else{
        cout<<"Search successful after "<<helper.getCount()<<" expansions"<<endl;
        std::string filename = "DebugImage.svg";
        helper.makeSVG(goal,filename,grid,goalx,goaly,goalr);
    }

    return 0;
}
    

//////////////////////
// Helper functions

void parseArgs(int argc, char** argv,
                std::string& inputfile,
                float& radius,
                float& initx, float& inity, float& initTheta,
                float& goalx, float& goaly,
                float& goalr, float& goalTheta, int& depth, 
				bool& heuristic, float& inflate){

    if (argc!=12) {
        std::cerr << "usage: main filename.png "
                  << "radius, initx, inity, initTheta, goalx, goaly, goalr,"
				  << "expansionDepth, algorithm, inflate" << endl;
        exit(1);
    }

    // Filename
    inputfile = argv[1];

    // All the floats
    char* endptr = 0;

    radius = strtod(argv[2], &endptr);
    if (radius < 0 || !endptr || *endptr) {
        std::cerr << "error parsing radius!\n";
        exit(1);
    }

    endptr = 0;
    initx = strtod(argv[3], &endptr);
    if (initx < 0 || !endptr || *endptr) {
        std::cerr << "error parsing initx!\n";
        exit(1);
    }

    endptr = 0;
    inity = strtod(argv[4], &endptr);
    if (inity < 0 || !endptr || *endptr) {
        std::cerr << "error parsing inity!\n";
        exit(1);
    }

    endptr = 0;
    initTheta = strtod(argv[5], &endptr);
    if (initTheta < 0 || !endptr || *endptr) {
        std::cerr << "error parsing initTheta!\n";
        exit(1);
    }

    endptr = 0;
    goalx = strtod(argv[6], &endptr);
    if (goalx < 0 || !endptr || *endptr) {
        std::cerr << "error parsing goalx!\n";
        exit(1);
    }

    endptr = 0;
    goaly = strtod(argv[7], &endptr);
    if (goaly < 0 || !endptr || *endptr) {
        std::cerr << "error parsing goaly!\n";
        exit(1);
    }

    endptr = 0;
    goalr = strtod(argv[8], &endptr);
    if (goalr < 0 || !endptr || *endptr) {
        std::cerr << "error parsing goalr!\n";
        exit(1);
    }
    
    endptr = 0;
    goalTheta = strtod(argv[9], &endptr);
    if (goalTheta < 0 || !endptr || *endptr) {
        std::cerr << "error parsing goalr!\n";
        exit(1);
    }

    depth = atoi(argv[10]);
    if (depth<0){
        std::cerr<<"error parsing maxDepth!\n";
        exit(1);
    }
	
	std::string arg = argv[11];
	
	if (arg == "0" || arg == "false") {
		heuristic = false;
	} else if (arg == "1" || arg == "true") {
		heuristic = true;
	} else {
		std::cerr << "error parsing algorithm, true for A*, false for Dikstra's\n";
		exit(1);
	}
    
	endptr = 0;
    inflate = strtod(argv[12], &endptr);
    if (inflate < 0 || !endptr || *endptr) {
        std::cerr << "error parsing inflate!\n";
        exit(1);
    }
	
}



