#include "pathplanner.h"
#include <iostream>
#include <fstream>

#ifdef __APPLE__
#include <GLUT/glut.h>
#else
#include <GL/glut.h>
#endif

using namespace std;
vector<RobotSegment, stateAllocator>* traj;

Line splitStringToLine(const string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        elems.push_back(item);
    }
    //if(elems.size() > 4) throw an exception
    float nums[4];
    for(int i = 0; i < 4; i++){
        nums[i] = atof(elems.back().c_str());
        elems.pop_back();
    }
    return Line(nums[0], nums[1], nums[2], nums[3]);
}

int main(int argc, char** argv){
    //default box
    vector<Line, lineAllocator > segments;

    if (argc > 1){
        ifstream inFile(argv[1]);
        string line;
        if(inFile.is_open()){
            while(inFile.good()){
                getline(inFile, line);
                segments.push_back(splitStringToLine(line, ' '));
                //TODO: finish
            }
        }
        inFile.close();
    } else {
        //segments.push_back(Line(0, 0, 1, 2));
        segments.push_back(Line(1, 0, 0, 0));
        segments.push_back(Line(1, 1, 1, 0));
        segments.push_back(Line(0, 1, 1, 1));
        segments.push_back(Line(0, 0, 0, 1));
    };

    //initializeGraphics(argc, argv);
    PathPlanner planner = PathPlanner(segments);
    planner.populateTrajectory();
    traj = planner.getPathTrajectory();
    for (int i = 0; i < traj->size(); i++){
        cout << traj->at(i).toString() << endl;
    }
    //glutMainLoop();
    return 0;
}
