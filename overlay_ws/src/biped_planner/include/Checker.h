#ifndef _CHECKER_H_
#define _CHECKER_H_

#include <vector>
#include "dubin.h"
#include "OccupancyGrid.h"

using namespace std;

class Checker {
public:
    virtual void getSuccessors(Dubin* initial, vector <Dubin*>& successors, float goalx, float goaly)=0;
	virtual void getSuccessors(Dubin* initial, vector<Dubin*>& successors)=0;
	virtual int getAddress(Dubin* node)=0;
};

class GridChecker : public Checker{
public:  
    GridChecker(OccupancyGrid* grid);
    void getSuccessors(Dubin* initial, vector<Dubin*>& successors); // When not using heuristic
	void getSuccessors(Dubin* initial, vector<Dubin*>& successors, float goalx, float goaly); 
	// Use heuristic
	int getAddress(Dubin* node); // Return map address of a node for discretizing space

private:
	void setHeuristic(Dubin* node, float goalx, float goaly);
    bool checkBound(Dubin* node);
    OccupancyGrid* _grid;
	float _r, _v;
	static float _omega[];
};

#endif
