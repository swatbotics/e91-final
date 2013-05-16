#ifndef _BIPED_CHECKER_H
#define _BIPED_CHECKER_H

#include "biped.h"
#include "OccupancyGrid.h"
#include <vector>

using namespace std;

#define PI 3.14159265
typedef vec2u<size_t> GridNode;

class BipedChecker{
public:
	BipedChecker(OccupancyGrid<unsigned char>* grid, float goalx, float goaly, float inflate_h, float inflate_z);
	void createHeuristic(float goalx, float goaly, float inflate_h, float inflate_z);
		// When using heuristic
	void getSuccessors(biped* initial, vector<biped*>& successors);
	long getAddress(biped* node);
	OccupancyGrid<unsigned char> getHeur();
	
private:
	OccupancyGrid<unsigned char>* _grid;
	OccupancyGrid<float>* _heur;
	
	void setHeuristic(biped* node);
	bool checkBound(biped* node);
	bool checkStep(biped* node);
	void makeMask(OccupancyGrid<unsigned char>& mask, int r);
	



};

#endif