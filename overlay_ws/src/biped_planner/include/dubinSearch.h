#ifndef _DUBINSEARCH_H_
#define _DUBINSEARCH_H_

#include "OccupancyGrid.h"
#include "dubin.h"
#include "Checker.h"
#include <climits>

class DubinSearch{
private:
    typedef std::vector<Dubin*> NodeArray;

    NodeArray _queue;
    size_t _expCount;
    NodeArray _poppedNodes;
    

public:

    DubinSearch();
    bool queueEmpty();
    void enqueue(float x, float y, float angle, Dubin* pred, float costToCome, float costToGo);
    void enqueue(Dubin* newNode);
    Dubin* dequeue();
    ~DubinSearch();
    Dubin* search(float initx, float inity, float initTheta, float goalx, float goaly, float goalr, 
		Checker* checker, int depth, bool heuristic, float inflate);
    Dubin* search(Dubin* initial, float goalx, float goaly, float goalr, Checker* checker, int depth, 
		bool heuristic, float inflate);

    size_t getCount();
    void makeSVG(Dubin* goal, std::string filename, OccupancyGrid& grid, float goalx, float goaly, float 		goalr, int zoom=12);
    
};

#endif
