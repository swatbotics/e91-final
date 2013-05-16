#ifndef _BIPEDSEARCH_H_
#define _BIPEDSEARCH_H_

#include "biped_checker.h"

class bipedSearch{
private:
    typedef std::vector<biped*> NodeArray;

    NodeArray _queue;
    size_t _expCount;
    NodeArray _poppedNodes;
    NodeArray _allocated;
    

public:

  
    bipedSearch();


    void clear();
    bool queueEmpty();
    void enqueue(float x, float y, float angle, foot ft, biped* pred, float costToCome, float costToGo);
    void enqueue(biped* newNode);
    biped* dequeue();
    ~bipedSearch();
	biped* search(float x, float y, float angle, float goalx, float goaly, float goalr, float goalTheta, BipedChecker* checker, int maxDepth, int viewDepth);
	biped* search(biped* initial, float goalx, float goaly, float goalr, float goalTheta, BipedChecker* checker, bool heuristic, int maxDepth, int viewDepth);

    size_t getCount();
    void makeSVG(biped* goal, std::string filename, OccupancyGrid<unsigned char> grid, float goalx, float goaly, float goalr, bool path, int zoom=12);
    
};

#endif
