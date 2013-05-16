#include "bipedSearch.h"
#include <math.h>
#include <algorithm>
#include <map>

bipedSearch::bipedSearch(){ clear(); }

bool bipedSearch::queueEmpty(){
  return _queue.empty();
}

bool comparebiped(biped* node1, biped* node2){
  return !(node1->totalCost() < node2->totalCost());
}

biped* bipedSearch::dequeue(){
  if (queueEmpty()) {
    cout<<"Yo Dawg, No trying to dequeue the empty queue!"<<endl;
    }
  else{
    biped* popped = _queue.front();
    pop_heap(_queue.begin(), _queue.end(), comparebiped);
    _queue.pop_back();
    _poppedNodes.push_back(popped);
    return popped;
  }
  return NULL;
}

void bipedSearch::enqueue(float x,
			  float y, 
			  float angle, 
			  foot ft,
			  biped* pred, 
			  float costToCome, 
			  float costToGo){
  biped* newNode = new biped(x, y, angle, ft);
  _allocated.push_back(newNode);
  newNode->pred = pred;
  newNode->costToCome = costToCome;
  newNode->costToGo = costToGo;
  enqueue(newNode);
  return;
}

void bipedSearch::enqueue(biped* newNode){
  _queue.push_back(newNode);
  push_heap(_queue.begin(), _queue.end(), comparebiped);
  return;
}

bipedSearch::~bipedSearch(){
  clear();
}

void bipedSearch::clear() {
  while (!_allocated.empty()){
    delete _allocated.back();
    _allocated.pop_back();
  }
  _queue.clear();
  _poppedNodes.clear();
  _expCount = 0;
}


size_t bipedSearch::getCount(){
  return _expCount;
}

biped* bipedSearch::search(float x, float y, float angle,
			   float goalx, float goaly, float goalr, float goalTheta, 
			   BipedChecker* checker, int maxDepth, int viewDepth){
  clear();
  biped* initial = new biped(x, y, angle, LEFT);
  _allocated.push_back(initial);
  initial->pred = NULL;
  initial->costToCome = 0;
  initial->costToGo = 0;
  return search(initial, goalx, goaly, goalr, goalTheta, checker, true, maxDepth, viewDepth);
}

float angleDiff(float a, float b) {
	float d = fabs(b-a);
	while (d > M_PI) { d = fabs(d - 2*M_PI); }
	return d;
}

biped* bipedSearch::search(biped* initial,
			   float goalx,
			   float goaly,
			   float goalr,
			   float goalTheta,
			   BipedChecker* checker, 
			   bool heuristic, 
			   int maxDepth, 
			   int viewDepth){


  map<long, biped*> m;
  map<long, biped*>::const_iterator it;
  m[checker->getAddress(initial)] = initial;
  int addr;
  enqueue(initial);
  biped* curr;
  biped* next;
  vector<biped*> succ;
  int searchDepth = 0;
  bool isBad;
  while(!_queue.empty()){
    curr = dequeue();
    _expCount++;
    // TODO Check for goalTheta
    float mx = curr->x;
    float my = curr->y;
    if (curr->pred) { 
    	mx = 0.5*mx + 0.5*curr->pred->x;
    	my = 0.5*my + 0.5*curr->pred->y;
    }
    if(((mx-goalx) * (mx-goalx) +
       (my-goaly) * (my-goaly) < goalr * goalr)&&(angleDiff(curr->theta, goalTheta))<10*M_PI/180){ // TODO: don't hardcode
      return curr;
    }
    if (curr->depth > (size_t)maxDepth){
      continue;
    }
    // Horizon
    if (int(curr->depth)==(searchDepth+viewDepth)){
      searchDepth++;
      std::vector<biped*> good; // new queue of only nodes in the good branch
      good.push_back(curr);
      push_heap(good.begin(), good.end(), comparebiped);
      biped* goodAnce = curr;
      for (int i=0; i<viewDepth-1; i++){
	goodAnce = goodAnce->pred;
      }
      biped* nextNode;
      biped* ancestor;
      while (!_queue.empty()){
	nextNode = _queue.front();
	// comparebiped is passed in as a function for sorting
	pop_heap(_queue.begin(), _queue.end(), comparebiped); 
	_queue.pop_back();
	ancestor = nextNode;
	isBad = true;
	while (ancestor!=NULL){
	  if (ancestor==goodAnce){
	    isBad = false;
	  }
	  ancestor = ancestor->pred;
	}
	if (isBad){
	  _poppedNodes.push_back(nextNode);
	} else {
	  good.push_back(nextNode);
	  push_heap(good.begin(), good.end(), comparebiped);
	}
      }
      _queue = good;
      good.clear();
    } 
    // Get successors
    checker->getSuccessors(curr, succ);
    // Add successors
    while(!succ.empty()){
      next = succ.back();
      float extra = angleDiff(next->theta, goalTheta) * 3; // TODO: don't hardcode
      next->costToGo = next->costToGo + extra;
      addr = checker->getAddress(next);
      it = m.find(addr);
      if (it==m.end() || m[addr]->totalCost() > next->totalCost()){		
	enqueue(next);
	m[addr] = next;
      } 
      succ.pop_back();
    }
  }
  return NULL;
}


void bipedSearch::makeSVG(biped* goal, std::string filename, OccupancyGrid<unsigned char> grid, 
							float goalx, float goaly, float goalr, bool path, int zoom){



	FILE* svg = fopen(filename.c_str(), "w");
    if (!svg) {
        std::cerr << "error opening svg for output! \n";
        return;
    }

    int width = grid.nx() * zoom;
    int height = grid.ny() * zoom;

    fprintf(svg, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
    fprintf(svg, "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
    fprintf(svg, "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
    fprintf(svg, "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\"\n");
    fprintf(svg, "     xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n");
    fprintf(svg, "     width=\"%upx\" height=\"%upx\" viewBox=\"0 0 %u %u\" >\n", 
          width, height, width, height);

    // Draw background and obstacle
    int f;
    for (size_t y=0; y<grid.ny(); ++y) {
        for (size_t x=0; x<grid.nx(); ++x) {

            f = grid(x,y);
                    
            fprintf(svg, "  <rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" "
                "fill=\"#%02x%02x%02x\" />\n", 
                (int)((x-.5)*zoom), (int)((y-.5)*zoom), zoom, zoom, f, f, f);
        }
    }
	
	fprintf(svg, "  <circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke=\"red\" stroke-width=\"2\" fill=\"none\"/>\n",
				goalx*zoom, goaly*zoom, goalr*zoom);
   
    fprintf(svg, "  <g stroke=\"#bfbfbf\" stroke-width=\"1.5\">\n");

    // Draw nodes and lines expanded
    biped* curr;
	// Draw found path
	if (path){
		while(!_poppedNodes.empty()){
			curr = _poppedNodes.back();
			fprintf(svg, "    <circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"%s\" %s/>\n",
                curr->x*zoom, curr->y*zoom, .2*zoom, "#007f00","");
			_poppedNodes.pop_back();
		}
		curr = goal;
		while (curr!=NULL){
			curr->draw(svg, zoom, 4, 2);
			curr = curr->pred;
		}
	} else {
		while(!_poppedNodes.empty()){
			curr = _poppedNodes.back();
			curr->draw(svg, zoom, 4, 2);
			_poppedNodes.pop_back();
		}		
	}
	
    fprintf(svg, "  </g>\n");
    fprintf(svg, "</svg>\n");
    fclose(svg);
    
    cout<<"Made Debug Image"<<endl;
}
