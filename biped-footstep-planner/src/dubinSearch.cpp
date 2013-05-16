#include "dubinSearch.h"
#include <iostream>
#include <algorithm>
#include "Checker.h"
#include "dubin.h"
#include <cstdio>
#include <cmath>
#include <map>

using namespace std;


DubinSearch::DubinSearch(){
    _queue.clear();
    _poppedNodes.clear();
    _expCount = 0;
}

bool DubinSearch::queueEmpty(){
    return _queue.empty();
}

bool compareDubin(Dubin* node1, Dubin* node2){
    return !(node1->getTotalCost() < node2->getTotalCost());
}

Dubin* DubinSearch::dequeue(){
    
    if (queueEmpty()) {
        cout<<"Yo Dawg, No trying to dequeue the empty queue!"<<endl;
    }

    else{
        Dubin* popped = _queue.front();
        pop_heap(_queue.begin(), _queue.end(), compareDubin);
        _queue.pop_back();

        _poppedNodes.push_back(popped);
        return popped;
    }
    return NULL;
}

void DubinSearch::enqueue(float x,
                    float y, 
                    float angle, 
                    Dubin* pred, 
                    float costToCome, 
                    float costToGo){
    Dubin* newNode = new Dubin(x, y, angle);
    newNode->predecessor = pred;
    newNode->costToCome = costToCome;
    newNode->costToGo = costToGo;

    _queue.push_back(newNode);
    push_heap(_queue.begin(), _queue.end(), compareDubin);

}

void DubinSearch::enqueue(Dubin* newNode){
    _queue.push_back(newNode);
    push_heap(_queue.begin(), _queue.end(), compareDubin);
}

DubinSearch::~DubinSearch(){
    while (!_queue.empty()){
        delete _queue.back();
        _queue.pop_back();
    }
    while (!_poppedNodes.empty()){
        delete _poppedNodes.back();
        _poppedNodes.pop_back();
    }
}

Dubin* DubinSearch::search(float initx, float inity, float initTheta,
                            float goalx, float goaly, float goalr, float goalTheta,
                            Checker* checker, int maxDepth, bool heuristic, float inflate){
    Dubin* initial = new Dubin(initx, inity, initTheta);
    initial->costToCome = 0;
	initial->costToGo = 0;
    return search(initial, goalx, goaly, goalr, goalTheta, checker, maxDepth, heuristic, inflate);
}



Dubin* DubinSearch::search(Dubin* initial,
                           float goalx, 
                           float goaly,
                           float goalr,
                           float goalTheta,
                           Checker* checker, int depth, 
						   bool heuristic, float inflate){
    
	map<int, Dubin*> m;
	map<int, Dubin*>::const_iterator it;
	enqueue(initial);
	m[checker->getAddress(initial)] = initial;
    Dubin* curr;
	Dubin* next;
    vector<Dubin*> succ;
	int addr;
	int searchDepth = 0;
	bool isBad;
    while (!_queue.empty()){
        curr = dequeue();
        _expCount++;
		// Check for reaching the goal
        if(((curr->x-goalx) * (curr->x-goalx) +
            (curr->y-goaly) * (curr->y-goaly) < goalr * goalr)&& (abs(goalTheta-curr->theta)<.05)){
			cout<<"total cost is "<<curr->costToCome<<endl;
            return curr;
        }
		// Kill nodes on the other branch
		if (int(curr->depth)==(searchDepth+depth)){
            searchDepth++;
			std::vector<Dubin*> good; // new queue of only nodes in the good branch
			good.push_back(curr);
			push_heap(good.begin(), good.end(), compareDubin);
			Dubin* goodAnce = curr;
			for (int i=0; i<depth-1; i++){
				goodAnce = goodAnce->predecessor;
			}
			Dubin* nextNode;
			Dubin* ancestor;
			int count1=0;
			int count2=0;
			while (!_queue.empty()){
				nextNode = _queue.front();
				pop_heap(_queue.begin(), _queue.end(), compareDubin);
				_queue.pop_back();
				ancestor = nextNode;
				isBad = true;
				while (ancestor!=NULL){
					if (ancestor==goodAnce){
						isBad = false;
					}
					ancestor = ancestor->predecessor;
				}
				if (isBad){
					_poppedNodes.push_back(nextNode);
					count1++;
				} else {
					good.push_back(nextNode);
					push_heap(good.begin(), good.end(), compareDubin);
					count2++;
				}
			}
			cout<<"depth "<<searchDepth<<" threw "<<count1<<" things away and kept "<<count2<<" things"<<endl;
			_queue = good;
			good.clear();
        } 
		// Get successors
		if (heuristic){
			checker->getSuccessors(curr, succ, goalx, goaly, goalTheta);
		} else {
			checker->getSuccessors(curr, succ);
		}
		// Add successors
        while(!succ.empty()){
			next = succ.back();
			next->costToGo = next->costToGo * inflate;
			addr = checker->getAddress(next);
			it = m.find(addr);
			if (it==m.end() || m[addr]->getTotalCost() > next->getTotalCost()){
				enqueue(next);
				m[addr] = next;
			} 
            // DEBUG (succ.back())->print();
            succ.pop_back();
        }
    }
        
    return NULL;
}

size_t DubinSearch::getCount(){
    return _expCount;
}

void DubinSearch::makeSVG(Dubin* goal, std::string filename, OccupancyGrid& grid, 
							float goalx, float goaly, float goalr, int zoom){

    //TODO
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

            if (grid(x,y)==0){
                f = 0;
            }
            else{
                f = 255;
            }
                    
            fprintf(svg, "  <rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" "
                "fill=\"#%02x%02x%02x\" />\n", 
                (int)x*zoom, (int)y*zoom, zoom, zoom, f, f, f);
        }
    }
	
	fprintf(svg, "  <circle cx=\"%f\" cy=\"%f\" r=\"%f\" stroke=\"red\" stroke-width=\"2\" fill=\"none\"/>\n",
				goalx*zoom, goaly*zoom, goalr*zoom);
   
    fprintf(svg, "  <g stroke=\"#bfbfbf\" stroke-width=\"1.5\">\n");

    // Draw nodes and lines expanded
    Dubin* curr;
    Dubin* pred;
    while (!_poppedNodes.empty()){
        curr = _poppedNodes.back();
        float cx = (curr->x + 0.5)*zoom;
        float cy = (curr->y + 0.5)*zoom;
        pred = curr->predecessor;
        float r = 0.2 * zoom;
        
        if(pred!=NULL){
            float cxp = (pred->x + 0.5)*zoom;
            float cyp = (pred->y + 0.5)*zoom;
			float r = curr->archRadius * zoom;
			if (r ==0) {
				fprintf(svg, "    <line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" %s/>\n",
					cx, cy, cxp, cyp, "");
			} else {
				fprintf(svg, "    <path d=\"M %f %f A %f %f 0 0 %d %f %f \" fill=\"none\"/>\n",
					cxp, cyp, fabs(r), fabs(r), int(r>0), cx, cy);
			}
				

        }

        fprintf(svg, "    <circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"%s\" %s/>\n",
                cx, cy, r, "#ffffff","");
        

        _poppedNodes.pop_back();
    }
	
	while (!_queue.empty()){
        curr = _queue.back();
        float cx = (curr->x + 0.5)*zoom;
        float cy = (curr->y + 0.5)*zoom;
        pred = curr->predecessor;
        float r = 0.5 * zoom;
        
        if(pred!=NULL){
            float cxp = (pred->x + 0.5)*zoom;
            float cyp = (pred->y + 0.5)*zoom;
			float r = curr->archRadius * zoom;
			if (r ==0) {
				fprintf(svg, "    <line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" %s/>\n",
					cx, cy, cxp, cyp, "");
			} else {
				fprintf(svg, "    <path d=\"M %f %f A %f %f 0 0 %d %f %f \" fill=\"none\"/>\n",
					cxp, cyp, fabs(r), fabs(r), int(r>0), cx, cy);
			}
				

        }
		fprintf(svg, "  </g>\n");
		fprintf(svg, "  <g stroke=\"#0000ff\" stroke-width=\"1\">\n");
        fprintf(svg, "    <circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"%s\" %s/>\n",
                cx, cy, r, "#ffffff","");
        fprintf(svg, "  </g>\n");
		fprintf(svg, "  <g stroke=\"#7f7f7f\" stroke-width=\"1\">\n");

        _queue.pop_back();
    }
	fprintf(svg, "  </g>\n");
	fprintf(svg, "  <g stroke=\"#7f007f\" stroke-width=\"3\">\n");
	// Draw found path
	curr = goal;
	while (curr!=NULL){
		float cx = (curr->x + 0.5)*zoom;
        float cy = (curr->y + 0.5)*zoom;
        pred = curr->predecessor;
        float r = 0.2 * zoom;
        
        if(pred!=NULL){
            float cxp = (pred->x + 0.5)*zoom;
            float cyp = (pred->y + 0.5)*zoom;
			float r = curr->archRadius * zoom;
			if (r ==0) {
				fprintf(svg, "    <line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" %s/>\n",
					cx, cy, cxp, cyp, "");
			} else {
				fprintf(svg, "    <path d=\"M %f %f A %f %f 0 0 %d %f %f \" fill=\"none\"/>\n",
					cxp, cyp, fabs(r), fabs(r), int(r>0), cx, cy);
			}
				

        }

        fprintf(svg, "    <circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"%s\" %s/>\n",
                cx, cy, r, "#ffffff","");
		curr = pred;
	}
	
    fprintf(svg, "  </g>\n");
    fprintf(svg, "</svg>\n");
    fclose(svg);
    
    cout<<"Made Debug Image"<<endl;
}
