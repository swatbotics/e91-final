#include "biped_checker.h"
#include <math.h>
#include <algorithm>

using namespace std;


BipedChecker::BipedChecker(OccupancyGrid<unsigned char>* grid, float goalx, float goaly, float inflate_h, float inflate_z){
  _grid = grid;
  createHeuristic(goalx, goaly, inflate_h, inflate_z);
}


class CompareGrid {
public:
  
  const OccupancyGrid<float>& _heur;
  
  CompareGrid(const OccupancyGrid<float>& n): _heur(n) {}
  
  bool operator()(GridNode a, GridNode b) {
    return _heur(a) > _heur(b);
  }
  
};


void BipedChecker::makeMask(OccupancyGrid<unsigned char>& mask, int r){
  // Dilate the boundaries
  OccupancyGrid<unsigned char> temp = OccupancyGrid<unsigned char>();
  temp.resize(_grid->dims());
  for (size_t x=0; x<_grid->nx(); x++){
    for (size_t y=0; y<_grid->ny(); y++){
      temp(x,y)=1;
      for (int u=-r; u<=r; u++){
	for (int v=-r; v<=r; v++){
	  if (x+u>=0 && y+v>=0 && x+u<_grid->nx() && y+v<_grid->ny()) {
	    if ((*_grid)(x+u,y+v)==0){
	      temp(x,y)=0;
	    }
	  }
	}
      }
    }
  }
  // Erode the boundaries
  for (size_t x=0; x<_grid->nx(); x++){
    for (size_t y=0; y<_grid->ny(); y++){
      mask(x,y)=0;
      for (int u=-r; u<=r; u++){
	for (int v=-r; v<=r; v++){
	  if (x+u>=0 && y+v>=0 && x+u<_grid->nx() && y+v<_grid->ny()) {
	    if (temp(x+u,y+v)==1){
	      mask(x,y)=1;
	    }
	  }
	}
      }
    }
  }
}


void BipedChecker::createHeuristic(float goalx, float goaly, float inflate_h, float inflate_z){
  
  const int dx[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };
  const int dy[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };
  
  OccupancyGrid<unsigned char> mask = OccupancyGrid<unsigned char>();
  mask.resize(_grid->dims());
  makeMask(mask, 3);
  _heur = new OccupancyGrid<float>();
  _heur->resize(_grid->dims());
  if (goalx<0 || goalx>=_grid->nx() || goaly<0 || goaly>=_grid->ny()){
    cout<<"Goal is not inside image of size "<<_grid->nx()<<" by "<<_grid->ny()<<endl;
    exit(-1);
  }
  // Initialize all heuristics to -1
  for (size_t i=0; i<_heur->nx(); i++){
    for (size_t j=0; j<_heur->ny(); j++){
      (*_heur)(i,j) = -1;
    }
  }
  // Make a queue
  std::vector<GridNode> queue;
  // Add goal to queue
  GridNode goal = GridNode((int)(goalx+.5), (int)(goaly+.5));
  (*_heur)(goal) = 0;
  queue.push_back(goal);
  push_heap(queue.begin(), queue.end(), CompareGrid(*_heur));
  float newHeur, height;
  // while queue not empty
  while (!queue.empty()){	
    GridNode curr = queue.front();
    pop_heap(queue.begin(), queue.end(), CompareGrid(*_heur));
    queue.pop_back();
    // add neighbors to queue with correct heuristic
    for (size_t i=0; i<8; i++){
      GridNode next = GridNode(curr.x()+dx[i], curr.y()+dy[i]);
      if (next.x()>=0 && next.x()<_grid->nx() && next.y()>=0 && next.y()<_grid->ny()){
	height = abs((*_grid)(next)-(*_grid)(curr));
	
	newHeur = (*_heur)(curr) + inflate_h*sqrt(dx[i]*dx[i]+dy[i]*dy[i]) + inflate_z*height*height/10;
	
	if (mask(next)!=0 && abs((*_grid)(next)-(*_grid)(curr))<30 && ( (*_heur)(next)>newHeur || (*_heur)(next)==-1)){
	  (*_heur)(next) = newHeur;
	  queue.push_back(next);
	  push_heap(queue.begin(), queue.end(), CompareGrid(*_heur));				
	}
      }
    }
  }
  //end
}

long BipedChecker::getAddress(biped* node){
  long x = int(node->x+0.5);
  long y = int(node->y+0.5);
  long theta = int((node->theta+PI/32) / (PI/16)) % 16;
  int address = x*int(_grid->nx())*16 + y*16 + theta;
  address = address * 2 + (node->ft==LEFT ? 0:1);
  return address;	
}

bool BipedChecker::checkBound(biped* node){
  // Check if position is in bounds
  int x = int(node->x+.5);
  int y = int(node->y+.5);
  int sizex = int(_grid->nx());
  int sizey = int(_grid->ny());
  if (x<0 || x>sizex || y<0 || y>sizey){
    // DEBUG cout<<x<<" "<<y<<"failed because out of bounds"<<endl;
    return false;
  }
  // Check if position is in/too close to walls
  // Here we're assuming that the foot is 2x4, thus a radius of sqrt(5)
  for (int u=-3; u<=3; u++){
    for (int v=-3; v<=3; v++){
      if (fabs(cos(node->theta)*u+sin(node->theta)*v)<2 && fabs(cos(node->theta)*v-sin(node->theta)*u)<1.5){
	if (x+u>=0 && y+v>=0 && x+u<sizex && y+v<sizey) {
	  if ((*_grid)(x+u,y+v)==0){
	    return false;
	  }		
	}
      }
    }
  }  
  return true;
}

bool BipedChecker::checkStep(biped* node){
  // Check if position is valid step
  int x = int(node->x+.5);
  int y = int(node->y+.5);
  int sizex = int(_grid->nx());
  int sizey = int(_grid->ny());
  if (x<0 || x>sizex || y<0 || y>sizey){
    // DEBUG cout<<x<<" "<<y<<"failed because out of bounds"<<endl;
    return false;
  }
  // Check if position is in/too close to walls
  // Here we're assuming that the foot is 2x4, thus a radius of sqrt(5)
  int height = 0;
  float correction = sqrt(2)*max(fabs(sin(node->theta-PI/4)),fabs(cos(node->theta-PI/4)))/2;
  //	cout<<correction<<" for "<<node->theta<<endl;
  //	int count=0;
  for (int u=-3; u<=3; u++){
    for (int v=-3; v<=3; v++){
      if (fabs(cos(node->theta)*u+sin(node->theta)*v)<2+correction && fabs(cos(node->theta)*v-sin(node->theta)*u)<1+correction){
	if (x+u>=0 && y+v>=0 && x+u<sizex && y+v<sizey) {
	  //					count++;
	  //					cout<<"checked "<<count<<" things"<<endl;
	  if (height==0){
	    height = (*_grid)(x+u,y+v);
	  }
	  if ((*_grid)(x+u,y+v)!=height || (*_grid)(x+u,y+v)==0){
	    return false;
	  }		
	}
      }
    }
  }
  
  return true;
}




void BipedChecker::setHeuristic(biped* node){
  int x = int(node->x+.5);
  int y = int(node->y+.5);
  node->costToGo = (*_heur)(x,y);
  return;
}

void BipedChecker::getSuccessors(biped* initial, vector<biped*>& successors){
  if (!checkStep(initial)){
    cout<<"Expanding an invalid node... ignored"<<endl;
    return;
  }
  successors.clear();
    //float dy = -4; // y distance between left and right foot
  float dy[14] = {-4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -4, -6, -5};
  if (initial->ft == LEFT) {
  	for (int i=0; i<14; i++) {
  		dy[i] = -1 * dy[i];
  	}  
  }
  //float dx[9] = {0,0,0,1,1,1,3,3,3}; // x distance, positive is forward
  //float dq[9] = {-PI/4,0,PI/4,-PI/4,0,PI/4,-PI/4,0,PI/4}; // angle change
  //float dx[9] = {0, 0.3, 0.6, 0.9, 1.2, 1.5, 1.8, 2.1, 2.4};
  //float dx[9] = {0, 0, 0, 1, 1, 1, 4.5, 4.5, 4.5 };
  float dx[14] = {4, 4, 4, 2, 2, 2, 3, 3, 3, 1, 1, 1, 0, 0};
  float dq[14] = {PI/12,0,PI/8,PI/12,0,PI/8,PI/12,0,PI/8, PI/12, 0, PI/8, 0, 0};
  float stair[3] = {0,10,40}; // Cost of going up stairs.
  //float cost[9] = {2,2,2,2.5,2,2.5,2.5,2,2.5};
  float cost[14] = {2, 2, 2, 1, 1, 1, 1.5, 1.5, 1.5, 1, 1, 1, 2, 1};
  int steps = 3;
  biped* next = NULL;
  biped* inter;
  biped* prev;
  float newx, newy, newq;
  int height, height1, height2;
  for (int i=0; i<14; i++){ // Go through each move
    newx = initial->x + cos(initial->theta)*dx[i] - sin(initial->theta)*dy[i];
    newy = initial->y + sin(initial->theta)*dx[i] + cos(initial->theta)*dy[i];
    newq = initial->theta + dq[i]*(initial->ft == LEFT ? -1 : 1);
    next = new biped(newx, newy, newq, (initial->ft == LEFT) ? RIGHT:LEFT);
    if (!checkStep(next)) {continue;}
    prev = initial->pred;
    if (prev!=NULL){
      int k;
      for (k=0; k<steps; k++){
	     inter = new biped((k*newx + (steps-k)*prev->x)/steps, 
			  (k*newy + (steps-k)*prev->y)/steps, 
			  (k*newq + (steps-k)*prev->theta)/steps, (initial->ft == LEFT) ? RIGHT:LEFT);
	if (!checkBound(inter)) {break;}
      }
      if (k!=steps) {continue;}
    }	
    while (newq>=2*PI){newq = newq-2*PI;} while (newq<0){newq = newq+2*PI;}		
    /*curr = initial;
      int k;
      for (k=0; k<steps; k++){	// Step through to make sure no passing through wall
      newx = curr->x + cos(curr->theta)*dx[i]*dt*(k+1) - sin(curr->theta)*dy*dt*(k+1);
      newy = curr->y + sin(curr->theta)*dx[i]*dt*(k+1) + cos(curr->theta)*dy*dt*(k+1);
      // Check for validity and add
      if (k!=0) {delete next;}
      next = new biped(newx, newy, 0, (curr->ft == LEFT) ? RIGHT:LEFT);
      if (!checkBound(next)) {break;}
      }
      if (!checkStep(next)) {continue;}
      newq = curr->theta + dq[i];
      // Angle correction for wrapping around
      while (newq>=2*PI){
      newq = newq-2*PI;
      }
      while (newq<0){
      newq = newq+2*PI;
      }	*/
    height1 = (*_grid)((int)(next->x+.5),(int)(next->y+.5));
    height2 = (*_grid)((int)(initial->x+.5),(int)(initial->y+.5));
    height = abs(height1-height2)/10; 		
    if (height<3){
      next->theta = newq;
      next->depth = initial->depth + 1;
      next->pred = initial;
      next->costToCome = initial->costToCome + cost[i] + stair[height];
      next->costToGo = 0;
      setHeuristic(next);
      successors.push_back(next);
    } else {
      delete next;
    }
    
  }
  return;
}

OccupancyGrid<unsigned char> BipedChecker::getHeur(){
  OccupancyGrid<unsigned char> output = OccupancyGrid<unsigned char>();
  output.resize(_grid->dims());
  for (size_t i=0; i<_grid->nx(); i++){
    for (size_t j=0; j<_grid->ny(); j++){
      output(i,j) = (unsigned char)((*_heur)(i,j)*2);
    }
  }
  return output;
}
