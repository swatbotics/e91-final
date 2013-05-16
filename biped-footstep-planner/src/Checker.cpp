#include "Checker.h"
#include "OccupancyGrid.h"
#include <math.h>

using namespace std;

#define PI 3.14159265
float GridChecker::_omega[] = {-PI/4,-PI/6,-PI/8,0,PI/8,PI/6,PI/4};

GridChecker::GridChecker(OccupancyGrid* grid){

    _grid = grid;
	// Define parameters for the car
    // Define size
    _r = 2; // radius of car is 4

     // Define moves
    _v = 2;

}

int GridChecker::getAddress(Dubin* node){
	int x = int(node->x+0.5);
	int y = int(node->y+0.5);
	int theta = int((node->theta+7.5) / 15) % 24;
	return (x*int(_grid->nx())*24 + y*24 + theta);
}

bool GridChecker::checkBound(Dubin* node){
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
    for (int u=-ceil(_r); u<= ceil(_r); u++){
        for (int v=-ceil(_r); v<=ceil(_r); v++){
			
            if (x+u>=0 && y+v>=0 && x+u<sizex && y+v<sizey && 
                    u*u+v*v<_r*_r) {
					if ((*_grid)(x+u,y+v)!=255){
				
						/* DEBUG cout<<x<<" "<<y<<" failed because at x="<<x+u
							<<" and y="<<y+v<<", "<<int((*_grid)(x+u,y+v))<<endl; */
						return false;
					}		
			}
        }
    }
    
    return true;
}

void GridChecker::setHeuristic(Dubin* node, float goalx, float goaly, float goalTheta){
	// Cartesian
	// node->costToGo = sqrt((node->x-goalx)*(node->x-goalx) + (node->y-goaly)*(node->y-goaly) ); 
	// return;
	// Dubin
	// Apply translation of (-x,-y), then rotation of -theta
	float gx =  cos(node->theta) * (goalx-node->x) + sin(node->theta) * (goaly-node->y);
	float gy = -sin(node->theta) * (goalx-node->x) + cos(node->theta) * (goaly-node->y);
	float radius = fabs(_v * _omega[0]);
	float dl2 = gx*gx + (gy-radius)*(gy-radius); // If we turn left, distance from goal to turing center
	float dr2 = gx*gx + (gy+radius)*(gy+radius); // If we turn right, distance from goal to turing center
	float dAngle = abs(goalTheta-node->theta)*.1;
	if ((gy>=0 && dl2-radius*radius>=0) || (gy<0 && dr2-radius*radius<0)){ // Left-Straight
		float s = sqrt(dl2-radius*radius); // length of straight path
		float alpha = atan2(s,radius); // Angle between d and r
		float beta = atan2(gy-radius,gx); // Angle of goal from turning center
		float theta = beta - alpha + PI/2; // Angle or turing
		if (theta<0){
			theta += 2*PI;
		}
		node->costToGo = s + theta * radius + dAngle;
	} else { // Right-Straight
		float s = sqrt(dr2-radius*radius); // length of straight path
		float alpha = atan2(s,radius); // Angle between d and r
		float beta = atan2(gy+radius,gx); // Angle of goal from turning center
		float theta = - beta - alpha + PI/2; // Angle or turing
		if (theta<0){
			theta += 2*PI;
		}
		node->costToGo = s + theta * radius + dAngle;
	}
}

void GridChecker::getSuccessors(Dubin* initial, vector<Dubin*>& successors, float goalx, float goaly, float goalTheta){
	getSuccessors(initial, successors);
	for (unsigned int i=0; i<successors.size(); i++){
		setHeuristic(successors.at(i),goalx,goaly, goalTheta);
	}
	return;
}

void GridChecker::getSuccessors(Dubin* initial, vector<Dubin*>& successors){

    successors.clear();

	float timeStep = 2;
	int steps = 5;
	float dt = timeStep/steps;

    if (!checkBound(initial)){
        cout<<"Initial node not valid"<<endl;
    }

    // For each move
    Dubin* next;
	Dubin* curr;
    float newTheta, newx, newy, radius;
    for (int i=0; i<7; i++){
     // Calculate successor
		int j;
		curr = initial;
		for (j=0; j<steps; j++){
			if (_omega[i]==0) {
				newx = curr->x + cos(curr->theta) * _v * dt;
				newy = curr->y + sin(curr->theta) * _v * dt;
				newTheta = curr->theta;
				radius = 0;
			} else {
				newTheta = curr->theta + dt * _omega[i];
				newx = curr->x + _v*( sin(newTheta) - sin(curr->theta)) / _omega[i];
				newy = curr->y + _v*(-cos(newTheta) + cos(curr->theta)) / _omega[i];
				radius = _v/_omega[i];
			}
		
		 // Angle correction for wrapping around
			while (newTheta>=2*PI){
				newTheta = newTheta-2*PI;
			}
			while (newTheta<0){
				newTheta = newTheta+2*PI;
			}
		 // Check for validity and add
			next = new Dubin(newx, newy, newTheta);
			if (!checkBound(next)) {break;}
			curr = next;
		}
        if (j==steps){
            next->setPred(initial,radius);
            next->costToCome = initial->costToCome + _v*timeStep;
            next->costToGo = 0;
            successors.push_back(next);
        }
        else{
            delete next;
        }
    }
     

}
