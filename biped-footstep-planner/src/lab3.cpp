#include "lab3.h"
#include <math.h>
#include <stdlib.h>
#include <iostream>
#include <assert.h>
#include <stdio.h>

using namespace std;

/* This function should initialize the OccupancyGrid dst to be the
 * same size as OccupancyGrid src, with all of the obstacles that src
 * has. In addition, any previously unoccupied cell (i.e. one whose
 * value is greater than or equal to 128) that lies a distance less
 * than or equal to radius of an occupied cell should be set to a
 * value of 127.
 */
void expandObstacles(const OccupancyGrid& src,
                     float radius,
                     OccupancyGrid& dst,
			bool expand) {

  // TODO: writeme!
  // resize the new grid
  dst.resize(src.dims());

  // fill in the new grid
 // unsigned char fill = 0;
  int r2 = ceil(radius*radius);

  for (size_t y=0; y<src.ny(); y++){
    for (size_t x=0; x<src.nx(); x++){
      
      dst(x,y)=255;

      for (int u=-ceil(radius); u<=ceil(radius); u++){
	for (int v=-ceil(radius); v<=ceil(radius); v++){
  	   
	  if (y+u>=0&&y+u<src.ny()&&x+v>=0&&x+v<src.nx()&&u*u+v*v<=r2){
	    if (src(x+v,y+u)!=255){
	       if (expand){
	         dst(x,y) = min(int(dst(x,y)),src(x+v,y+u)*(r2-u*u-v*v)/r2+255*(u*u+v*v)/r2);
               }
               else{
	         dst(x,y)=0;
               }
	    }
	  }
	}
      } 
    }
  }


}

/* This function should implement Dijkstra's or A* to search the
 * occupancy grid provided for a collision-free from init to goal.
 *
 * A grid cell is determined to be collision free if the value stored
 * in the OccupancyGrid at that cell is greater than or equal to 127.
 *
 * A robot at any free position on the grid should be allowed to move
 * to any of the eight neighboring cells, provided the destination
 * cell is also free space.
 *
 * If the useheuristic parameter is false, then the costToGo of all
 * nodes is set to zero when enqueing them, thus implementing
 * Dijkstra's algorithm.
 *
 * If the useheuristic parameter is true, then the costToGo at some
 * position p is set to be the Euclidean distance from that position
 * to the goal, thus implementing A*.
 */
bool search(const OccupancyGrid& grid,
		const vec2u& init,
		const vec2u& goal,
		GridSearchData& helper, 
		bool useheuristic,
		bool truncate,
		float inflate) {

	helper.reset();
	helper.initialize(grid);

	// TODO: writeme!
	GridNode curr;


	helper.enqueue(init, init, 0, 0);

	while (!helper.queueEmpty()){
		curr = helper.dequeue();
		assert(curr.type == NODE_DEAD);
		if (curr.position == goal){
			return true;
		}
		const int dx[8] = {-1, 0, 1, -1, 1, -1,  0,  1};
		const int dy[8] = { 1, 1, 1,  0, 0, -1, -1, -1};

		for (int dir=0; dir<8; dir++){
			vec2u next(curr.position.x()+dx[dir],curr.position.y()+dy[dir]);

			if (next.x()>=0&&next.x()<grid.nx()&&next.y()>=0&&next.y()<grid.ny()&&grid(next)!=0){
				float costToGo = 0;
				if (useheuristic){
					//costToGo = sqrt(pow(next.x()-goal.x(),2)+pow(next.y()-goal.y(),2));
					int dx = abs(next.x() - goal.x());
					int dy = abs(next.y() - goal.y());
					//costToGo = sqrt(dx*dx + dy*dy);
					costToGo = std::min(dx,dy)*sqrt(2)+std::max(dx,dy)-std::min(dx,dy);
					costToGo = inflate*costToGo;
				}
				GridNode n = helper.lookup(next);
				float k = curr.costToCome+sqrt(dx[dir]*dx[dir]+dy[dir]*dy[dir])+(255-grid(next))/255 ;
				if (k<n.costToCome-0.5||n.type==NODE_UNVISITED){
					helper.enqueue(next, curr.position, k, costToGo);
				}
			}
			if (truncate && next == goal){
				return true;
			}
		}


	}
	return false;
}
