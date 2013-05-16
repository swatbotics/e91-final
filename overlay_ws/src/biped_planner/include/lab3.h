#ifndef _LAB3_H_
#define _LAB3_H_

#include "GridSearch.h"

/* This function should initialize the OccupancyGrid dst to be the
 * same size as OccupancyGrid src, with all of the obstacles that src
 * has. In addition, any previously unoccupied cell (i.e. one whose
 * value is greater than or equal to 128) that lies a distance less
 * than or equal to radius of an occupied cell should be set to a
 * value of 127.
 *
 * You should complete this function in src/lab3.cpp.
 */
void expandObstacles(const OccupancyGrid& src,
                     float radius,
                     OccupancyGrid& dst,
			bool expand);

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
 *
 * You should complete this function in src/lab3.cpp.
 */
bool search(const OccupancyGrid& grid,
            const vec2u& init,
            const vec2u& goal,
            GridSearchData& helper,
            bool useheuristic,
		bool truncate,
		float inflate);

             

#endif
