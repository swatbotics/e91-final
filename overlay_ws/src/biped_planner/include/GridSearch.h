#ifndef _GRIDSEARCH_H_
#define _GRIDSEARCH_H_

#include "OccupancyGrid.h"

//////////////////////////////////////////////////////////////////////

/* An enumeration for the the status of a vertex on the graph. */
enum GridNodeType {

  /* Invalid position - beyond dimensions of grid?. */
  NODE_INVALID,

  /* Not yet visited. */
  NODE_UNVISITED,

  /* In the queue. */
  NODE_IN_QUEUE,

  /* Removed from the queue. */
  NODE_DEAD

};

//////////////////////////////////////////////////////////////////////

/* A struct to keep track of data about states in the world. */
struct GridNode {

  /* The current type of node, see above. */
  GridNodeType type;

  /* The position of the node in the map. */
  vec2u position;

  /* The predecessor of the node.  If there is no predecessor
   * (i.e. for the initial position), this should be set identical to
   * position above. 
   */
  vec2u predecessor;

  /* Optimal cost to get to this position. */
  float costToCome;

  /* Heuristic cost-to-go estimate. */
  float costToGo;

  /* Constructor sets type to NODE_INVALID, costToCome to a very large
     number, and costToGo to zero. */
  GridNode();

  /* Returns costToCome + costToGo */
  float totalCost() const;

};

/* Typedef for a vector of GridNode structs. */
typedef std::vector<GridNode> GridNodeArray;

//////////////////////////////////////////////////////////////////////

/* A class to help with implementing A* search on a grid. Notably, it
 * implements a priority queue, and it also lets you query the state
 * (UNVISITED, IN_QUEUE, DEAD) of positions in a grid.
 */
class GridSearchData {
public:

  /* Constructor simply calls reset(). */
  GridSearchData();

  /* Prepare to do a search on the grid provided.  This initializes
   * the correct number of GridNode structs to hold information about
   * the OccupancyGrid provided, and clears the queue.  Initially, all
   * GridNode structs are set to have the correct position, no
   * predecessor, type UNVISITED, costToCome of a very large number,
   * and costToGo of zero.
   */
  void initialize(const OccupancyGrid& grid);

  /* Forget all the information about the current OccupancyGrid. */
  void reset();

  /* True if there are no nodes in the queue. */
  bool queueEmpty() const;

  /* Enqueue a node with the given position, predecessor, costToCome,
   * and costToGo. Note that if the node has no predecessor (i.e. the
   * initial position), you should pass in the same value for both
   * position and predecessor. After calling this method, calling
   * lookup() with the position provided will return the information
   * provided.  Also, the type of the node will be NODE_IN_QUEUE.
   *
   * Calling this function is the only way to affect the state of
   * the internal array of GridNode structs maintained by the
   * GridSearchData class.
   */
  void enqueue(const vec2u& position,
               const vec2u& predecessor,
               float costToCome,
               float costToGo);
  
  /* Returns the node in the queue with least totalCost(). If the
   * queue is empty, returns a node of type NODE_INVALID. Otherwise,
   * the type is set to NODE_DEAD.  Note that subsequently modifying
   * the GridNode returned by this function has no effect on future
   * calls to lookup() or dequeue(). (See enqueue() above).
   */
  GridNode dequeue();

  /* Returns the node associated with the given position. Note that
   * subsequently modifying the GridNode returned by this function has
   * no effect on future calls to lookup() or dequeue().  (See
   * enqueue() above).
   */
  GridNode lookup(const vec2u& position) const;

  /* Generate an SVG file illustrating the results of the current
   * search.  Obstacle cells are painted blue, and free cells that
   * have been visited are rainbow colored according to costToCome.
   * Predecessor relationships are drawn with purple lines. Each
   * visited node is also decorated with a circle: nodes in the queue
   * are filled with white, dead nodes are filled with purple.
   * The path back from the goal is drawn in black.
   */
  void makeDebugSVG(const std::string& filename,
                    const OccupancyGrid& grid,
                    const vec2u& goal,
                    int zoom=12) const;

  /* Generate a PNG file illustrating the results of the current
   * search.  The color scheme is as in makeDebugSVG(), except that no
   * circles or lines are drawn.  This is useful for larger maps.
   */
  void makeDebugPNG(const std::string& filename,
                    const OccupancyGrid& grid,
                    const vec2u& goal) const;

  /* Return the number of expansions (i.e. calls to enqueue on nodes
   * of type NODE_UNVISITED). 
   */
  size_t expansionCount();

private:

  typedef std::vector<size_t> IndexArray;
  
  vec2u _dims;

  GridNodeArray _nodes;
  IndexArray _queue;

  size_t _expCount;

};


#endif
