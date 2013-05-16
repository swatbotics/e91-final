#include "lab3.h"
#include <stdlib.h>
#include <math.h>

static float heuristic(const vec2u& a, const vec2u& b) {
  float dx = float(a.x()) - float(b.x());
  float dy = float(a.y()) - float(b.y());
  return sqrt(dx*dx + dy*dy);
}

bool search(const OccupancyGrid& grid,
            const vec2u& initial, 
            const vec2u& goal,
            GridSearchData& helper, 
            bool useheuristic) {

  helper.initialize(grid);
  helper.enqueue(initial, initial, 0, heuristic(initial, goal));
  
  while (!helper.queueEmpty()) {

    GridNode n = helper.dequeue();
    
    if (n.position == goal) {
      return true;
    }
    
    const int dx[8] = { -1,  0,  1, -1, 1, -1, 0, 1 };
    const int dy[8] = { -1, -1, -1,  0, 0,  1, 1, 1 };

    const vec2u& np = n.position;

    for (int dir=0; dir<8; ++dir) {

      vec2u newpos(np.x() + dx[dir], np.y() + dy[dir]);
      float cost = sqrt(abs(dx[dir]) + abs(dy[dir]));
      
      if (newpos.x() < grid.nx() && newpos.y() < grid.ny() && 
          grid(newpos) > 127) {
        
        GridNode nn = helper.lookup(newpos);
        float costToCome = n.costToCome + cost;
        float costToGo = useheuristic ? heuristic(newpos, goal) : 0;
        
        if (nn.type == NODE_UNVISITED || costToCome < nn.costToCome) {

          helper.enqueue(newpos, np, costToCome, costToGo);

        }

      }
    }
    

  }

  return false;

}

void expandObstacles(const OccupancyGrid& src,
                     float radius,
                     OccupancyGrid& dst) {

  int n = ceil(radius);
  float r2 = radius*radius;

  dst.resize(src.dims());

  for (size_t y=0; y<src.ny(); ++y) {
    for (size_t x=0; x<src.nx(); ++x) {

      unsigned char newval = 255;

      for (int v=-n; v<=n; ++v) {
        for (int u=-n; u<=n; ++u) {

          if (u*u + v*v > r2) {
            continue;
          }

          vec2u s(x+u, y+v);
          if (s.x() < src.nx() && 
              s.y() < src.ny() && 
              src(s) < 255) {
            newval = 64;
          }

        }
      }
      
      dst(x,y) = std::min(src(x,y), newval);

    }
  }
  
}
