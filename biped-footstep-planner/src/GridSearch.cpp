#include "GridSearch.h"
#include <float.h>
#include <assert.h>
#include <png.h>
#include <math.h>
#include <set>
#include <algorithm>

//////////////////////////////////////////////////////////////////////

GridNode::GridNode(): 
  type(NODE_INVALID),
  position(-1, -1),
  predecessor(-1, -1),
  costToCome(FLT_MAX),
  costToGo(0) {}


float GridNode::totalCost() const {
  return costToCome + costToGo;
}


//////////////////////////////////////////////////////////////////////

GridSearchData::GridSearchData() {
  reset();
}

size_t GridSearchData::expansionCount() {
  return _expCount;
}

void GridSearchData::reset() {
  _dims = vec2u(0,0);
  _nodes.clear();
  _queue.clear();
  _expCount = 0;
}

void GridSearchData::initialize(const OccupancyGrid& grid) {

  reset();

  _dims = grid.dims();

  _nodes.resize(_dims.prod(), GridNode());

  for (size_t i=0; i<_dims.prod(); ++i) {
    _nodes[i].type = NODE_UNVISITED;
    _nodes[i].position = vec2u::ind2sub(_dims, i);
    _nodes[i].predecessor = _nodes[i].position;
  }


  _queue.clear();

}

bool GridSearchData::queueEmpty() const {

  return _queue.empty();

}


class CompareIndices {
public:
  
  const GridNodeArray& nodes;

  CompareIndices(const GridNodeArray& n): nodes(n) {}

  bool operator()(size_t a, size_t b) {
    return nodes[a].totalCost() > nodes[b].totalCost();
  }
  
};

GridNode GridSearchData::dequeue() {

  if (queueEmpty()) {

    return GridNode();

  } else {

    size_t idx = _queue.front();
    std::pop_heap(_queue.begin(), _queue.end(), CompareIndices(_nodes));
    _queue.pop_back();

    _nodes[idx].type = NODE_DEAD;

    return _nodes[idx];

  }
  

}


GridNode GridSearchData::lookup(const vec2u& position) const {

  if (position.x() >= _dims.x() || position.y() >= _dims.y()) {
    std::cerr << "warning: position out of bounds in lookup\n";
    return GridNode();
  } 

  size_t idx = vec2u::sub2ind(_dims, position);
  
  return _nodes[idx];

}

void GridSearchData::enqueue(const vec2u& position,
                         const vec2u& predecessor,
                         float costToCome,
                         float costToGo) {

  if (costToCome < 0 || costToGo < 0) {
    std::cerr << "warning: enqueueing node with negative cost! "
              << "this will cause problems!\n";
  }

  if (position.x() >= _dims.x() || position.y() >= _dims.y()) {
    std::cerr << "warning: invalid index in enqueue: position out of bounds!\n";
    return;
  }

  size_t idx = vec2u::sub2ind(_dims, position);

  GridNode& n = _nodes[idx];

  bool wasEnqueued = (n.type == NODE_IN_QUEUE);

  n.type = NODE_IN_QUEUE;
  n.predecessor = predecessor;
  n.costToCome = costToCome;
  n.costToGo = costToGo;


  // No longer valid with inflated search
/*
  float newCost = costToCome + costToGo;
  
  if (newCost > n.totalCost()) {
    std::cerr << "warning: enqueuing node with higher cost! "
              << "this will cause problems!\n";
  }*/
  
  if (wasEnqueued) {
    
    std::make_heap(_queue.begin(), _queue.end(), CompareIndices(_nodes));

  } else {

    _queue.push_back(idx);
    std::push_heap(_queue.begin(), _queue.end(), CompareIndices(_nodes));
    ++_expCount;

  }


}


//////////////////////////////////////////////////////////////////////

static void getRGB(float costToCome, int& fr, int& fg, int& fb) {

  costToCome /= 4;

  const int rgbtbl[24][3] = {
    { 255,   0,   0 },
    { 255,  63,   0 },
    { 255, 127,   0 },
    { 255, 191,   0 },
    { 255, 255,   0 },
    { 191, 255,   0 },
    { 127, 255,   0 },
    {  63, 255,   0 },
    {   0, 255,   0 },
    {   0, 255,  63 },
    {   0, 255, 127 },
    {   0, 255, 191 },
    {   0, 255, 255 },
    {   0, 191, 255 },
    {   0, 127, 255 },
    {   0,  63, 255 },
    {   0,   0, 255 },
    {  63,   0, 255 },
    { 127,   0, 255 },
    { 191,   0, 255 },
    { 255,   0, 255 },
    { 255,   0, 191 },
    { 255,   0, 127 },
    { 255,   0,  63 },
  };

  // shade by cost-to-come
  int idx0 = floor(costToCome);
  float u = costToCome - idx0;
  idx0 = idx0 % 24;
  int idx1 = (idx0 + 1) % 24;

  fr = u*rgbtbl[idx1][0] + (1-u)*rgbtbl[idx0][0];
  fg = u*rgbtbl[idx1][1] + (1-u)*rgbtbl[idx0][1];
  fb = u*rgbtbl[idx1][2] + (1-u)*rgbtbl[idx0][2];

}

typedef std::set<vec2u> Vec2uSet;

static void markPath(const GridSearchData& helper, 
                     const vec2u& goal,
                     Vec2uSet& path) {

  path.clear();

  GridNode n = helper.lookup(goal);
  
  while (n.type == NODE_DEAD || n.type == NODE_IN_QUEUE) {
    
    path.insert(n.position);

    if (n.predecessor != n.position) {
      n = helper.lookup(n.predecessor);
    } else {
      break;
    }

  }


}

void GridSearchData::makeDebugPNG(const std::string& filename,
                                  const OccupancyGrid& grid,
                                  const vec2u& goal) const {


  if (_dims != grid.dims()) {
    std::cerr << "grid size does not agree with my size in GridSearchData::makeDebugPNG()\n";
    return;
  }

  Vec2uSet path;
  markPath(*this, goal, path);

  
  FILE* fp = fopen(filename.c_str(), "wb");
  if (!fp) { 
    std::cerr << "couldn't open " << filename << " for output!\n";
    return;
  }
  
  png_structp png_ptr = png_create_write_struct
    (PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);

  if (!png_ptr) {
    std::cerr << "error creating png write struct\n";
    return;
  }
  
  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    std::cerr << "error creating png info struct\n";
    png_destroy_write_struct(&png_ptr,
			     (png_infopp)NULL);
    fclose(fp);
    return;
  }  

  if (setjmp(png_jmpbuf(png_ptr))) {
    std::cerr << "error in png processing\n";
    png_destroy_write_struct(&png_ptr, &info_ptr);
    fclose(fp);
    return;
  }

  png_init_io(png_ptr, fp);

  png_set_IHDR(png_ptr, info_ptr, 
	       grid.nx(), grid.ny(),
	       8, 
	       PNG_COLOR_TYPE_RGB,
	       PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_DEFAULT,
	       PNG_FILTER_TYPE_DEFAULT);

  png_write_info(png_ptr, info_ptr);

  // TODO: write PNG
  std::vector<unsigned char> rgb(grid.nx()*3);
  
  // cell backgrounds
  for (size_t y=0; y<grid.ny(); ++y) {

    unsigned char* dst = &(rgb[0]);

    for (size_t x=0; x<grid.nx(); ++x) {

      vec2u pos(x,y);

      GridNode n = lookup(pos);

      int fr=255, fg=255, fb=255;
      
      unsigned char gxy = grid(x,y);

      if (path.count(pos)) {

        fr = fg = fb = 0;

      } else if (gxy != 255) { // obstacle

	// This check is no longer true because we can go through cells that are not totally white
      /*  if (n.type != NODE_UNVISITED) {
          std::cerr << "warning: visited an obstacle cell!\n";
        }*/

        // draw a rectangle of the obstacle color
        fb = 127 + gxy;
        fr = fg = fb*0.7;
                
      } else if (n.type == NODE_DEAD || n.type == NODE_IN_QUEUE) {

        getRGB(n.costToCome, fr, fg, fb);

      }

      dst[0] = fr;
      dst[1] = fg;
      dst[2] = fb;
      dst += 3;

    }

    png_write_row(png_ptr, (png_bytep)&(rgb[0]));

  }
  

  png_write_end(png_ptr, info_ptr);

  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(fp);

}



void GridSearchData::makeDebugSVG(const std::string& filename,
                                  const OccupancyGrid& grid,
                                  const vec2u& goal,
                                  int scl) const {


  if (_dims != grid.dims()) {
    std::cerr << "grid size does not agree with my size in GridSearchData::makeDebugPNG()\n";
    return;
  }

  Vec2uSet path;
  markPath(*this, goal, path);

  FILE* svg = fopen(filename.c_str(), "w");
  if (!svg) {
    std::cerr << "error opening svg for output!\n";
    return;
  }

  int width = grid.nx() * scl;
  int height = grid.ny() * scl;

  fprintf(svg, "<?xml version=\"1.0\" encoding=\"utf-8\"?>\n");
  fprintf(svg, "<!DOCTYPE svg PUBLIC \"-//W3C//DTD SVG 1.1//EN\"\n");
  fprintf(svg, "  \"http://www.w3.org/Graphics/SVG/1.1/DTD/svg11.dtd\">\n");
  fprintf(svg, "<svg version=\"1.1\" xmlns=\"http://www.w3.org/2000/svg\"\n");
  fprintf(svg, "     xmlns:xlink=\"http://www.w3.org/1999/xlink\"\n");
  fprintf(svg, "     width=\"%upx\" height=\"%upx\" viewBox=\"0 0 %u %u\" >\n", 
          width, height, width, height);
  
  // cell backgrounds
  for (size_t y=0; y<grid.ny(); ++y) {
    for (size_t x=0; x<grid.nx(); ++x) {

      vec2u pos(x,y);

      GridNode n = lookup(pos);

      bool fill = false;
      int fr=0, fg=0, fb=0;
      
      unsigned char gxy = grid(x,y);

      if (gxy <= 127) { // obstacle

	// No longer valid check with non-white cells that can be accessed
      /*  if (n.type != NODE_UNVISITED) {
          std::cerr << "warning: visited an obstacle cell!\n";
        } */

        // draw a rectangle of the obstacle color
        fb = 127 + gxy;
        fr = fg = fb*0.7;
        fill = true;
                
      } else if (n.type == NODE_DEAD || n.type == NODE_IN_QUEUE) {

        fill = true;
        getRGB(n.costToCome, fr, fg, fb);

      }

      if (fill) {
        fprintf(svg, "  <rect x=\"%d\" y=\"%d\" width=\"%d\" height=\"%d\" "
                "fill=\"#%02x%02x%02x\" />\n", 
                (int)x*scl, (int)y*scl, scl, scl, fr, fg, fb);
      }


    }
  }

  // links to parents

  fprintf(svg, "  <g stroke=\"#7f007f\" stroke-width=\"1.5\">\n");

  for (size_t y=0; y<grid.ny(); ++y) {
    for (size_t x=0; x<grid.nx(); ++x) {

      GridNode n = lookup(vec2u(x,y));
      if (n.position != n.predecessor) {

        // draw a line
        float x1 = (n.predecessor.x() + 0.5) * scl;
        float y1 = (n.predecessor.y() + 0.5) * scl;
        float x2 = (n.position.x() + 0.5) * scl;
        float y2 = (n.position.y() + 0.5) * scl;

        fprintf(svg, "    <line x1=\"%f\" y1=\"%f\" x2=\"%f\" y2=\"%f\" %s/>\n",
                x1, y1, x2, y2,
                (path.count(n.predecessor) && path.count(n.position)) ?
                "stroke=\"#000000\" stroke-width=\"2.5\" " : "");


      }

    }
  }

  // circles
  for (size_t y=0; y<grid.ny(); ++y) {
    for (size_t x=0; x<grid.nx(); ++x) {

      GridNode n = lookup(vec2u(x,y));
      
      if (n.type == NODE_DEAD || n.type == NODE_IN_QUEUE) {
        float cx = (x + 0.5)*scl;
        float cy = (y + 0.5)*scl;
        float r = 0.2*scl;
        fprintf(svg, "    <circle cx=\"%f\" cy=\"%f\" r=\"%f\" fill=\"%s\" %s/>\n",
                cx, cy, r, n.type == NODE_IN_QUEUE ? "#ffffff" : "#7f007f",
                path.count(n.position) ? "stroke=\"#000000\" stroke-width=\"2.5\" " : "");
      }
      

    }
  }
  
  fprintf(svg, "  </g>\n");
  
  fprintf(svg, "</svg>\n");

  fclose(svg);


}
