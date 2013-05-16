#ifndef _OCCUPANCYGRID_H_
#define _OCCUPANCYGRID_H_

#include "vec2u.h"
#include <vector>
#include <png.h>
#include <cstdio>

/* Class to represent an occupancy grid.  Cells holding values of less
 * than 128 should be considered obstacles.
 */
template <class dtype>
class OccupancyGrid {
public:

  /* Construct an empty occupancy grid. */
  OccupancyGrid();

  /* Reset to an empty grid. */
  void clear();

  /* Return the number of columns in the grid. */
  size_t nx() const;

  /* Return the number of rows in the grid. */
  size_t ny() const;

  /* Return the number of cells in the grid. */
  size_t size() const;

  /* Return the (x,y) dimensions of the grid. */
  const vec2u<size_t>& dims() const;

  /* Return true if the grid is empty. */
  bool empty() const;

  /* Resize the grid to the given directions. */
  void resize(const vec2u<size_t>& dims);

  /* Resize the grid to the given directions. */
  void resize(size_t nx, size_t ny);

  /* Load occupancy grid from an 8-bit per pixel PNG file. */
  void load(const std::string& pngfilename);

  /* Save occupancy grid to a 8-bit grayscale PNG file. */
  void save(const std::string& pngfilename) const;

  /* Convert the (x,y) subscript to the corresponding linear index. */
  size_t sub2ind(const vec2u<size_t>& pos) const;

  /* Convert the (x,y) subscript to the corresponding linear index. */
  size_t sub2ind(size_t x, size_t y) const;

  /* Convert a linear index to an (x,y) subscript. */
  vec2u<size_t> ind2sub(size_t idx) const;

  /* Access a cell of the grid via linear index. */
  dtype& operator[](size_t idx);

  /* Access a cell of the grid via linear index. */
  dtype operator[](size_t idx) const;

  /* Access a cell of the grid via (x,y) subscript. */
  dtype& operator()(const vec2u<size_t>& pos);

  /* Access a cell of the grid via (x,y) subscript. */
  dtype operator()(const vec2u<size_t>& pos) const;

  /* Access a cell of the grid via (x,y) indices. */
  dtype& operator()(size_t x, size_t y);

  /* Access a cell of the grid via (x,y) indices. */
  dtype operator()(size_t x, size_t y) const;

private:

  vec2u<size_t> _dims;
  std::vector<dtype> _data;

};


// Function instanciation

template <class dtype>
OccupancyGrid<dtype>::OccupancyGrid() { 
  clear();
}

template <class dtype>
void OccupancyGrid<dtype>::clear() {
  _data.clear();
  _dims = vec2u<size_t>(0,0);
}

template <class dtype>
size_t OccupancyGrid<dtype>::nx() const { return _dims.x(); }

template <class dtype>
size_t OccupancyGrid<dtype>::ny() const { return _dims.y(); }

template <class dtype>
size_t OccupancyGrid<dtype>::size() const { return _dims.prod(); }

template <class dtype>
const vec2u<size_t>& OccupancyGrid<dtype>::dims() const { return _dims; }

template <class dtype>
bool OccupancyGrid<dtype>::empty() const { return size() == 0; }

template <class dtype>
void OccupancyGrid<dtype>::resize(const vec2u<size_t>& dims) { 
  _dims = dims;
  _data.resize(size());
}

template <class dtype>
void OccupancyGrid<dtype>::resize(size_t nx, size_t ny) {
  resize(vec2u<size_t>(nx, ny));
}

template <class dtype>
size_t OccupancyGrid<dtype>::sub2ind(const vec2u<size_t>& pos) const {
  return vec2u<size_t>::sub2ind(_dims, pos);
}

template <class dtype>
size_t OccupancyGrid<dtype>::sub2ind(size_t x, size_t y) const {
  return vec2u<size_t>::sub2ind(_dims, vec2u<size_t>(x,y));
}

template <class dtype>
vec2u<size_t> OccupancyGrid<dtype>::ind2sub(size_t idx) const {
  return vec2u<size_t>::ind2sub(_dims, idx);
}

template <class dtype>
dtype& OccupancyGrid<dtype>::operator[](size_t idx) {
  return _data[idx];
}

template <class dtype>
dtype OccupancyGrid<dtype>::operator[](size_t idx) const {
  return _data[idx];
}

template <class dtype>
dtype& OccupancyGrid<dtype>::operator()(const vec2u<size_t>& pos) {
  return _data[sub2ind(pos)];
}

template <class dtype>
dtype OccupancyGrid<dtype>::operator()(const vec2u<size_t>& pos) const {
  return _data[sub2ind(pos)];
}


template <class dtype>
dtype& OccupancyGrid<dtype>::operator()(size_t x, size_t y) {
  return _data[sub2ind(x,y)];
}

template <class dtype>
dtype OccupancyGrid<dtype>::operator()(size_t x, size_t y) const {
  return _data[sub2ind(x,y)];
}

template <class dtype>
void OccupancyGrid<dtype>::load(const std::string& pngfilename) {

  FILE* fp = fopen(pngfilename.c_str(), "rb");
  if (!fp) {
    std::cerr << "couldn't open " << pngfilename << " for input!\n";
    return;
  }

  png_byte header[8];
  fread(header, 1, 8, fp);

  if (png_sig_cmp(header, 0, 8) != 0) {
    std::cerr << "not a PNG file!\n";
    fclose(fp);
    return;
  }

  png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, 
                                               NULL, NULL, NULL);

  if (!png_ptr) {
    std::cerr << "error creating png_ptr!\n";
    fclose(fp);
    return;
  }

  png_infop info_ptr = png_create_info_struct(png_ptr);
  if (!info_ptr) {
    std::cerr << "error creating info_ptr!\n";

    png_destroy_read_struct(&png_ptr,
                            (png_infopp)NULL, (png_infopp)NULL);
    fclose(fp);
    return;
  }

  png_infop end_info = png_create_info_struct(png_ptr);
  if (!end_info) {
    std::cerr << "error creating end_info!\n";
    png_destroy_read_struct(&png_ptr, &info_ptr, (png_infopp)NULL);
    fclose(fp);
    return;
  }

  if (setjmp(png_jmpbuf(png_ptr))) {
    std::cerr << "error reading PNG!\n";
    png_destroy_read_struct(&png_ptr, &info_ptr,
                            &end_info);
    fclose(fp);
    return;
  }

  png_init_io(png_ptr, fp);

  png_set_sig_bytes(png_ptr, 8);
  
  png_read_info(png_ptr, info_ptr);

  png_uint_32 width, height;
  int bit_depth, color_type, interlace_type, compression_type, filter_method;

  png_get_IHDR(png_ptr, info_ptr, &width, &height,
               &bit_depth, &color_type, &interlace_type,
               &compression_type, &filter_method);

  if (bit_depth != 8 ||
      (color_type != PNG_COLOR_TYPE_GRAY &&
       color_type != PNG_COLOR_TYPE_PALETTE)) {
    std::cerr << "not an 8-bit uninterlaced image!\n";
    png_destroy_read_struct(&png_ptr, &info_ptr,
                            &end_info);
    fclose(fp);
    return;
  }

  resize(width, height);

  int number_passes = png_set_interlace_handling(png_ptr);
  
  for (int pass = 0; pass<number_passes; ++pass) {
    dtype* dst = &(_data[0]);
    for (size_t y=0; y<height; ++y) {
      png_bytep row_pointers = (png_bytep)dst;
      png_read_rows(png_ptr, &row_pointers, NULL, 1);
      dst += width;
    }
  }
  
  png_destroy_read_struct(&png_ptr, &info_ptr,
                          &end_info);
  fclose(fp);
  std::cout<<"map loaded"<<std::endl;
  return;

  

}


template <class dtype>
void OccupancyGrid<dtype>::save(const std::string& pngfilename) const {

  FILE* fp = fopen(pngfilename.c_str(), "wb");
  if (!fp) { 
    std::cerr << "couldn't open " << pngfilename << " for output!\n";
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
	       nx(), ny(),
	       8, 
	       PNG_COLOR_TYPE_GRAY,
	       PNG_INTERLACE_NONE,
	       PNG_COMPRESSION_TYPE_DEFAULT,
	       PNG_FILTER_TYPE_DEFAULT);

  png_write_info(png_ptr, info_ptr);

  const dtype* rowptr = &(_data[0]);

  for (dtype y=0; y<ny(); ++y) {
    png_write_row(png_ptr, (png_byte*)rowptr);
    rowptr += nx();
  }
  
  png_write_end(png_ptr, info_ptr);

  png_destroy_write_struct(&png_ptr, &info_ptr);

  fclose(fp);

}

#endif
