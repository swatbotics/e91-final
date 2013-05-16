#ifndef _VEC2U_H_
#define _VEC2U_H_

#include <iostream>

/* Struct to hold an (x,y) index into an image or grid. */
template <class dtype>
struct vec2u {

  /* Array of indices */
  dtype v[2];

  /* Default constructor doesn't initialize. */
  vec2u() {}

  /* Construct to single value. */
  explicit vec2u(dtype k) {
    v[0] = v[1] = k;
  }

  /* Construct with x and y coordinates. */
  vec2u(dtype x, dtype y) {
    v[0] = x; 
    v[1] = y;
  }

  /* Access the x coordinate. */
  const dtype& x() const { return v[0]; }

  /* Access the x coordinate. */
  dtype& x() { return v[0]; }

  /* Access the y coordinate. */
  const dtype& y() const { return v[1]; }

  /* Access the y coordinate. */
  dtype& y() { return v[1]; }

  /* Return the product of the coordinates. */
  dtype prod() const {
    return v[0]*v[1];
  }

  /* Return the minimum coordinate. */
  dtype min() const {
    return std::min( v[0], v[1] );
  }

  /* Return the maximum coordinate. */
  dtype max() const {
    return std::max( v[0], v[1] );
  }

  /* Return the minimum coordinate along each dimension. */
  static vec2u min(const vec2u& v1, const vec2u& v2) {
    return vec2u( std::min(v1[0], v2[0]),
                  std::min(v1[1], v2[1]) );
  }

  /* Return the maximum coordinate along each dimension. */
  static vec2u max(const vec2u& v1, const vec2u& v2) {
    return vec2u( std::max(v1[0], v2[0]),
                  std::max(v1[1], v2[1]) );
  }

  /* Elementwise access. */
  const dtype& operator[](dtype d) const { return v[d]; }

  /* Elementwise access. */
  dtype& operator[](dtype d) { return v[d]; }

  /* Addition-assignment */
  vec2u& operator+=(const vec2u& v2) {
    v[0] += v2[0];
    v[1] += v2[1];
    return *this;
  }

  /* Subtraction-assignment */
  vec2u& operator-=(const vec2u& v2) {
    v[0] -= v2[0];
    v[1] -= v2[1];
    return *this;
  }

  /* Multiplication-assignment. */
  vec2u& operator*=(dtype s) {
    v[0] *= s;
    v[1] *= s;
    return *this;
  }

  /* Convert dimensions and subscript to linear index. */
  static dtype sub2ind(const vec2u& d, const vec2u& s) {
    return s[1]*d[0] + s[0];
  }

  /* Convert dimensions and subscript to linear index. */
  static dtype sub2ind(const vec2u& d, dtype x, dtype y) {
    return y*d[0] + x;
  }

  /* Convert dimensions and linear index to subscript. */
  static vec2u ind2sub(const vec2u& d, dtype idx) { 
    return vec2u(idx%d[0], idx/d[0]);
  }

};

/* Addition operator. */
template <class dtype>
inline vec2u<dtype> operator+(const vec2u<dtype>& u, const vec2u<dtype>& v) {
  return vec2u<dtype>(u[0] + v[0], u[1] + v[1]);
}

/* Subtraction operator. */
template <class dtype>
inline vec2u<dtype> operator-(const vec2u<dtype>& u, const vec2u<dtype>& v) {
  return vec2u<dtype>(u[0] - v[0], u[1] - v[1]);
}

/* Multiplication operator. */
template <class dtype>
inline vec2u<dtype> operator*(const vec2u<dtype>& v, float s) {
  return vec2u<dtype>(v[0]*s, v[1]*s);
}

/* Multiplication operator. */
template <class dtype>
inline vec2u<dtype> operator*(float s, const vec2u<dtype>& v) {
  return vec2u<dtype>(v[0]*s, v[1]*s);
}

/* Comparison for equality. */
template <class dtype>
inline bool operator==(const vec2u<dtype>& a, const vec2u<dtype>& b) {
  return (a[0] == b[0] && a[1] == b[1]);
}

/* Comparison for inequality. */
template <class dtype>
inline bool operator!=(const vec2u<dtype>& a, const vec2u<dtype>& b) {
  return (a[0] != b[0] || a[1] != b[1]);
}

/* Comparison for lexical sorting. */
template <class dtype>
inline bool operator<(const vec2u<dtype>& a, const vec2u<dtype>& b) {
  for (int i=0; i<2; ++i) {
    if (a[i] < b[i]) { return true; }
    else if (a[i] > b[i]) { return false; }
  }
  return false;
}

/* Stream output. */
template <class dtype>
inline std::ostream& operator<<(std::ostream& ostr, const vec2u<dtype>& v) {
  return ostr << "[" << v[0] << ", " << v[1] << "]";
}

#endif
