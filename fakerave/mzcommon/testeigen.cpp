#include <Eigen/Core>
#include <Eigen/Geometry>
#include "quat.h"
#include <iostream>
#include <typeinfo>
#include "Transform3.h"
#include <cxxabi.h>

typedef Eigen::Matrix3f Matrix3f;
typedef Eigen::Vector3f Vector3f;
typedef Eigen::Quaternionf Quaternionf;
typedef Eigen::Isometry3f Isometry3f;

const char* demangle(const char* name) {
  char buf[1024];
  size_t size=1024;
  int status;
  char* res = abi::__cxa_demangle (name,
                                   buf,
                                   &size,
                                   &status);
  return res;
}

std::ostream& operator<<(std::ostream& ostr, const Quaternionf& q) {
  return ostr << q.vec().transpose() << q.w();
}

Isometry3f transform(const Quaternionf& q, 
                     const Vector3f& v) {

  return Isometry3f(q.matrix()).pretranslate(v);

}

template <class Derived>
void set00(Eigen::MatrixBase<Derived>& M) {
  std::cout << "type of M is " << demangle(typeid(M).name()) << "\n";
  M(0,0) = 1.0;
}

int main(int argc, char** argv) {

  /*
  std::cout << "sizeof(Vector3f) is " << sizeof(Vector3f) << "\n";
  std::cout << "sizeof(vec3f) is " << sizeof(vec3f) << "\n\n";

  std::cout << "sizeof(Quaternionf) is " << sizeof(Quaternionf) << "\n";
  std::cout << "sizeof(quatf) is " << sizeof(quatf) << "\n\n";

  std::cout << "sizeof(Matrix3f) is " << sizeof(Matrix3f) << "\n";
  std::cout << "sizeof(mat3f) is " << sizeof(mat3f) << "\n\n";

  std::cout << "sizeof(Isometry3f) is " << sizeof(Isometry3f) << "\n";
  std::cout << "sizeof(Transform3f) is " << sizeof(Transform3f) << "\n\n";

  quatf q = quatf::fromEuler(vec3f(0.1, 0.2, 0.3));
  vec3f v(.4, .5, .6);

  Quaternionf qq(&(q[0]));
  Vector3f vv(&(v[0]));

  std::cout << "q = " << q << "\n";
  std::cout << "qq = " << qq.vec().transpose() << ", " << qq.w() << "\n";
  std::cout << "euler angles: " << qq.matrix().eulerAngles(0,1,2).transpose() << "\n\n";

  Transform3f t(q, v);
  Isometry3f tt = transform(qq, vv);

  std::cout << "t.matrix() =\n" << t.matrix() << "\n";
  std::cout << "tt.matrix() =\n" << tt.matrix() << "\n";
  std::cout << "tt.rotation() = \n" << Quaternionf(tt.rotation()) << "\n\n";

  vec3f p(7,8,9);
  Vector3f pp(&(p[0]));

  std::cout << "t*p = " << t*p << "\n";
  std::cout << "tt*pp = " << (tt*pp).transpose() << "\n";
  
  */

  Eigen::MatrixXf m(4,4);
  m.setZero();
  
  std::cout << "m =\n" << m << "\n";
  
  set00(m);

  std::cout << "now m =\n" << m << "\n";

  Eigen::Block<Eigen::MatrixXf> b = m.block(1, 1, 3, 3);

  set00(b);

  std::cout << "now m =\n" << m << "\n";

  return 0;

}
