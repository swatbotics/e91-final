#include "TriMesh3.h"
#include <stdlib.h>

int main(int argc, char** argv) {

  if (argc != 2) { 
    std::cerr << "usage: " << argv[0] << " mesh.wrl\n";
    exit(1);
  }

  TriMesh3f mesh;

  try {
    mesh.parseWrl(argv[1]);
    mesh.saveObj("foo.obj");
  } catch (std::exception& e) {
    std::cerr << e.what() << "\n";
    exit(1);
  }

  return 0;

}
