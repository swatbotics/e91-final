hubomz
======

Hubo planning, control, and visualziation software

Please contact Matt Zucker <mzucker1@swarthmore.edu> with questions,
comments, or rants.

Some IK/FK code from Hubo Motion library -
https://github.com/hubo/hubo-motion-rt

Building
========

To build the software, you will need these libraries:
 
  - GLUT
  - eigen3
  - expat
  - qt4 (optional, but required for calibration GUI)

You must install cmake to build the software as well.  To build:

    cd /path/to/hubomz
    mkdir build
    cd build
    cmake .. -DCMAKE_BUILD_TYPE=Release
    make

To run the software, you will need an OpenRAVE model of the Hubo plus.
Get the openHubo package from https://github.com/daslrobotics/openHubo
and patch it using the huboplus.patch in the root directory of this
project.

    cd /path/to/openHubo
    patch -p0 < /path/to/hubomz/huboplus.patch

Next, symlink the huboplus directory from openHubo into the root
directory of this repository.

    cd /path/to/hubomz
    ln -s /path/to/openHubo/robots/huboplus
   
Now, you can run the software, for instance:

    cd /path/to/hubomz/build
    ./zmpdemo -g ../myhubo.kinbody.xml
    
On the mac, instead run

    ./zmpdemo.app/Contents/MacOS/zmpdemo -g ../myhubo.kinbody.xml


Tests
=====

A few tests can automatically be run. After you've successfully run `make`, do:

    make
    ctest
