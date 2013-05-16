/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#ifndef _MZGLWIDGET_H_
#define _MZGLWIDGET_H_

#ifdef MZ_HAVE_QT4
#include <QGLWidget>
#else
#include <qgl.h>
#endif

#include "GlCamera.h"

#ifdef __APPLE__
#include <OpenGL/GLU.h>
#else
#include <GL/glu.h>
#endif

class MzGlWidget: public QGLWidget {
public:

  MzGlWidget ( QWidget * parent = 0, 
               const char * name = 0, 
               const QGLWidget * shareWidget = 0, 
               Qt::WFlags f = 0 );
  
  MzGlWidget ( QGLContext * context, 
               QWidget * parent, 
               const char * name = 0, 
               const QGLWidget * shareWidget = 0, 
               Qt::WFlags f = 0 );

  MzGlWidget ( const QGLFormat & format, 
               QWidget * parent = 0, 
               const char * name = 0, 
               const QGLWidget * shareWidget = 0, 
               Qt::WFlags f = 0 );
  
  const GlCamera& camera() const;
  GlCamera& camera();

  void setupBasicLight(const vec4f& position=vec4f(-1,1,1,0));

  void drawSphere(float r, 
                  int slices=32, 
                  int stacks=24);

  void drawSphere(const vec3f& p, 
                  float r, 
                  int slices=32, 
                  int stacks=24);

  void drawCapsule(const vec3f& p0,
                   const vec3f& p1,
                   float r,
                   int slices=32,
                   int sstacks=24,
                   int cstacks=4);

protected:

  virtual void initializeGL();
  virtual void resizeGL(int width, int height);
  virtual void paintGL();

  // these call the corresponding camera events by default
  virtual void mousePressEvent(QMouseEvent* e);
  virtual void mouseReleaseEvent(QMouseEvent* e);
  virtual void mouseMoveEvent(QMouseEvent* e);
  virtual void mouseDoubleClickEvent(QMouseEvent* e);

  // TODO: mouse wheel event!

  GLUquadric* _q;

private:

  static GlCamera::MouseMode _btn2cam(int button);
  void _init();

  GlCamera _camera;

};

#endif
