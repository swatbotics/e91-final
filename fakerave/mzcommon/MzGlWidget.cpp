/********************************************************/
/*    Author: Matt Zucker <mzucker1@swarthmore.edu>     */
/* Copyright (c) 2012 Matt Zucker. All rights reserved. */
/********************************************************/

#include "MzGlWidget.h"
#include "glstuff.h"

#ifdef MZ_HAVE_QT4
#include <QMouseEvent>
#endif

MzGlWidget::MzGlWidget ( QWidget * parent, 
                         const char * name, 
                         const QGLWidget * shareWidget, 
                         Qt::WFlags f ):
  QGLWidget(parent, name, shareWidget, f)
{
  _init();
}
  

MzGlWidget::MzGlWidget ( QGLContext * context, 
                         QWidget * parent, 
                         const char * name, 
                         const QGLWidget * shareWidget, 
                         Qt::WFlags f ):
  QGLWidget(context, parent, name, shareWidget, f)
{
  _init();
}

MzGlWidget::MzGlWidget ( const QGLFormat & format, 
                         QWidget * parent, 
                         const char * name, 
                         const QGLWidget * shareWidget, 
                         Qt::WFlags f ):
  QGLWidget(format, parent, name, shareWidget, f)
{ 
  _init();
}

const GlCamera& MzGlWidget::camera() const {
  return _camera;
}

GlCamera& MzGlWidget::camera() {
  return _camera;
}

void MzGlWidget::setupBasicLight(const vec4f& position) {

  const GLfloat lightdim[4] = { 0.3, 0.3, 0.3, 1.0 };
  const GLfloat lightbrt[4] = { 0.7, 0.7, 0.7, 1.0 };
  const GLfloat white[4] = { 1, 1, 1, 1 };

  glEnable(GL_COLOR_MATERIAL);
  glEnable(GL_LIGHT0);
  
  glLightfv(GL_LIGHT0, GL_POSITION, position.v);
  glLightfv(GL_LIGHT0, GL_AMBIENT,  lightdim);
  glLightfv(GL_LIGHT0, GL_DIFFUSE,  lightbrt);
  glLightfv(GL_LIGHT0, GL_SPECULAR, white);
  
  glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, white);
  glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 120.0f);
  
  glEnable(GL_LIGHTING);
  
}


void MzGlWidget::drawSphere(float r, int slices, int stacks) {
  gluSphere(_q, r, slices, stacks);
}

void MzGlWidget::drawSphere(const vec3f& p,
                            float r,
                            int slices,
                            int stacks) {
  glMatrixMode(GL_MODELVIEW);
  glPushMatrix();
  glTranslatef(p[0], p[1], p[2]);
  drawSphere(r, slices, stacks);
  glPopMatrix();
}

void MzGlWidget::drawCapsule(const vec3f& p0,
                             const vec3f& p1,
                             float r,
                             int slices,
                             int sstacks,
                             int cstacks) {
  glstuff::draw_capsule(_q, p0, p1, r, slices, sstacks, cstacks);
}

void MzGlWidget::initializeGL() {

  glClearColor(1, 1, 1, 1);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_CULL_FACE);

  if (!_q) { _q = gluNewQuadric(); }

}

void MzGlWidget::resizeGL(int width, int height) {

  glViewport(0, 0, width, height);
  _camera.setViewport(0, 0, width, height);

}

void MzGlWidget::paintGL() {

  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

  glMatrixMode(GL_PROJECTION);
  _camera.loadMatrix(GlCamera::MATRIX_PROJECTION);

  glMatrixMode(GL_MODELVIEW);
  _camera.loadMatrix(GlCamera::MATRIX_MODELVIEW);

}

void MzGlWidget::mousePressEvent(QMouseEvent* e) {
  const QPoint& p = e->pos();
  GlCamera::MouseMode m = _btn2cam(e->button());
  _camera.mousePress(p.x(), p.y(), m);
  updateGL();
  
}

void MzGlWidget::mouseReleaseEvent(QMouseEvent* e) {
  const QPoint& p = e->pos();
  GlCamera::MouseMode m = _btn2cam(e->button());
  _camera.mouseRelease(p.x(), p.y(), m);
  updateGL();

}

void MzGlWidget::mouseDoubleClickEvent(QMouseEvent* e) {
  _camera.recallHomePosition();
  updateGL();
}

void MzGlWidget::mouseMoveEvent(QMouseEvent* e) {
  const QPoint& p = e->pos();
#ifdef MZ_HAVE_QT4
  GlCamera::MouseMode m = _btn2cam(e->buttons() & Qt::MouseButtonMask);
#else
  GlCamera::MouseMode m = _btn2cam(e->state() & Qt::MouseButtonMask);
#endif
  _camera.mouseMove(p.x(), p.y(), m);
  updateGL();
}

void MzGlWidget::_init() {
  _q = 0;
}

GlCamera::MouseMode MzGlWidget::_btn2cam(int button) {
  if (button == Qt::LeftButton) {
    return GlCamera::MOUSE_ROTATE;
  } else if (button == Qt::MidButton) {
    return GlCamera::MOUSE_PAN_XY;
  } else if (button == Qt::RightButton) {
    return GlCamera::MOUSE_ZOOM;
  } else {
    return GlCamera::MOUSE_NONE;
  }
}
