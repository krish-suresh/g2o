// g2o - General Graph Optimization
// Copyright (C) 2011 Kurt Konolige
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright
//   notice, this list of conditions and the following disclaimer in the
//   documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
// TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
// PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
// LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
// NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

#include "vertex_cam.h"

#ifdef G2O_HAVE_OPENGL
#include "g2o/stuff/opengl_primitives.h"
#include "g2o/stuff/opengl_wrapper.h"
#endif
namespace g2o {

// constructor
VertexCam::VertexCam() {}

bool VertexCam::read(std::istream& is) {
  // first the position and orientation (vector3 and quaternion)
  Vector3 t;
  internal::readVector(is, t);
  Quaternion r;
  internal::readVector(is, r.coeffs());
  r.normalize();  // recover nummeric precision

  // form the camera object
  SBACam cam(r, t);

  // now fx, fy, cx, cy, baseline
  number_t fx, fy, cx, cy, tx;

  // try to read one value
  is >> fx;
  if (is.good()) {
    is >> fy >> cx >> cy >> tx;
    cam.setKcam(fx, fy, cx, cy, tx);
  } else {
    is.clear();
    std::cerr << "cam not defined, using defaults" << std::endl;
    cam.setKcam(300, 300, 320, 320, cst(0.1));
  }

  setEstimate(cam);
  return true;
}

bool VertexCam::write(std::ostream& os) const {
  const SBACam& cam = estimate();

  // first the position and orientation (vector3 and quaternion)
  internal::writeVector(os, cam.translation());
  internal::writeVector(os, cam.rotation().coeffs());

  // now fx, fy, cx, cy, baseline
  os << cam.Kcam(0, 0) << " ";
  os << cam.Kcam(1, 1) << " ";
  os << cam.Kcam(0, 2) << " ";
  os << cam.Kcam(1, 2) << " ";
  os << cam.baseline << " ";
  return os.good();
}

#ifdef G2O_HAVE_OPENGL
VertexCamDrawAction::VertexCamDrawAction()
    : DrawAction(typeid(VertexCam).name()), _pointSize(nullptr) {}

bool VertexCamDrawAction::refreshPropertyPtrs(
    HyperGraphElementAction::Parameters* params_) {
  if (!DrawAction::refreshPropertyPtrs(params_)) return false;
  if (_previousParams) {
    _pointSize = _previousParams->makeProperty<FloatProperty>(
        _typeName + "::POINT_SIZE", 1.);
  } else {
    _pointSize = nullptr;
  }
  return true;
}

HyperGraphElementAction* VertexCamDrawAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params) {
  if (typeid(*element).name() != _typeName) return nullptr;
  initializeDrawActionsCache();
  refreshPropertyPtrs(params);
  if (!_previousParams) return this;

  if (_show && !_show->value()) return this;
  VertexCam* that = static_cast<VertexCam*>(element);
  glPushMatrix();
  glPushAttrib(GL_ENABLE_BIT | GL_POINT_BIT);
  glDisable(GL_LIGHTING);
  glColor3f(POSE_EDGE_COLOR);
  if (that->estimate().baseline == 1) {
    glTranslatef((float)that->estimate().translation()(0), (float)that->estimate().translation()(1),
                (float)that->estimate().translation()(2));
    // opengl::drawPoint(ps);
    opengl::drawSphere(0.05f);
    glTranslatef(-(float)that->estimate().translation()(0), -(float)that->estimate().translation()(1),
                -(float)that->estimate().translation()(2));
  }
  glColor3f(POSE_VERTEX_COLOR);
  double len = 0.25;
  Eigen::Vector3d origin(0,0,0);
  Eigen::Vector3d xaxis(len,0,0);
  Eigen::Vector3d yaxis(0,len,0);
  Eigen::Vector3d zaxis(0,0,len);
  auto pose = that->estimate();
  origin = pose*origin;
  xaxis = pose*xaxis;
  yaxis = pose*yaxis;
  zaxis = pose*zaxis;

  glBegin(GL_LINES);
  glColor3f(1,0,0);
  glVertex3f(origin.x(), origin.y(), origin.z());
  glVertex3f(xaxis.x(), xaxis.y(), xaxis.z());
  glEnd();

  glBegin(GL_LINES);
  glColor3f(0,1,0);
  glVertex3f(origin.x(), origin.y(), origin.z());
  glVertex3f(yaxis.x(), yaxis.y(), yaxis.z());
  glEnd();

  glBegin(GL_LINES);
  glColor3f(0,0,1);
  glVertex3f(origin.x(), origin.y(), origin.z());
  glVertex3f(zaxis.x(), zaxis.y(), zaxis.z());
  glEnd();

  glPopAttrib();
  drawCache(that->cacheContainer(), params);
  drawUserData(that->userData(), params);
  glPopMatrix();
  return this;
}
#endif

VertexCamWriteGnuplotAction::VertexCamWriteGnuplotAction()
    : WriteGnuplotAction(typeid(VertexCam).name()) {}

HyperGraphElementAction* VertexCamWriteGnuplotAction::operator()(
    HyperGraph::HyperGraphElement* element,
    HyperGraphElementAction::Parameters* params_) {
  if (typeid(*element).name() != _typeName) return nullptr;
  WriteGnuplotAction::Parameters* params =
      static_cast<WriteGnuplotAction::Parameters*>(params_);
  if (!params->os) {
    std::cerr << __PRETTY_FUNCTION__ << ": warning, no valid os specified"
              << std::endl;
    return nullptr;
  }

  VertexCam* v = static_cast<VertexCam*>(element);
  *(params->os) << v->estimate().translation().x() << " " << v->estimate().translation().y() << " "
                << v->estimate().translation().z() << " " << std::endl;
  return this;
}

}  // namespace g2o
