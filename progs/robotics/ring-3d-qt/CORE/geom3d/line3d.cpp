/*****************************************************************
 * File: line3d.cc
 * Synopsis:
 *      Basic 3-dimensional geometry
 * Author: Shubin Zhao (shubinz@cs.nyu.edu), 2001.
 *
 *****************************************************************
 * CORE Library Version 1.4 (July 2001)
 *       Chee Yap <yap@cs.nyu.edu>
 *       Chen Li <chenli@cs.nyu.edu>
 *       Zilin Du <zilin@cs.nyu.edu>
 *
 * Copyright (c) 1995, 1996, 1998, 1999, 2000, 2001 Exact Computation Project
 *
 * WWW URL: http://cs.nyu.edu/exact/
 * Email: exact@cs.nyu.edu
 *
 * $Id: line3d.cpp,v 1.1 2006/04/03 18:55:31 exact Exp $
 *****************************************************************/

#include <CORE/geom3d/line3d.h>

/************************************************************
 * *   constructors
************************************************************/

Line3d::Line3d(const Point3d &p, const Vector &v) : p0(p), p1(p + v), V(v) { }

Line3d::Line3d(const Point3d &_p0, const Point3d &_p1) : p0(_p0), p1(_p1), V(_p1 - _p0) { }

Line3d::Line3d(const Line3d &l) : p0(l.p0), p1(l.p1), V(l.V) { }

Line3d::Line3d() : p0(0.0, 0.0, 0.0), p1(0.0, 0.0, 0.0), V(0) {}
 // line passes through the origin with direction 0.

/************************************************************
 * *   Member functions 
************************************************************/

double Line3d::distance(const Point3d& q) const {
  Vector Vpq = q - p0;
  Vector u = V.cross( Vpq );
  return u.norm()/V.norm();
}

Point3d Line3d::projection(const Point3d& q) const {
  Vector Vpq = q - p0;
  double lambda = dotProduct(Vpq, V)/dotProduct(V, V);
  return p0 + lambda * V;
}

bool Line3d::contains(const Point3d& p) const {
  return V.cross( p - p0 ).isZero();
}

bool Line3d::isCoincident(const Line3d& l) const {
  return contains( l.startPt() ) && contains( l.stopPt() );
}

bool Line3d::isSkew(const Line3d& l2) const {
  //assert(intersects(l2) >= 0);
  double d = dotProduct(p0 - l2.startPt(), V.cross(l2.direction()));
  return (d != 0);
}

bool Line3d::isParallel(const Line3d& l2) const {
  return V.cross(l2.direction()).isZero();
}

int Line3d::intersects(const Line3d &l) const {
  // return the dimension of intersection
  // return -1 if not intersect
  if( !coplanar(p0, p1, l.startPt(), l.stopPt()) )
    return -1;
  else if( isCoincident( l ) ) 
    return 1;
  else if( isParallel( l ) ) 
    return -1;
  else 	// intersection is point
    return 0;    
}

GeomObj* Line3d::intersection(const Line3d &l) const {
  if( intersects( l ) == -1 ) 
    return NULL;

  if( isCoincident( l ) )
    return new Line3d(*this);
    
  Vector u = l.direction().cross(V);
  Vector w = l.direction().cross( l.startPt() - p0 );
  double lambda = dotProduct(w,u) / dotProduct(u,u);
  Vector t = lambda * V;
  return new Point3d(p0 + t);
}

//// reference: http://geomalgorithms.com/a06-_intersect-2.html
//bool Line3d::intersects( const Triangle3d& T ){
//  Vector    u, v, n;              // triangle vectors
//  Vector    w0, w;           // ray vectors
//  float     r, a, b;              // params to calc ray-plane intersect

//  // get triangle edge vectors and plane normal
//  u = Vector(T.V2() - T.V1());
//  v = Vector(T.V3() - T.V1());
//  n = u.cross(v);                 // cross product
//  if (n == (Vector)0)             // triangle is degenerate
//      return false;                  // do not deal with this case

//  w0 = Vector(p0 - T.V1());
//  a = -(n*w0);
//  b = n*direction();
//  if (fabs(b) < 1e-8) {     // ray is  parallel to triangle plane
//    if (a == 0) {                // ray lies in triangle plane
//      Segment3d seg[3];
//      seg[0] = Segment3d(T.V1(), T.V2());
//      seg[1] = Segment3d(T.V2(), T.V3());
//      seg[2] = Segment3d(T.V3(), T.V1());
//      for(int i=0;i<3;++i){
//        GeomObj* intersection = seg[i].intersection(*this);
//        if(intersection != NULL){
//          if(intersection->dim() == 0){
//            return true;
//          }
//          else if(intersection->dim() == 1){
//            return true;
//          }
//        }
//      }
//    }
//    else return false;              // ray disjoint from plane
//  }

//  // get intersect point of ray with triangle plane
//  r = a / b;
//  if (r < 0.0)                    // ray goes away from triangle
//      return false;                   // => no intersect
//  // for a segment, also test if (r > 1.0) => no intersect

//  Point3d I = p0 + r * direction();            // intersect point of ray and plane

//  // is I inside T?
//  float    uu, uv, vv, wu, wv, D;
//  uu = u*u;
//  uv = u*v;
//  vv = v*v;
//  w  = I - T.V0();
//  wu = w*u;
//  wv = w*v;
//  D = uv * uv - uu * vv;

//  // get and test parametric coords
//  float s, t;
//  s = (uv * wv - vv * wu) / D;
//  if (s < 0.0 || s > 1.0)         // I is outside T
//      return false;
//  t = (uv * wu - uu * wv) / D;
//  if (t < 0.0 || (s + t) > 1.0)  // I is outside T
//      return false;

//  return true;                       // I is in T
//}

std::ostream &operator<<(std::ostream &o, const Line3d &l) {
  return o << "Line3d[" << l.p0 << "," << l.p1 << ";" << l.V << "]";
}

std::istream& operator>>(std::istream& in, Line3d& l) {
  Point3d pStart, pEnd;
  std::cout << "\nInput start point(-,-,-):";
  in >> pStart;
  std::cout << "Input end point(-,-,-):";
  in >> pEnd;
  l = Line3d(pStart, pEnd); 
  return in;
}
