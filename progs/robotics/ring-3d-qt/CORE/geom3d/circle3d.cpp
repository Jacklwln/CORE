/*****************************************************************
 * File: circle3d.cpp
 * Synopsis:
 *      Basic 3-dimensional geometry
 * Author: Ching-Hsiang Hsu (chhsu@cs.nyu.edu), 2017.
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
 * $Id: triangle3d.cpp,v 1.2 2010/05/19 04:48:20 exact Exp $
 *****************************************************************/

#include <CORE/geom3d/circle3d.h>
#include <CORE/geom3d/triangle3d.h>

extern FILE* g_fptr;
extern bool verbose;

/************************************************************
 *   constructors
 ************************************************************/
 
 
/************************************************************
 *   predicates
 ************************************************************/

bool Circle3d::do_intersect(Point3d pp, double rB){
  Point3d p_ = toPlane().projection(pp);
  double h = p_.distance(pp);
  double d = abs(p_.distance(p)-r);
  if(h*h+d*d <= rB*rB)
    return true;
  return false;
}

bool Circle3d::do_intersect(Segment3d seg, double rB){

  Point3d onCircle(*(seg.separation_circle(*this)));
  return do_intersect(seg.nearPt(onCircle), rB);

//  Point3d p1 = seg.startPt();
//  Point3d p2 = seg.stopPt();
//  Point3d p1_ = toPlane().projection(p1);
//  Point3d p2_ = toPlane().projection(p2);
//  Vector p2p1_(p2_-p1_); // p2'-p1' vector
//  p2p1_.normalize();
//  double t = Vector(p-p1_)*p2p1_/(p2p1_*p2p1_);
//  if(t < 0 || t > 1) return false;

//  Vector p2p1(p2-p1); // p2-p1 vector
//  p2p1.normalize();
//  Point3d pp = seg.startPt()+t*p2p1;
//  return do_intersect(pp, rB);
}

bool Circle3d::do_intersect(Triangle3d T, double rB){
  Point3d onCircle(*(T.separation_circle(*this)));
  Point3d onTriangle(T.nearPt(onCircle));

  return onTriangle.distance(onCircle)-rB <= 1e-8;
}

/************************************************************
 *  Intersection
 ************************************************************/

void Circle3d::intersection(Line3d l, Point3d pro, Point3d **p1, Point3d **p2){
  Vector v(l.direction());
  double a = v*v;
  double b = 2*(v*(pro-p));
  double c = Vector(pro-p)*Vector(pro-p)-r*r;

  double bb4ac = b*b-4*a*c;
  if(bb4ac >= 0){
    double t1 = (-b+sqrt(bb4ac))/(2*a);
    double t2 = (-b-sqrt(bb4ac))/(2*a);

    (*p1) = new Point3d(pro+t1*v);
    (*p2) = new Point3d(pro+t2*v);
  }
  else{
    (*p1) = (*p2) = NULL;
  }
}

void Circle3d::intersection(Line3d l, Point3d **p1, Point3d **p2){
  Vector v(l.direction());
  if(v*v > 0){
    double c = r*r/(v*v);

    double t1 = sqrt(c);
    double t2 = -sqrt(c);

    (*p1) = new Point3d(p+t1*v);
    (*p2) = new Point3d(p+t2*v);
  }
  else{
    (*p1) = (*p2) = NULL;
  }
}


/***************************************************************
 *   I/O 
 ***************************************************************/

std::ostream &operator<<(std::ostream &o, const Circle3d& C) {
  return o << "Circle3d:\n" << "\t" << C.p << std::endl 
                            << "\t" << C.n;
}

std::istream& operator>>(std::istream& in, Circle3d& C) {
  Point3d pp;
  double rr;
  Vector nn;
  std::cout << "\nInput point(-,-,-):";
  in >> pp;
  std::cout << "\nInput radius:";
  in >> rr;
  std::cout << "\nInput normal vector(-,-,-):";
  in >> nn;
  C = Circle3d(pp, rr, nn);
  return in;
}

