#include "subdivision/ConfBox3d.h"
#include "strategy/PriorityQueue.h"

extern FILE *g_fptr;
extern bool verbose;

double ConfBox3d::r0;
int ConfBox3d::boxIdCounter;
ConfBox3dPredicate* ConfBox3d::predicate;
vector<ConfBox3d*> ConfBox3d::boxes;


ConfBox3d::ConfBox3d(Point3d pp, double ww):
  depth(1), mB(pp), width(ww),
  parent(0), status(UNKNOWN),
  pSet(0), dist2Source(-1), heapId(-1), prev(0), visited(false), inQueue(false), inVorQueue(false), bidirection(0),
  inSource(false), distDij(-1){
  box = new Box3d(pp, ww);
  rot_mB.set(0,0,0);
  rot_width = -2;
  boxIdCounter++;
  boxId = boxIdCounter;
  rB = (ww * sqrt(3))/2;
  boxes.push_back(this);
  predicate = new ConfBox3dPredicate();
  children.clear();

  ring = Circle3d(pp, 0, Vector(0,0,0));
}

ConfBox3d::ConfBox3d(Point3d pp, double ww, Point3d rot_pp, double rot_ww):
  depth(1), mB(pp), width(ww), rot_mB(rot_pp), rot_width(rot_ww),
  parent(0), status(UNKNOWN),
  pSet(0), dist2Source(-1), heapId(-1), prev(0), visited(false), inQueue(false), inVorQueue(false), bidirection(0),
  inSource(false), distDij(-1){
  box = new Box3d(pp, ww);
  rot = new Rot3dSide(rot_pp, rot_ww);
  boxIdCounter++;
  boxId = boxIdCounter;
  rB = (ww * sqrt(3))/2;
  boxes.push_back(this);
  predicate = new ConfBox3dPredicate();
  children.clear();

  rot_pp = Rot3dSide::cube2sphere(rot_pp);
  Vector direction(rot_pp.X(), rot_pp.Y(), rot_pp.Z());
  direction.normalize();
  ring = Circle3d(pp, r0, direction);
}

bool ConfBox3d::isLeaf() {
  return children.empty();
}

bool ConfBox3d::getRot(float rot[3]) {
  if (!this->rot) {
    return false;
  }
  rot[0] = this->rot->origin->X();
  rot[1] = this->rot->origin->Y();
  rot[2] = this->rot->origin->Z();
  return true;
}

// have bug ......
ConfBox3d* ConfBox3d::getBox(Point3d pp, Point3d rot_pp) {


//  fprintf(g_fptr, "get box (%f, %f, %f) (%f, %f, %f)\n",
//          pp.X(), pp.Y(), pp.Z(),
//          rot_pp.X(), rot_pp.Y(), rot_pp.Z());

//  fprintf(g_fptr, "is leaf %d\n", isLeaf());
//  fprintf(g_fptr, "box (%f, %f, %f) %f %d\n",
//          box->origin->X(), box->origin->Y(), box->origin->Z(), box->width, box->containsPoint(pp));
//  if(rot_width != -2){
//    fprintf(g_fptr, "rot (%f, %f, %f) %f %d\n",
//            rot->origin->X(), rot->origin->Y(), rot->origin->Z(), rot->width, rot->containsPoint(rot_pp));
//  }

  if (isLeaf()) {
    bool containsPoint = box->containsPoint(pp) && (!rot || rot->containsPoint(rot_pp));
    return containsPoint ? this : 0;
  }
  else {
    for (unsigned i = 0; i < children.size(); ++i) {
      ConfBox3d* b = children[i]->getBox(pp, rot_pp);
      if (b != 0) {
        return b;
      }
    }
    return 0;
  }
}

void ConfBox3d::addCorner(Corner* c) {
  Corners.push_back(c);
  vorCorners.push_back(c);
}

void ConfBox3d::addEdge(Edge* e) {
  Edges.push_back(e);
  vorEdges.push_back(e);
}

void ConfBox3d::addWall(Wall* w) {
  Walls.push_back(w);
  vorWalls.push_back(w);
}

void ConfBox3d::addSphere(Sphere* s) {
  //spheres.push_back(s);
}

bool ConfBox3d::isFree() {
  return status == FREE;
}

Status ConfBox3d::getStatus() {
  if(status == UNKNOWN) predicate->classification(this, this->ring);
  return status;
}

vector<ConfBox3d*> ConfBox3d::getChildren() {
  return children;
}

vector<ConfBox3d*> ConfBox3d::getNeighbors() {
  return neighbors;
}

bool ConfBox3d::contains(ConfBox3d* other){
  return box->containsPoint(other->mB) &&
       (!rot || rot->containsPoint(other->rot_mB));
}
bool ConfBox3d::rotIsOverlapped(ConfBox3d* other){
  return !rot || rot->isOverlap(other->rot);
}

// When any of the boxes does not have restrictions on the rotational degrees of freedom, just compare their translation boxes;
// Otherwise, one of the components should be the same and the other adjacent
bool ConfBox3d::isNeighbor(ConfBox3d* other) {
  if (!rot || !other->rot) {
    return box->isAdjacent(other->box);
  }
  else {
    return (rot->isIdentical(other->rot) && box->isAdjacent(other->box)) ||
           (rot->isOverlap(other->rot) && box->isAdjacent(other->box)) ||
           (rot->isAdjacent(other->rot) && box->isIdentical(other->box));
  }
}

bool ConfBox3d::isAdjacent(ConfBox3d* other){
  if (rot == NULL || other->rot == NULL) {
    return box->isAdjacent(other->box);
  }
  else {
    return (box->isAdjacent(other->box)) && rot->isAdjacent(other->rot);
  }
}

bool ConfBox3d::split(double epsilon) {

  if (!children.empty()) {
    // Subdivided already
    return false;
  }

  if (box->split(epsilon)) {
    // Can still subdivide the translational component
    vector<Box3d*> box3dChildren = box->children;
    for (unsigned i = 0; i < box3dChildren.size(); ++i) {
      Box3d* b = box3dChildren[i];
      children.push_back(new ConfBox3d(*b->origin, b->width));
    }
  }
  else if (!rot) {
    // Need to subdivide into six sides
    for (int i = 0; i < 3; i++) {
      for (int j = -1; j < 3; j += 2) {
        double r[3] = {0, 0, 0};
        r[i] = j;
        children.push_back(new ConfBox3d(*box->origin, box->width, Point3d(r[0], r[1], r[2]), 2));
      }
    }
  }
  else if (rot->split(epsilon/r0)) {
    // Can split the rotational component
    vector<Rot3dSide*> rotChildren = rot->children;
    for (unsigned i = 0; i < rotChildren.size(); ++i) {
      Rot3dSide* r = rotChildren[i];
      children.push_back(new ConfBox3d(*box->origin, box->width, *r->origin, r->width));
    }
  }
  else {
    // Cannot subdivide, epsilon reached
    return false;
  }


  for (int i = 0; i < children.size(); i++) {
    children[i]->parent = this;
    children[i]->depth = this->depth + 1;

    children[i]->Walls.insert(children[i]->Walls.begin(),
                              Walls.begin(),
                              Walls.end());
    children[i]->Edges.insert(children[i]->Edges.begin(),
                              Edges.begin(),
                              Edges.end());
    children[i]->Corners.insert(children[i]->Corners.begin(),
                                Corners.begin(),
                                Corners.end());


    children[i]->vorWalls.insert(children[i]->vorWalls.begin(),
                                 vorWalls.begin(),
                                 vorWalls.end());
    children[i]->vorEdges.insert(children[i]->vorEdges.begin(),
                                 vorEdges.begin(),
                                 vorEdges.end());
    children[i]->vorCorners.insert(children[i]->vorCorners.begin(),
                                   vorCorners.begin(),
                                   vorCorners.end());
  }


  for (unsigned i = 0; i < children.size(); ++i) {
    for (unsigned j = i + 1; j < children.size(); ++j) {

      if(verbose){
        fprintf(g_fptr, "children 1 children neighbor (%f, %f, %f) %f",
                children[i]->mB.X(), children[i]->mB.Y(), children[i]->mB.Z(), children[i]->width);
        if(children[i]->rot_width != -2)
          fprintf(g_fptr, " Br (%f, %f, %f) %f",
                  children[i]->rot_mB.X(), children[i]->rot_mB.Y(), children[i]->rot_mB.Z(), children[i]->rot_width);
        fprintf(g_fptr, "\n");

        fprintf(g_fptr, "children children 2 neighbor (%f, %f, %f) %f",
              children[j]->mB.X(), children[j]->mB.Y(), children[j]->mB.Z(), children[j]->width);
        if(children[j]->rot_width != -2)
          fprintf(g_fptr, " Br (%f, %f, %f) %f",
                children[j]->rot_mB.X(), children[j]->rot_mB.Y(), children[j]->rot_mB.Z(), children[j]->rot_width);
        fprintf(g_fptr, "\n");
      }

      if (children[i]->isNeighbor(children[j])) {
        if(verbose)
        fprintf(g_fptr, "is neighbor\n");

        children[i]->neighbors.push_back(children[j]);
        children[j]->neighbors.push_back(children[i]);
      }
    }
  }
  for (unsigned i = 0; i < children.size(); ++i) {
    for (unsigned j = 0; j < neighbors.size(); ++j) {

      if(neighbors[j]->isLeaf()){

        if(verbose){
          fprintf(g_fptr, "children 1 neighbor neighbor (%f, %f, %f) %f",
                  children[i]->mB.X(), children[i]->mB.Y(), children[i]->mB.Z(), children[i]->width);
          if(children[i]->rot_width != -2)
            fprintf(g_fptr, " Br (%f, %f, %f) %f",
                    children[i]->rot_mB.X(), children[i]->rot_mB.Y(), children[i]->rot_mB.Z(), children[i]->rot_width);
          fprintf(g_fptr, "\n");

          fprintf(g_fptr, "children neighbor 2 neighbor (%f, %f, %f) %f",
                neighbors[j]->mB.X(), neighbors[j]->mB.Y(), neighbors[j]->mB.Z(), neighbors[j]->width);
          if(neighbors[j]->rot_width != -2)
            fprintf(g_fptr, " Br (%f, %f, %f) %f",
                  neighbors[j]->rot_mB.X(), neighbors[j]->rot_mB.Y(), neighbors[j]->rot_mB.Z(), neighbors[j]->rot_width);
          fprintf(g_fptr, "\n");
        }

        if (children[i]->isNeighbor(neighbors[j])) {
          if(verbose)
          fprintf(g_fptr, "is neighbor\n");

          children[i]->neighbors.push_back(neighbors[j]);
          neighbors[j]->neighbors.push_back(children[i]);
        }
      }
      else{
        BoxQueue* PQ = new SeqQueue();
        PQ->push(neighbors[j]);
        while(!PQ->empty()){
          ConfBox3d* current = PQ->extract();
          for(unsigned k=0;k<current->children.size();++k){
            if(current->children[k]->isLeaf()){

              if(verbose){
                fprintf(g_fptr, "children 3 neighbor neighbor (%f, %f, %f) %f",
                        children[i]->mB.X(), children[i]->mB.Y(), children[i]->mB.Z(), children[i]->width);
                if(children[i]->rot_width != -2)
                  fprintf(g_fptr, " Br (%f, %f, %f) %f",
                          children[i]->rot_mB.X(), children[i]->rot_mB.Y(), children[i]->rot_mB.Z(), children[i]->rot_width);
                fprintf(g_fptr, "\n");

                fprintf(g_fptr, "children neighbor 4 neighbor (%f, %f, %f) %f",
                      current->children[k]->mB.X(), current->children[k]->mB.Y(), current->children[k]->mB.Z(), current->children[k]->width);
                if(current->children[k]->rot_width != -2)
                  fprintf(g_fptr, " Br (%f, %f, %f) %f",
                        current->children[k]->rot_mB.X(), current->children[k]->rot_mB.Y(), current->children[k]->rot_mB.Z(), current->children[k]->rot_width);
                fprintf(g_fptr, "\n");
              }

              if(children[i]->isNeighbor(current->children[k])){
                if(verbose)
                fprintf(g_fptr, "is neighbor\n");

                children[i]->neighbors.push_back(current->children[k]);
                current->children[k]->neighbors.push_back(children[i]);
              }
            }
            else{
              if(children[i]->isAdjacent(current->children[k]))
                PQ->push(current->children[k]);
            }
          }
        }
      }
    }
  }

  return true;
}

ConfBox3d* ConfBox3d::makePointBox(Point3d pp, Point3d rot_pp) {
  if(abs(rot_pp.X())*abs(rot_pp.Y())*abs(rot_pp.Z()) > 0)
    return new ConfBox3d(pp, 0, rot_pp, 0);
  else
    return new ConfBox3d(pp, 0);
}
