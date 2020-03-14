#include <math.h>
// #include "kdtree.h"
#include <time.h> 
#include <random>
#include <stdio.h> 
#include <stdlib.h> 
#include <algorithm>
#include <vector>
#include <limits>

#ifndef UTILHEADER_H
#define UTILHEADER_H

// define classes needed
class NodePRM{

private:
  int elemId;
  std::vector <int> adjIDs;
  std::vector <double> elemCoords;
  double G_val;
  bool expanded;

public:
  NodePRM();

// Get stuff
  double getNthCoord( int n ) const;

  std::vector <double> getCoords() const;

  int getID() const;

  std::vector<int> getAdjIDs() const;

  double getG() const;

  bool isExpanded() const;

// Set stuff
  void setCoord(std::vector <double> coordsIn);

  void insertAdj(int adjID_);

  void setElemID(int elemId_);

  void setG( double G_val_ );

  void expand();

};

// define useful functions
double randomDouble( double LB, double UB );

double distanceFn( NodePRM node1, NodePRM node2 );


bool same_component( NodePRM pushNodeIn, NodePRM existingNodeIn, std::vector<NodePRM> listOfNodesIn );

bool can_connect( NodePRM pushNodeIn, NodePRM existingNodeIn , double* map, int x_size, int y_size);


int IsValidArmConfiguration(double* angles, int numofDOFs, double*  map,
       int x_size, int y_size);

int IsValidLineSegment(double x0, double y0, double x1, double y1, double*  map,
       int x_size,
       int y_size);

typedef struct {
  int X1, Y1;
  int X2, Y2;
  int Increment;
  int UsingYIndex;
  int DeltaX, DeltaY;
  int DTerm;
  int IncrE, IncrNE;
  int XIndex, YIndex;
  int Flipped;
} bresenham_param_t;// ;

int get_next_point(bresenham_param_t *params);

void get_current_point(bresenham_param_t *params, int *x, int *y);

void get_bresenham_parameters(int p1x, int p1y, int p2x, int p2y, bresenham_param_t *params);

void ContXY2Cell(double x, double y, short unsigned int* pX, short unsigned int *pY, int x_size, int y_size);

#endif