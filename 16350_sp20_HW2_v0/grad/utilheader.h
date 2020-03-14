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

#endif