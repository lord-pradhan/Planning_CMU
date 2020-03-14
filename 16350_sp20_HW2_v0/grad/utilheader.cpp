#include "utilheader.h"
#include <stack>

NodePRM::NodePRM(): expanded(false), G_val( std::numeric_limits<double>::infinity() ){}

double NodePRM::getNthCoord( int n ) const{

    return elemCoords[n];    
}

std::vector <double> NodePRM::getCoords() const{

	return elemCoords;
}

int NodePRM::getID() const{

	return elemId;
}

std::vector<int> NodePRM::getAdjIDs() const{

	return adjIDs;
}

double NodePRM::getG() const{

	return G_val;
}

bool NodePRM::isExpanded() const{

	return expanded;
}

void NodePRM::setCoord(std::vector <double> coordsIn){

    elemCoords = coordsIn;
}

void NodePRM::insertAdj(int adjID_){

    adjIDs.push_back( adjID_ );
}

void NodePRM::setElemID(int elemId_){

    elemId = elemId_;
}

void NodePRM::setG(double G_val_){

	G_val = G_val_;
}

void NodePRM::expand(){

	expanded = true;
}

double randomDouble( double LB, double UB ){

  std::uniform_real_distribution<double> unif(LB,UB);
  std::default_random_engine re;
  return unif(re);
}

double distanceFn( NodePRM node1, NodePRM node2 ){

  std::vector<double> difference;
  std::transform( node1.getCoords().begin(), node1.getCoords().end(), node2.getCoords().begin(), 
    difference.begin() , std::minus<double>());

  double distance=0;
  for (auto i : difference){

    distance += i*i;
  }

  return distance;
}

bool same_component( NodePRM pushNodeIn, NodePRM existingNodeIn, std::vector<NodePRM> listOfNodesIn ){

  std::stack <NodePRM> stackDFS;
  std::vector<bool> visited( listOfNodesIn.size() , false );

  stackDFS.push(pushNodeIn);

  while(!stackDFS.empty()){

    NodePRM topNode = stackDFS.top();
    stackDFS.pop();

    if (!visited[ topNode.getID() ]){

      visited[ topNode.getID() ] = true;
    }

    for( auto i : topNode.getAdjIDs() ){

      if( !visited[i] ){

        if( pushNodeIn.getID() == i )
          return true;

        else
          stackDFS.push( listOfNodesIn[i] );
      }
    }
  }
  
  return false;
}


bool can_connect( NodePRM pushNodeIn, NodePRM existingNodeIn , double* map, int x_size, int y_size){

	int numChecks = 20;

	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< pushNodeIn.getCoords().size() ; j++){

			xVals[j] = pushNodeIn.getCoords()[j] + i * 
					( existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] ) / 20.0;
		}

		double anglesArr[ pushNodeIn.getCoords().size() ];
		std::copy( xVals.begin(), xVals.end(), anglesArr );

		if(!IsValidArmConfiguration( anglesArr, pushNodeIn.getCoords().size(), map, x_size, y_size) )
			return false;
	}

	return true;
}
