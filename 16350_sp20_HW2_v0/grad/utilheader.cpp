#include "utilheader.h"
#include <stack>
#include "mex.h"
#include <iostream> // std::cout 
#include <functional> // std::plus 
#include <algorithm> // std::transform 
#include <math.h>
// #include "mex.h"
// #include "kdtree.h"
#include <time.h> 
#include <random>
#include <stdio.h> 
#include <stdlib.h> 
#include <vector>
#include <queue>
#include <limits>
#include <bits/stdc++.h> 
#include <iostream>


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

double randomDouble( double min, double max ){

  // std::uniform_real_distribution<double> unif(LB,UB);
  // std::default_random_engine re;
  // return unif(re);
	return (max - min) * ( (double)rand() / (double)RAND_MAX ) + min;

}

double distanceFn( NodePRM node1, NodePRM node2 ){

	// mexPrintf("Entered distanceFn \n");
	// mexEvalString("drawnow");
	// std::vector<double> difference;
	double distance=0;

	for(int i=0; i< node1.getCoords().size();i++){

		distance += (node1.getCoords()[i] - node2.getCoords()[i])*
					(node1.getCoords()[i] - node2.getCoords()[i]);
	}	
	// std::transform( node1.getCoords().begin(), node1.getCoords().end(), node2.getCoords().begin(), 
	// difference, std::minus<double>() );

	// for (auto i : difference){

	// 	distance += i*i;
	// }

	// mexPrintf("Exiting distanceFn \n");
	// mexEvalString("drawnow");
	return distance;
}

bool same_component( NodePRM pushNodeIn, NodePRM existingNodeIn, std::vector<NodePRM> listOfNodesIn ){

	std::stack <NodePRM> stackDFS;
	std::vector<bool> visited( listOfNodesIn.size() , false );

	stackDFS.push(pushNodeIn);

	mexPrintf("entering same_component while loop \n");
	mexEvalString("drawnow");

	while(!stackDFS.empty()){

		NodePRM topNode = stackDFS.top();
		stackDFS.pop();

		if (!visited[ topNode.getID() ]){

			visited[ topNode.getID() ] = true;
		}

		for( auto i : topNode.getAdjIDs() ){

			if( !visited[i] ){

				if( pushNodeIn.getID() == i ){
				  
					mexPrintf("exiting same_component while loop true \n");
					mexEvalString("drawnow");
					return true;
				}

				else
					stackDFS.push( listOfNodesIn[i] );
			}
		}
	}
	mexPrintf("exiting same_component while loop false \n");
	mexEvalString("drawnow");

	return false;
}


bool can_connect( NodePRM pushNodeIn, NodePRM existingNodeIn , double* map, 
	int x_size, int y_size){

	mexPrintf("Entered can_connect \n");
	mexEvalString("drawnow");
	int numChecks = 20;

	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< pushNodeIn.getCoords().size() ; j++){

			xVals.push_back( pushNodeIn.getCoords()[j] + (double) (i+1) * 
					( existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] ) / 20.0 );
		}

		// double anglesArr[ pushNodeIn.getCoords().size() ];
		// std::copy( xVals.begin(), xVals.end(), anglesArr );

		if(!IsValidArmConfiguration( xVals.data(), pushNodeIn.getCoords().size(), map, x_size, y_size) ){
			
			mexPrintf("Exiting can_connect false \n");
			mexEvalString("drawnow");
			return false;
		}
	}

	mexPrintf("Exiting can_connect true\n");
	mexEvalString("drawnow");
	return true;
}

///////// for RRT  ///////
NodeRRT::NodeRRT(){}

std::vector <double> NodeRRT::getCoords() const{

	return elemCoords;
}

NodeRRT* NodeRRT::getParent() const{

	return parent;
}

std::vector<NodeRRT*> NodeRRT::getChildren() const{

	return children;
}

void NodeRRT::addChild( NodeRRT*  child_){

	children.push_back( child_ );
}

void NodeRRT::setParent( NodeRRT* parent_ ){

	parent = parent_;
}

void NodeRRT::setCoord(std::vector <double> coordsIn){

	elemCoords = coordsIn;
}



//useful functions
double distanceRRT( std::vector<double> vect1, std::vector<double> vect2 ){

	// mexPrintf("Entered distanceFn \n");
	// mexEvalString("drawnow");
	// std::vector<double> difference;
	double distance=0;

	for(int i=0; i< vect1.size() ; i++){

		distance += (vect1[i] - vect2[i])*
					(vect1[i] - vect2[i]);
	}
	// std::transform( node1.getCoords().begin(), node1.getCoords().end(), node2.getCoords().begin(), 
	// difference, std::minus<double>() );

	// for (auto i : difference){

	// 	distance += i*i;
	// }

	// mexPrintf("Exiting distanceFn \n");
	// mexEvalString("drawnow");
	return distance;
}


// class for priority queue
NodePQ::NodePQ( NodeRRT* nodeIn_, std::vector<double> currSamplePt_ ): 
nodeIn(nodeIn_), currSamplePt(currSamplePt_) {}

NodePQ::NodePQ(){}

double NodePQ::getDist() const{

	return distanceRRT( currSamplePt, nodeIn->getCoords() );
}


int extend( NodeRRT* root_, NodeRRT* tail_, std::vector<double> currSamplePt_ , int eps_, double* map, 
	int x_size, int y_size, std::vector<double> endCoord_ ){

	NodeRRT* nearestNode = nearestNeighbour( currSamplePt_, root_ );

	NodeRRT* newNode;

	int returnKey = newConfig( currSamplePt_, nearestNode, newNode, eps_, map, x_size, y_size, endCoord_ );
	// '0' = reached, '1' = advanced, '2' = trapped, '3' = goal reached 

	if( returnKey == 0){

		newNode->setParent(nearestNode);
		nearestNode->addChild( newNode );
		return 0;
	}

	else if( returnKey == 1 ){

		newNode->setParent(nearestNode);
		nearestNode->addChild( newNode );
		return 1;
	}

	else if( returnKey == 2 ){

		newNode = nullptr;
		return 2;	
	}

	else if(returnKey == 3){

		tail_->setCoord( newNode->getCoords() );
		tail_->setParent(nearestNode);
		nearestNode->addChild(tail_);
		newNode = nullptr;
		return 3;
	}
}



// struct CompareNN{
//     bool operator()(NodePQ const &n1 , NodePQ const &n2) {
//         // return "true" if "p1" is ordered before "p2", for example:
//         // long eps = 1;
//         return n1.getDist() > n2.getDist();
//     }
// };

NodeRRT* nearestNeighbour( std::vector<double> currSamplePt_, NodeRRT* root_ ){

	std::priority_queue< NodePQ, std::vector<NodePQ>, CompareNN > min_queue;

	NodePQ nodeTemp( root_, currSamplePt_ );
	min_queue.push( nodeTemp );

	treeDFS( root_, currSamplePt_ , min_queue);

	return min_queue.top().nodeIn;
}


void treeDFS( NodeRRT* nodeIn, std::vector<double> currSamplePt_, 
	std::priority_queue< NodePQ, std::vector<NodePQ>, CompareNN > &min_queue ){

	if ( !nodeIn->getChildren().empty() ){

		for (auto i : nodeIn->getChildren()){

			NodePQ nodeTemp( i, currSamplePt_ );
			min_queue.push( nodeTemp );
			treeDFS(  i, currSamplePt_, min_queue );
		}
		return;
	}
	else {

		return;
	}
}


int newConfig( std::vector<double> currSamplePt_, NodeRRT* nearestNode_, NodeRRT* newNode_ , double eps_, 
 double* map, int x_size, int y_size, std::vector<double> endCoord_ ){

	mexPrintf("Entered newConfig\n");
	mexEvalString("drawnow");
	int numChecks = 20;
	double ratio_temp = std::min( distanceRRT( nearestNode_->getCoords(), currSamplePt_ ), eps_ );
	ratio_temp /= distanceRRT( nearestNode_->getCoords(), currSamplePt_ );

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
													- nearestNode_->getCoords()[j] ) ) ;
	}


	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

			xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
													- nearestNode_->getCoords()[j] ) / 20.0 );
		}

		if(!IsValidArmConfiguration( xVals.data(), xVals.size(), map, x_size, y_size) ){

			if(i==0){
				return 2;
			}
			else{
				newNode_->setCoord( xVals );
				return 1;
			}
		}

		if(reachedGoal(xVals, endCoord_)){

			newNode_->setCoord(xVals);
			return 3;
		}
	}

	if( eps_ >= distanceRRT( nearestNode_->getCoords(), currSamplePt_ ) ){

		newNode_->setCoord( currSamplePt_ );
		return 0;
	}
	else{

		newNode_->setCoord( xEnd );
		return 1;
	}

	mexPrintf("Exiting can_connect true\n");
	mexEvalString("drawnow");
	return true;
}

bool reachedGoal( std::vector<double> xVals_, std::vector<double> endCoord_){

	double tol = 2 * PI / 180.0;

	for(int i ; i< endCoord_.size() ; i++){

		if( fabs(xVals_[i] - endCoord_[i]) > tol )

			return false;
	}

	return true;
}