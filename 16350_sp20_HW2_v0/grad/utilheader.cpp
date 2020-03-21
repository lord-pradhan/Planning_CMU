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
	return sqrt(distance);
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

		if(IsValidArmConfiguration( xVals.data(), pushNodeIn.getCoords().size(), map, x_size, y_size)==0 ){
			
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
NodeRRT::NodeRRT(){
	// parent = nullptr;	
}

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
	return sqrt(distance);
}


// class for priority queue
NodePQ::NodePQ( NodeRRT* nodeIn_, std::vector<double> currSamplePt_ ): 
nodeIn(nodeIn_), currSamplePt(currSamplePt_) {}

NodePQ::NodePQ(){}

double NodePQ::getDist() const{

	return distanceRRT( currSamplePt, nodeIn->getCoords() );
}


int extend( NodeRRT* root_, NodeRRT* tail_, std::vector<double> currSamplePt_ , double eps_, double* map, 
	int x_size, int y_size, std::vector<double> endCoord_, double tol ){

	NodeRRT* nearestNode = nearestNeighbour( currSamplePt_, root_ );

	NodeRRT* newNode = new NodeRRT;

	int returnKey = newConfig( currSamplePt_, nearestNode, newNode, eps_, map, x_size, y_size, endCoord_ , tol);
	// '0' = reached, '1' = advanced, '2' = trapped, '3' = goal reached 

	if( returnKey == 0){

		newNode->setParent(nearestNode);
		nearestNode->addChild( newNode );
		// delete newNode;
		return 0;
	}

	else if( returnKey == 1 ){

		newNode->setParent(nearestNode);
		nearestNode->addChild( newNode );
		// delete newNode;
		return 1;
	}

	else if( returnKey == 2 ){

		// newNode = nullptr;
		delete newNode;
		return 2;	
	}

	else if(returnKey == 3){

		tail_->setCoord( newNode->getCoords() );
		tail_->setParent(nearestNode);
		nearestNode->addChild(tail_);
		// newNode = nullptr;
		// delete newNode;
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

	// mexPrintf("size of min_queue is %d \n", min_queue.size());
	// mexEvalString("drawnow");

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
 double* map, int x_size, int y_size, std::vector<double> endCoord_, double tol ){

	// mexPrintf("Entered newConfig\n");
	// mexEvalString("drawnow");
	int numChecks = 20;
	double distanceTemp = distanceRRT( nearestNode_->getCoords(), currSamplePt_);
	double ratio_temp = std::min( distanceTemp , eps_ ) / distanceTemp;
	// ratio_temp /= distanceRRT( nearestNode_->getCoords(), currSamplePt_ );
	// mexPrintf("Ratio is %lf \n distance is %lf \n", ratio_temp, distanceTemp );
	// mexEvalString("drawnow");

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
													- nearestNode_->getCoords()[j] ) ) ;
	}


	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

			xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
													- nearestNode_->getCoords()[j] ) / (double) numChecks );
		}

		if(IsValidArmConfiguration( xVals.data(), xVals.size(), map, x_size, y_size) == 0){

			if(i==0){
				return 2;
			}
			else{
				newNode_->setCoord( xVals );
				return 1;
			}
		}

		if(reachedGoal(xVals, endCoord_, tol)){

			newNode_->setCoord(xVals);
			return 3;
		}
	}

	if( eps_ >= distanceTemp ){

		newNode_->setCoord( currSamplePt_ );
		return 0;
	}
	else{

		newNode_->setCoord( xEnd );
		return 1;
	}
}

bool reachedGoal( std::vector<double> xVals_, std::vector<double> endCoord_, double tol){

	// double tol = 4.0 * PI / 180.0;

	for(int i=0 ; i< endCoord_.size() ; i++){

		if( (xVals_[i] - endCoord_[i]) > tol || (xVals_[i] - endCoord_[i]) < -tol )

			return false;
	}

	return true;
}





// functions for RRT* //////

int extend_star( NodeRRT* tree1_, std::vector<double> currSamplePt_ , NodeRRT* newNode_,
 double eps_, double* map, int x_size, int y_size ){

	NodeRRT* nearestNode = nearestNeighbour( currSamplePt_, tree1_ );

	int returnKey = newConfig_star( currSamplePt_, nearestNode, newNode_, eps_,  map, x_size, y_size);
	// '0' = reached, '1' = advanced, '2' = trapped, '3' = goal reached 

	if( returnKey == 0){

		newNode_->setParent(nearestNode);
		nearestNode->addChild( newNode_ );
		// delete newNode;
		return 0;
	}

	else if( returnKey == 1 ){

		newNode_->setParent(nearestNode);
		nearestNode->addChild( newNode_ );
		// delete newNode;
		return 1;
	}

	else if( returnKey == 2 ){

		// newNode = nullptr;
		delete newNode_;
		return 2;	
	}

	// else if(returnKey == 3){

	// 	tail_->setCoord( newNode->getCoords() );
	// 	tail_->setParent(nearestNode);
	// 	nearestNode->addChild(tail_);
	// 	// newNode = nullptr;
	// 	// delete newNode;
	// 	return 3;
	// }
}

int newConfig_star( std::vector<double> currSamplePt_, NodeRRT* nearestNode_, NodeRRT* newNode_, double eps_,
				 double* map, int x_size, int y_size ){

	// mexPrintf("Entered newConfig\n");
	// mexEvalString("drawnow");
	int numChecks = 20;
	double distanceTemp = distanceRRT( nearestNode_->getCoords(), currSamplePt_);
	double ratio_temp = std::min( distanceTemp , eps_ ) / distanceTemp;
	// ratio_temp /= distanceRRT( nearestNode_->getCoords(), currSamplePt_ );
	// mexPrintf("Ratio is %lf \n distance is %lf \n", ratio_temp, distanceTemp );
	// mexEvalString("drawnow");

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
													- nearestNode_->getCoords()[j] ) ) ;
	}


	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

			xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
													- nearestNode_->getCoords()[j] ) / (double) numChecks );
		}

		if(IsValidArmConfiguration( xVals.data(), xVals.size(), map, x_size, y_size) == 0){

			if(i==0){
				return 2;
			}
			else{
				newNode_->setCoord( xVals );
				return 1;
			}
		}

		// if(reachedGoal(xVals, endCoord_, tol)){

		// 	newNode_->setCoord(xVals);
		// 	return 3;
		// }
	}

	if( eps_ >= distanceTemp ){

		newNode_->setCoord( currSamplePt_ );
		return 0;
	}
	else{

		newNode_->setCoord( xEnd );
		return 1;
	}
}

int connect_star( NodeRRT* tree2_, NodeRRT* newNode1_, NodeRRT* newNode2_ , 
	double eps_, double* map, int x_size, int y_size){
	// '0' = reached, '1' = advanced, '2' = trapped

	int extKey;

	do{ 
		extKey = extend_star( tree2_, newNode1_->getCoords(), newNode2_, eps_, map, x_size, y_size );

	} while(extKey!=1);

	if (extKey==0){
		
		return 0;
	}
	else if(extKey==2)
		return 2;
}

void swapTrees( NodeRRT* tree1_, NodeRRT* tree2_ ){

	NodeRRT temp = *tree1_;
	*tree1_ = *tree2_;
	*tree2_ = temp;
}