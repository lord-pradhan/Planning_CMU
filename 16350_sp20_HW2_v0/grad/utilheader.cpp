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
#include <list>
#include <chrono> 
using namespace std::chrono;


#define PI 3.141592654

//the length of each link in the arm (should be the same as the one used in runtest.m)
#define LINKLENGTH_CELLS 10


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

		distance += std::min( (node1.getCoords()[i] - node2.getCoords()[i])*
		(node1.getCoords()[i] - node2.getCoords()[i]), (2*PI - fabs(node1.getCoords()[i] - node2.getCoords()[i]))*
		(2*PI - fabs(node1.getCoords()[i] - node2.getCoords()[i]) ));
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

	stackDFS.push(existingNodeIn);

	while(!stackDFS.empty()){

		NodePRM topNode = stackDFS.top();
		stackDFS.pop();

		if (!visited[ topNode.getID() ]){

			visited[ topNode.getID() ] = true;
		}

		for( auto i : topNode.getAdjIDs() ){

			if( !visited[i] ){

				if( pushNodeIn.getID() == i ){

					return true;
				}

				else
					stackDFS.push( listOfNodesIn[i] );
			}
		}
	}

	return false;
}


bool can_connect( NodePRM pushNodeIn, NodePRM existingNodeIn , double* map, 
	int x_size, int y_size, int numChecks){

	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j =0; j< pushNodeIn.getCoords().size() ; j++){

			if(existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] >= PI){
				
				xVals.push_back( pushNodeIn.getCoords()[j] + (double) (i+1) * 
						(existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] - 2*PI) / numChecks );
				// printf("greater than pi : %d\n", j);
			}
			else if(existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] <= -PI){

				xVals.push_back( pushNodeIn.getCoords()[j] + (double) (i+1) * 
						(existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] + 2*PI) / numChecks );	
				// printf("lesser than -pi : %d \n", j);
			}
			else{

				xVals.push_back( pushNodeIn.getCoords()[j] + (double) (i+1) * 
						(existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j])/numChecks );
				// printf("in between : %d\n", j);
			}
		}

		if(IsValidArmConfiguration( xVals.data(), pushNodeIn.getCoords().size(), map, x_size, y_size)==0 ){
 
			return false;
		}
	}

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

		distance += std::min( (vect1[i] - vect2[i])* (vect1[i] - vect2[i]) , 
						(2*PI - fabs(vect1[i] - vect2[i]) )*(2*PI - fabs(vect1[i] - vect2[i]) ) );
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
	int x_size, int y_size, std::vector<double> endCoord_, double tol, int numChecks ){

	NodeRRT* nearestNode = nearestNeighbour( currSamplePt_, root_ );

	NodeRRT* newNode = new NodeRRT;

	int returnKey = newConfig( currSamplePt_, nearestNode, newNode, eps_, map, x_size, 
				y_size, endCoord_ , tol, numChecks);
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


NodeRRT* nearestNeighbour( std::vector<double> currSamplePt_, NodeRRT* root_ ){

	double min_val = distanceRRT( root_->getCoords(), currSamplePt_ );
	NodeRRT* nearest = root_;
	treeDFS( root_, currSamplePt_ , nearest, min_val );

	// auto stopNN = high_resolution_clock::now();
 //    auto duration1 = duration_cast<nanoseconds>(stopNN - startNN);
 //    mexPrintf("nearest neighbour takes %d time each cycle \n", duration1.count());
	// printf("exited NNstar \n");
	return nearest;
}


void treeDFS( NodeRRT* nodeIn, std::vector<double> currSamplePt_, NodeRRT* &nearest_, 
	double &min_val_){

	if ( !nodeIn->getChildren().empty() ){

		for (auto i : nodeIn->getChildren()){

			if( distanceRRT(i->getCoords(), currSamplePt_) < min_val_ )
			{
				min_val_ = distanceRRT(i->getCoords(), currSamplePt_);
				nearest_ = i;
			}
			treeDFS(  i, currSamplePt_, nearest_, min_val_ );
		}
		return;
	}
	else {

		return;
	}
}


int newConfig( std::vector<double> currSamplePt_, NodeRRT* nearestNode_, NodeRRT* newNode_ , double eps_, 
 double* map, int x_size, int y_size, std::vector<double> endCoord_, double tol, int numChecks ){

	// mexPrintf("Entered newConfig\n");
	// mexEvalString("drawnow");
	// int numChecks = 500;
	double distanceTemp = distanceRRT( nearestNode_->getCoords(), currSamplePt_);
	double ratio_temp = std::min( distanceTemp , eps_ ) / distanceTemp;
	// ratio_temp /= distanceRRT( nearestNode_->getCoords(), currSamplePt_ );
	// mexPrintf("Ratio is %lf \n distance is %lf \n", ratio_temp, distanceTemp );
	// mexEvalString("drawnow");

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		// wrap to pi
		if(currSamplePt_[j] - nearestNode_->getCoords()[j] >= PI){

			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
								- nearestNode_->getCoords()[j] - 2*PI) ) ;
		}
		else if(currSamplePt_[j] - nearestNode_->getCoords()[j] <= -PI ){

			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
								- nearestNode_->getCoords()[j] + 2*PI) ) ;	
		}
		else{

			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*(currSamplePt_[j] 
								- nearestNode_->getCoords()[j]) ) ;
		}
	}

	std::vector<double> xValsPrev = nearestNode_->getCoords();
	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

			// wrap to pi
			if(xEnd[j] - nearestNode_->getCoords()[j] >= PI){

				xVals.push_back( nearestNode_->getCoords()[j] + 
				(double) (i+1)*( xEnd[j] - nearestNode_->getCoords()[j] - 2*PI) / (double) numChecks );
			}
			else if(xEnd[j] - nearestNode_->getCoords()[j] <= -PI){

				xVals.push_back( nearestNode_->getCoords()[j] + 
				(double) (i+1)*( xEnd[j] - nearestNode_->getCoords()[j] + 2*PI) / (double) numChecks );
			}
			else{

				xVals.push_back( nearestNode_->getCoords()[j] + 
					(double) (i+1)*( xEnd[j] - nearestNode_->getCoords()[j] ) / (double) numChecks );
			}
		}

		if(IsValidArmConfiguration( xVals.data(), xVals.size(), map, x_size, y_size) == 0){

			if(i==0){
				return 2;
			}
			else{
				newNode_->setCoord( xValsPrev );
				return 1;
			}
		}

		xValsPrev = xVals;

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





// functions for RRT connect //////

int extend_connect( NodeRRT* tree1_, std::vector<double> currSamplePt_ , NodeRRT* newNode_,
 double eps_, double* map, int x_size, int y_size , int numChecks){

	// printf("entered extend_star \n");
	NodeRRT* nearestNode = nearestNeighbour( currSamplePt_, tree1_ );

	int returnKey = newConfig_connect( currSamplePt_, nearestNode, newNode_, eps_,  map, x_size, 
					y_size, numChecks);
	// '0' = reached, '1' = advanced, '2' = trapped, '3' = goal reached 
	// printf("before setting parents and children\n");
	if( returnKey == 0){
		// printf("return key = 0\n");
		newNode_->setParent(nearestNode);
		nearestNode->addChild( newNode_ );
		// delete newNode;
		return 0;
	}

	else if( returnKey == 1 ){
		// printf("return key is 1\n");
		newNode_->setParent(nearestNode);
		nearestNode->addChild( newNode_ );
		// delete newNode;
		return 1;
	}

	else if( returnKey == 2 ){

		// newNode = nullptr;
		// printf("return key is 2\n");
		delete newNode_;
		return 2;	
	}
}

int newConfig_connect( std::vector<double> currSamplePt_, NodeRRT* nearestNode_, NodeRRT* newNode_, double eps_,
				 double* map, int x_size, int y_size, int numChecks ){

	// printf("entered newConfig_star \n"); 
	// int numChecks = 500;
	double distanceTemp = distanceRRT( nearestNode_->getCoords(), currSamplePt_);
	double ratio_temp = std::min( distanceTemp , eps_ ) / distanceTemp;

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		//wrap to pi
		if(currSamplePt_[j] - nearestNode_->getCoords()[j] >= PI){

			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
											- nearestNode_->getCoords()[j] - 2*PI) ) ;
		}
		else if(currSamplePt_[j] - nearestNode_->getCoords()[j] <= -PI){

			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
											- nearestNode_->getCoords()[j] + 2*PI) ) ;
		}
		else{

			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
											- nearestNode_->getCoords()[j] ) ) ;	
		}

	}

	std::vector<double> xValsPrev = nearestNode_->getCoords();
	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

			//wrap to pi
			if(xEnd[j] - nearestNode_->getCoords()[j] >= PI){
				xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
									- nearestNode_->getCoords()[j] - 2*PI ) / (double) numChecks );
			}
			else if(xEnd[j] - nearestNode_->getCoords()[j] <= -PI){
				xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
									- nearestNode_->getCoords()[j] + 2*PI) / (double) numChecks );	
			}
			else{
				xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
									- nearestNode_->getCoords()[j]) / (double) numChecks );		
			}
		}

		if(IsValidArmConfiguration( xVals.data(), xVals.size(), map, x_size, y_size) == 0){

			if(i==0){
				return 2;
			}
			else{
				// printf("set new node coords ad/vanced\n");
				newNode_->setCoord( xValsPrev );
				return 1;
			}
		}

		xValsPrev = xVals;
	}

	if( eps_ >= distanceTemp ){

		// printf("set new node coords reached\n");
		newNode_->setCoord( currSamplePt_ );
		return 0;
	}
	else{
		// printf("set new node coords advanced\n");
		newNode_->setCoord( xEnd );
		return 1;
	}
}

int connect( NodeRRT* tree2_, NodeRRT* newNode1_, NodeRRT* tail2_ , 
	double eps_, double* map, int x_size, int y_size, int numChecks){
	// '0' = reached, '1' = advanced, '2' = trapped
	// printf("entered connect_star \n");
	int extKey;

	do{ 
		// tail2_ = nullptr;
		NodeRRT* newNode2 = new NodeRRT;
		extKey = extend_connect( tree2_, newNode1_->getCoords(), newNode2, eps_, map, x_size, 
					y_size, numChecks );

		if (extKey==0){
		
			// tail2_->setCoord(newNode2->getCoords());
			// tail2_->setParent(newNode2->getParent());
			// tail2_ = newNode2;
			// swapTrees(tail2_, newNode2);
			*tail2_ = *newNode2;
			return 0;
		}
		else if(extKey==2)

			return 2;

		// printf("extKey returned %d\n", extKey);

	} while(extKey==1);
	// else
	// 	return 1;
	return extKey;
}

void swapTrees( NodeRRT* tree1_, NodeRRT* tree2_ ){

	NodeRRT temp = *tree1_;
	*tree1_ = *tree2_;
	*tree2_ = temp;
}



//////////////////////////////////////
///// ******* RRT* ********///////////

NodeRRT_star::NodeRRT_star(){

	parent = nullptr;
}

std::vector <double> NodeRRT_star::getCoords() const{

	return elemCoords;
}

NodeRRT_star* NodeRRT_star::getParent() const{

	return parent;
}

std::vector<NodeRRT_star*> NodeRRT_star::getChildren() const{

	return children;
}

// double NodeRRT_star::getG() const{
// 	return G_val;
// }

void NodeRRT_star::addChild( NodeRRT_star*  child_){

	children.push_back( child_ );
}

void NodeRRT_star::setParent( NodeRRT_star* parent_ ){

	parent = parent_;
}

void NodeRRT_star::setCoord(std::vector <double> coordsIn){

	elemCoords = coordsIn;
}

void NodeRRT_star::popChild( NodeRRT_star* popNode ){

	// int index=-1;
	// children.erase(std::find(children.begin(),children.end(), popNode));
		
	for(auto it = children.begin(); it!=children.end(); it++){

		if(popNode->getCoords() == (*it)->getCoords()){
		
			// printf("popped child\n");		
			children.erase( it );
			return;
		}
	}
}


// class for priority queue
NodePQ_star::NodePQ_star( NodeRRT_star* nodeIn_, std::vector<double> currSamplePt_ ): 
nodeIn(nodeIn_), currSamplePt(currSamplePt_) {}

NodePQ_star::NodePQ_star(){}

double NodePQ_star::getDist() const{

	return distanceRRT( currSamplePt, nodeIn->getCoords() );
}


NodeRRT_star* nearestNeighbour_star( std::vector<double> currSamplePt_, NodeRRT_star* root_ ){

	// auto startNN = high_resolution_clock::now();
	// printf("entered NNstar \n");
	double min_val = distanceRRT( root_->getCoords(), currSamplePt_ );
	NodeRRT_star* nearest = root_;
	treeDFS_star( root_, currSamplePt_ , nearest, min_val );

	// auto stopNN = high_resolution_clock::now();
 //    auto duration1 = duration_cast<nanoseconds>(stopNN - startNN);
 //    mexPrintf("nearest neighbour takes %d time each cycle \n", duration1.count());
	// printf("exited NNstar \n");
	return nearest;
}

void treeDFS_star( NodeRRT_star* nodeIn, std::vector<double> currSamplePt_, NodeRRT_star*
				&nearest_, double &min_val_ ){

	// printf("entered DFS\n");
	if ( !nodeIn->getChildren().empty() ){

		for (auto i : nodeIn->getChildren()){

			if( distanceRRT(i->getCoords(), currSamplePt_) < min_val_ )
			{
				min_val_ = distanceRRT(i->getCoords(), currSamplePt_);
				nearest_ = i;
			}
			treeDFS_star(  i, currSamplePt_, nearest_, min_val_ );
		}
		return;
	}
	else {

		return;
	}
}


int newConfig_star( std::vector<double> currSamplePt_, NodeRRT_star* nearestNode_, 
	NodeRRT_star* newNode_ , double eps_,  double* map, int x_size, int y_size, 
	std::vector<double> endCoord_, double tol, int numChecks ){

	double distanceTemp = distanceRRT( nearestNode_->getCoords(), currSamplePt_);
	double ratio_temp = std::min( distanceTemp , eps_ ) / distanceTemp;
	// ratio_temp /= distanceRRT( nearestNode_->getCoords(), currSamplePt_ );
	// mexPrintf("Ratio is %lf \n distance is %lf \n", ratio_temp, distanceTemp );
	// mexEvalString("drawnow");

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		// wrap to pi
		if(currSamplePt_[j] - nearestNode_->getCoords()[j] >= PI){
			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
														- nearestNode_->getCoords()[j] - 2*PI ) ) ;
		}
		else if(currSamplePt_[j] - nearestNode_->getCoords()[j] <= -PI){
			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
												- nearestNode_->getCoords()[j] + 2*PI ) ) ;
		}
		else{
			xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
												- nearestNode_->getCoords()[j] ) ) ;	
		}
	}

	std::vector<double> xValsPrev = nearestNode_->getCoords();
	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

			// wrap to pi
			if(xEnd[j] - nearestNode_->getCoords()[j] >= PI){
				xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
										- nearestNode_->getCoords()[j] - 2*PI ) / (double) numChecks );
			}
			else if(xEnd[j] - nearestNode_->getCoords()[j] <= -PI){
				xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
										- nearestNode_->getCoords()[j] + 2*PI ) / (double) numChecks );
			}
			else{
				xVals.push_back( nearestNode_->getCoords()[j] + (double) (i+1)*( xEnd[j] 
										- nearestNode_->getCoords()[j] ) / (double) numChecks );	
			}
		}

		if(IsValidArmConfiguration( xVals.data(), xVals.size(), map, x_size, y_size) == 0){

			if(i==0){

				delete newNode_;
				return 2;
			}
			else{
				newNode_->setCoord( xValsPrev );
				return 1;
			}
		}

		xValsPrev = xVals;

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


bool can_connect_star( NodeRRT_star* node1, NodeRRT_star* node2 , double* map, int x_size, 
	int y_size, int numChecks){

	// mexPrintf("Entered can_connect \n");
	// mexEvalString("drawnow");
	// int numChecks = 200;

	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< node1->getCoords().size() ; j++){

			// wrap to pi
			if(node2->getCoords()[j] - node1->getCoords()[j] >= PI){
				xVals.push_back( node1->getCoords()[j] + (double) (i+1) * 
						( node2->getCoords()[j] - node1->getCoords()[j] - 2*PI ) / numChecks );
			}
			else if(node2->getCoords()[j] - node1->getCoords()[j] <= -PI){
				xVals.push_back( node1->getCoords()[j] + (double) (i+1) * 
						( node2->getCoords()[j] - node1->getCoords()[j] + 2*PI ) / numChecks );
			}
			else{
				xVals.push_back( node1->getCoords()[j] + (double) (i+1) * 
						( node2->getCoords()[j] - node1->getCoords()[j] ) / numChecks );	
			}
		}

		if(IsValidArmConfiguration( xVals.data(), node1->getCoords().size(), map, x_size, y_size)==0 ){
			
			return false;
		}
	}

	return true;
}


void nearDFS( NodeRRT_star* nodeIn, NodeRRT_star* newNode_, std::vector<NodeRRT_star*> &nearNodes_, 
	double nearDist_ ){

	if ( !nodeIn->getChildren().empty() ){

		for (auto i : nodeIn->getChildren()){

			// NodePQ nodeTemp( i, currSamplePt_ );
			// min_queue.push( nodeTemp );
			if( distanceRRT( i->getCoords() , newNode_->getCoords() ) < nearDist_ ){

				nearNodes_.push_back( i );
			}

			nearDFS(  i, newNode_, nearNodes_, nearDist_ );
		}

		return;
	}
	else {

		return;
	}
}

void nearFn( NodeRRT_star* root_, NodeRRT_star* newNode_, std::vector<NodeRRT_star*> &nearNodes_,
 double nearDist_ ){

	// double nearDist = std::min( eps_ );

	if( distanceRRT(root_->getCoords() , newNode_->getCoords() ) < nearDist_ ){

		nearNodes_.push_back( root_ );
	}

	nearDFS(root_, newNode_, nearNodes_, nearDist_);
	return;
}


int extend_star( NodeRRT_star* root_, std::vector<NodeRRT_star*> &goalNodes_, 
	std::vector<double> currSamplePt_, double eps_, double* map, int x_size, int y_size, 
	std::vector<double> endCoord_, double tol_, double nearDist_, int numChecks){

	NodeRRT_star* nearestNode = nearestNeighbour_star( currSamplePt_, root_ );

	NodeRRT_star* newNode = new NodeRRT_star;

	int steerKey = newConfig_star( currSamplePt_, nearestNode, newNode, eps_, map, x_size, y_size,
					endCoord_, tol_, numChecks );

	if(steerKey==2){
		// delete newNode;
		// printf("trapped\n");
		return steerKey;
	}

	// if( // steerkey 
	// 	can_connect_star( nearestNode, newNode, map, x_size, y_size, numChecks ) ){

	NodeRRT_star* minNode = nearestNode;
	int minIndex;

	std::vector<NodeRRT_star*> nearNodes;

	nearFn( root_, newNode, nearNodes, nearDist_ );

	std::vector<bool> freeNearNodes;

	for(int i=0; i<nearNodes.size(); i++){

		if( can_connect_star( nearNodes[i], newNode, map, x_size, y_size, numChecks ) ){

			freeNearNodes.push_back(true);
			double costNew = costOfNode(nearestNode) + 
							distanceRRT( newNode->getCoords(), nearestNode->getCoords() );

			double costTemp = costOfNode(nearNodes[i]) + distanceRRT( newNode->getCoords(),
							 nearNodes[i]->getCoords() );

			if( costTemp < costNew ){

				minNode = nearNodes[i];	
				minIndex = i;
			}
		}
		else{
			freeNearNodes.push_back(false);
		}
	}

	minNode->addChild(newNode);
	newNode->setParent(minNode);

	for( int i=0; i<nearNodes.size(); i++ ){

		if( i!=minIndex ){

			if( (freeNearNodes[i]==true) && (costOfNode(nearNodes[i]) > costOfNode(newNode)+
				distanceRRT(newNode->getCoords(), nearNodes[i]->getCoords() )) ){

				NodeRRT_star* tempParent = nearNodes[i]->getParent();
				newNode->setParent(tempParent);
				tempParent->addChild(newNode);
				tempParent->popChild(nearNodes[i]);	
				// mexPrintf("rewiring edges done\n");
				// mexEvalString("drawnow");
			}
		}
	}

		// for(auto i: nearNodes){

		// 	if(i!=nullptr)
		// 		delete i;
		// }
		// nearNodes.clear();

	// if(newNode!=nullptr)

	if( steerKey==3 ){

		goalNodes_.push_back(newNode);
		return steerKey;
	}

	return steerKey;

}

double costOfNode( NodeRRT_star* nodeIn ){

	double cost = 0;

	NodeRRT_star* traverse = nodeIn;
	while(traverse->getParent()!=nullptr){

		cost+= distanceRRT( traverse->getCoords(), traverse->getParent()->getCoords() );
		traverse= traverse->getParent();
	}

	return cost;
}

double volSphereFn(int numofDOFs){

	double vol;
	switch(numofDOFs){

		case 1:	vol = 2.0;
			break;

		case 2: vol = 3.142;
			break;

		case 3: vol = 4.189;
			break;

		case 4: vol = 4.935;
			break;

		case 5: vol = 5.264;
			break;

		case 6: vol = 5.168;
			break;

		case 7: vol = 4.725;
			break;

		case 8: vol = 4.059;
			break;

		case 9: vol = 3.299;
			break;

		default: vol = -1; printf("numofDOFs more than 9 not allowed \n");
			break;
	}

	return vol;
}


//post processing
double costPath(double*** angles, int* planlength, int numofDOFs, int x_size){
  // (*plan)[i][j] 
  double x0,y0,x1,y1;

  struct EEPos
  {
    double x, y;

    EEPos(double x_, double y_): x(x_), y(y_){}
  };

  std::vector<EEPos> EEpositions;

  for(int j=0; j<*planlength; j++){

	x1 = ((double)x_size)/2.0;
	y1 = 0;

    for(int i = 0; i < numofDOFs; i++)
    {
      //compute the corresponding line segment
      x0 = x1;
      y0 = y1;
      x1 = x0 + LINKLENGTH_CELLS*cos( 2*PI-(*angles)[j][i] );
      y1 = y0 - LINKLENGTH_CELLS*sin( 2*PI-(*angles)[j][i] );
    }

    EEPos temp(x1, y1);
    EEpositions.push_back( temp );
  }

  double cost=0;

  for(int i=0; i< (*planlength)-1; i++ ){

    cost+= sqrt( pow( (EEpositions[i+1].x - EEpositions[i].x),2 ) + 
                pow( (EEpositions[i+1].y - EEpositions[i].y),2 ) );
  }

  return cost;
}
