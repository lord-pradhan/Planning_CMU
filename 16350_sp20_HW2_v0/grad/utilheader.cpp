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

	stackDFS.push(existingNodeIn);

	// mexPrintf("entering same_component while loop \n");
	// mexEvalString("drawnow");

	while(!stackDFS.empty()){

		NodePRM topNode = stackDFS.top();
		stackDFS.pop();

		if (!visited[ topNode.getID() ]){

			visited[ topNode.getID() ] = true;
		}

		for( auto i : topNode.getAdjIDs() ){

			if( !visited[i] ){

				if( pushNodeIn.getID() == i ){
				  
					// mexPrintf("exiting same_component while loop true \n");
					// mexEvalString("drawnow");
					return true;
				}

				else
					stackDFS.push( listOfNodesIn[i] );
			}
		}
	}
	// mexPrintf("exiting same_component while loop false \n");
	// mexEvalString("drawnow");

	return false;
}


bool can_connect( NodePRM pushNodeIn, NodePRM existingNodeIn , double* map, 
	int x_size, int y_size){

	// mexPrintf("Entered can_connect \n");
	// mexEvalString("drawnow");
	int numChecks = 20;

	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< pushNodeIn.getCoords().size() ; j++){

			xVals.push_back( pushNodeIn.getCoords()[j] + (double) (i+1) * 
					( existingNodeIn.getCoords()[j] - pushNodeIn.getCoords()[j] ) / numChecks );
		}

		if(IsValidArmConfiguration( xVals.data(), pushNodeIn.getCoords().size(), map, x_size, y_size)==0 ){
			
			// mexPrintf("Exiting can_connect false \n");
			// mexEvalString("drawnow");
			return false;
		}
	}

	// mexPrintf("Exiting can_connect true\n");
	// mexEvalString("drawnow");
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
	int numChecks = 500;
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

	std::vector<double> xValsPrev = nearestNode_->getCoords();
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
 double eps_, double* map, int x_size, int y_size ){

	// printf("entered extend_star \n");
	NodeRRT* nearestNode = nearestNeighbour( currSamplePt_, tree1_ );

	int returnKey = newConfig_connect( currSamplePt_, nearestNode, newNode_, eps_,  map, x_size, y_size);
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
				 double* map, int x_size, int y_size ){

	// printf("entered newConfig_star \n"); 
	int numChecks = 500;
	double distanceTemp = distanceRRT( nearestNode_->getCoords(), currSamplePt_);
	double ratio_temp = std::min( distanceTemp , eps_ ) / distanceTemp;

	std::vector<double> xEnd;

	for(int j = 0; j< nearestNode_->getCoords().size() ; j++){

		xEnd.push_back(nearestNode_->getCoords()[j] + ratio_temp*( currSamplePt_[j] 
													- nearestNode_->getCoords()[j] ) ) ;
	}

	std::vector<double> xValsPrev = nearestNode_->getCoords();
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
	double eps_, double* map, int x_size, int y_size){
	// '0' = reached, '1' = advanced, '2' = trapped
	// printf("entered connect_star \n");
	int extKey;

	do{ 
		// tail2_ = nullptr;
		NodeRRT* newNode2 = new NodeRRT;
		extKey = extend_connect( tree2_, newNode1_->getCoords(), newNode2, eps_, map, x_size, y_size );

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
}

void swapTrees( NodeRRT* tree1_, NodeRRT* tree2_ ){

	NodeRRT temp = *tree1_;
	*tree1_ = *tree2_;
	*tree2_ = temp;
}



//////////////////////////////////////
///// ******* RRT* ********///////////

NodeRRT_star::NodeRRT_star(): G_val(std::numeric_limits<double>::infinity()) {}

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

	children.erase(std::find(children.begin(),children.end(), popNode));
}


// class for priority queue
NodePQ_star::NodePQ_star( NodeRRT_star* nodeIn_, std::vector<double> currSamplePt_ ): 
nodeIn(nodeIn_), currSamplePt(currSamplePt_) {}

NodePQ_star::NodePQ_star(){}

double NodePQ_star::getDist() const{

	return distanceRRT( currSamplePt, nodeIn->getCoords() );
}

// void NodeRRT_star::setG(double G_val_){G_val = G_val_;}

NodeRRT_star* nearestNeighbour_star( std::vector<double> currSamplePt_, NodeRRT_star* root_ ){

	std::priority_queue< NodePQ_star, std::vector<NodePQ_star>, CompareNN_star > min_queue;

	NodePQ_star nodeTemp( root_, currSamplePt_ );
	min_queue.push( nodeTemp );

	treeDFS_star( root_, currSamplePt_ , min_queue);

	// mexPrintf("size of min_queue is %d \n", min_queue.size());
	// mexEvalString("drawnow");

	return min_queue.top().nodeIn;
}

void treeDFS_star( NodeRRT_star* nodeIn, std::vector<double> currSamplePt_, 
	std::priority_queue< NodePQ_star, std::vector<NodePQ_star>, CompareNN_star > &min_queue ){

	if ( !nodeIn->getChildren().empty() ){

		for (auto i : nodeIn->getChildren()){

			NodePQ_star nodeTemp( i, currSamplePt_ );
			min_queue.push( nodeTemp );
			treeDFS_star(  i, currSamplePt_, min_queue );
		}
		return;
	}
	else {

		return;
	}
}


int newConfig_star( std::vector<double> currSamplePt_, NodeRRT_star* nearestNode_, NodeRRT_star* newNode_ , double eps_, 
 double* map, int x_size, int y_size, std::vector<double> endCoord_, double tol ){

	// mexPrintf("Entered newConfig\n");
	// mexEvalString("drawnow");
	int numChecks = 500;
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

	std::vector<double> xValsPrev = nearestNode_->getCoords();
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


bool can_connect_star( NodeRRT_star* node1, NodeRRT_star* node2 , double* map, int x_size, int y_size){

	// mexPrintf("Entered can_connect \n");
	// mexEvalString("drawnow");
	int numChecks = 200;

	for(int i = 0; i<numChecks; i++){

		std::vector<double> xVals;

		for(int j = 0; j< node1->getCoords().size() ; j++){

			xVals.push_back( node1->getCoords()[j] + (double) (i+1) * 
					( node2->getCoords()[j] - node1->getCoords()[j] ) / numChecks );
		}

		if(IsValidArmConfiguration( xVals.data(), node1->getCoords().size(), map, x_size, y_size)==0 ){
			
			// mexPrintf("Exiting can_connect false \n");
			// mexEvalString("drawnow");
			return false;
		}
	}

	// mexPrintf("Exiting can_connect true\n");
	// mexEvalString("drawnow");
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
}


int extend_star( NodeRRT_star* root_, std::vector<NodeRRT_star*> &goalNodes_, 
	std::vector<double> currSamplePt_, double eps_, double* map, int x_size, int y_size, 
	std::vector<double> endCoord_, double tol_, double nearDist_){

	NodeRRT_star* nearestNode = nearestNeighbour_star( currSamplePt_, root_ );

	NodeRRT_star* newNode = new NodeRRT_star;

	int steerKey = newConfig_star( currSamplePt_, nearestNode, newNode, eps_, map, x_size, y_size,
					endCoord_, tol_ );

	if(steerKey==2)
		return steerKey;

	if( can_connect_star( nearestNode, newNode, map, x_size, y_size ) ){

		NodeRRT_star* minNode = nearestNode;

		std::vector<NodeRRT_star*> nearNodes;

		nearFn( root_, newNode, nearNodes, nearDist_ );

		for(auto i : nearNodes){

			if( can_connect_star( i, newNode, map, x_size, y_size ) ){

				double costTemp = costOfNode(i) + distanceRRT( newNode->getCoords(), i->getCoords() );

				if( costTemp < costOfNode(newNode) ){

					minNode = i;										
				}
			}
		}

		minNode->addChild(newNode);
		newNode->setParent(minNode);

		for( auto i : nearNodes ){

			if( i != minNode ){

				if( can_connect_star(i, newNode, map, x_size, y_size ) && (costOfNode(i) > 
					costOfNode(newNode)+distanceRRT(newNode->getCoords(), i->getCoords())) ){

					NodeRRT_star* tempParent = i->getParent();
					newNode->setParent(tempParent);
					tempParent->addChild(newNode);
					tempParent->popChild(i);	
				}
			}
		}
	}

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